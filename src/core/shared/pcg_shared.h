// Single-source PCG hash + sampling primitives.
// Shared by MSL kernel (src/core/metal/lumice_trace.metal: trace_layer_kernel
// emit gate + gen_root_kernel + transit_root_kernel) and CUDA kernels
// (src/core/backend/cuda_trace_backend.cu: trace_single_ms_kernel emit gate +
// transit_multi_ms_kernel).
//
// Background: scrum-cuda-backend-complete 296.4 (task-cuda-multi-ms) needs
// device-resident continuation sampling on CUDA, mirroring Metal's
// transit_root_kernel form (scrum-267). The PCG hash, orientation sampler,
// triangle area-weighted picker and axis-angle rotation builder used to live
// inline in lumice_trace.metal; this header collapses them to a single source
// so the two backends cannot drift on the hash/orientation math.
//
// Surface contract:
//   - Pointers / references with address-space qualifiers are allowed via
//     the LM_THREAD / LM_DEVICE / LM_CONSTANT_REF shims in lm_shims.h.
//   - No atomics, no threadgroup constructs, no kernel attributes.
//   - Functions live in `namespace lm_pcg`; the Metal shader pulls them in
//     with `using namespace lm_pcg;` right after the #include so existing
//     unqualified call sites continue to compile.
//
// PCG stream semantics: the host derives (seed, global_idx, slot) so that
// every (dispatch, layer, ci, batch, tid, draw) tuple maps to a unique stream
// position. Callers MUST advance `global_idx` between dispatches (the
// gen_ray_base / gate_ray_base pattern in the Metal as-built and the
// transit_ray_count_ / gate_ray_count_ counters in the CUDA backend) — re-using
// the same global_idx across batches collapses two batches onto the same PCG
// sequence and silently kills cross-seed independence (scrum-267.3 lesson).
#ifndef LM_PCG_SHARED_H_
#define LM_PCG_SHARED_H_

#include "lm_shims.h"

#if defined(__METAL_VERSION__)
// MSL: metal_stdlib provides uint32_t / uint16_t / uint8_t typedefs and the
// math functions LM_*  macros expand to. No extra includes required because
// `lumice_trace.metal` already pulls in <metal_stdlib>.
#else
#include <cstdint>
#endif

// Naming note: the functions in this namespace preserve the snake_case
// identifiers used in the original `lumice_trace.metal` definitions (scrum-267
// as-built). Renaming to project-default CamelCase would touch every MSL +
// CUDA call site for zero behavior gain, so the lint check is silenced
// section-wide rather than per function.
// NOLINTBEGIN(readability-identifier-naming)
namespace lm_pcg {

// --- Constants ------------------------------------------------------------
// `lat_path` discriminator (mirrors host SampleSphericalPointsSph code paths,
// math.cpp:404/444). Selecting a path is a host-side decision based on
// axis_dist; the kernel only dispatches.
LM_CONSTANT uint32_t kLatPathFullSphere = 0u;  // axis_dist.IsFullSphereUniform()
LM_CONSTANT uint32_t kLatPathNoRandom = 1u;
LM_CONSTANT uint32_t kLatPathRayleigh = 2u;                // kGaussian near-pole optimization
LM_CONSTANT uint32_t kLatPathGaussLegacy = 3u;             // kGaussianLegacy
LM_CONSTANT uint32_t kLatPathGenericReject = 4u;           // kGaussian/kUniform/kZigzag/kLaplacian
LM_CONSTANT uint32_t kLatPathLaplacianTightEnvelope = 5u;  // kLaplacian near-pole optimization
LM_CONSTANT uint32_t kLatPathLutInverseCdf = 6u;           // unified area-measure inverse-CDF LUT (330.2)

// DistributionType enum values (must match src/core/math.hpp). Crystal-rot
// parity REQUIRES the GenericReject path know the actual proposal type, so
// lat_dist_type is carried separately from lat_path.
LM_CONSTANT uint32_t kDistNoRandom = 0u;
LM_CONSTANT uint32_t kDistUniform = 1u;
LM_CONSTANT uint32_t kDistGaussian = 2u;
LM_CONSTANT uint32_t kDistZigzag = 3u;
LM_CONSTANT uint32_t kDistLaplacian = 4u;
LM_CONSTANT uint32_t kDistGaussianLegacy = 5u;

LM_CONSTANT uint32_t kMaxTriPerKernel = 64u;
LM_CONSTANT int kMaxRejectionAttempts = 1000;

// --- Stream-family nonces -------------------------------------------------
// PCG-stream separation nonce for the per-ray wavelength draw in
// `gen_root_kernel` (both Metal `lumice_trace.metal` and CUDA
// `cuda_trace_backend.cu`, form-for-form mirrored). The wl draw MUST live in
// its own PCG seed domain, independent of the orientation / triangle-pick
// draws that share `(mixed_seed, global_idx)` — otherwise the wl_idx becomes
// correlated with the crystal orientation whenever the orientation stream
// consumes enough slots to reach the wl slot.
//
// Background (task-gpu-wl-stream-decouple-green-tint / issue.md):
// The previous design put wl on `slot = 20` and reused `mixed_seed`, betting
// that the orientation stream never consumed 20 slots. Under a near-pole
// axis distribution (e.g. `zenith` = laplacian mean=0), the acceptance rate
// `cos(phi)/M` → 0 and GenericReject can consume up to
// `kMaxRejectionAttempts = 1000` iterations (× 2 slots for Laplacian) per
// ray — ~10% of rays reach slot 20. The colliding draw then reads the same
// PCG hash as the wl draw, so wl_idx becomes a function of the orientation,
// systematically suppressing the red end in the halo column (empirically
// `metal_lap.green_excess = +3.2` vs `cpu_lap = +0.7`). Seed-domain isolation
// (XOR-mix into the seed) is immune to slot-count consumption, regardless of
// how many slots the orientation loop burns — this is the only fix that does
// not depend on the "orientation never consumes N slots" assumption.
//
// **Nonce uniqueness clearinghouse** — all other stream nonces currently in
// use, so that new nonces (added here or elsewhere) can be picked to not
// collide:
//   kTransitNonce        = 0xA5A5A5A5u  (metal_trace_backend.mm)
//   kCudaGateNonce       = 0x5A5A5A5Au  (cuda_trace_backend.cu)
//   kCudaGenNonce        = 0x3C9A7F11u  (cuda_trace_backend.cu)
//   kMetalShuffleNonce   = 0xB17CA3D9u  (metal_trace_backend.mm; symmetric to
//                                        the CUDA shuffle nonce)
//   kCudaDrainNonce      = 0xD5A1B3C7u  (cuda_trace_backend.cu)
//   kWlStreamNonce       = 0x9E3779B9u  (this header; the 32-bit golden-ratio
//                                        constant — same value used in the
//                                        issue.md causal-verification patch
//                                        that shrank metal_lap.green_excess
//                                        from +3.2 → +1.0)
// All six values are pairwise distinct. `pcg_hash` has good avalanche on any
// non-zero XOR delta, so no additional statistical validation is required.
LM_CONSTANT uint32_t kWlStreamNonce = 0x9E3779B9u;
// (BuildWlStream is defined after PcgStream below.)

// --- GenRootKernelParams --------------------------------------------------
// Single source for both `gen_root_kernel` (Metal-only, device root sampler)
// and `transit_root_kernel` / `transit_multi_ms_kernel` (Metal + CUDA, device-
// resident frame transit between MS layers). The sun_* / wl_pool_size fields
// are populated by BuildGenRootParams but unused by the transit form.
//
// task-gpu-rng-ray-index-uint64: `gen_ray_base` carries only the LOW 32 bits
// of the host-side size_t counter; `gen_ray_base_hi` carries the HIGH 32 bits
// (host splits via `lumice::SplitPcgRayBase` — see trace_backend.hpp). Together
// they let the kernel logically address the full 64-bit ray index space without
// introducing device-side 64-bit integer arithmetic. See `pcg_advance_hi` +
// `pcg_seed_with_high` below for the mixing rule; `high==0` (session under
// 2^32 rays) is bit-exact with the pre-fix stream by construction.
struct GenRootKernelParams {
  uint32_t gen_seed;         // PCG master seed (== spec.seed [XOR transit/gate nonce])
  uint32_t gen_ray_base;     // low 32 bits of running ray-count (may wrap; the mod-2^32 wrap is intentional)
  uint32_t gen_ray_base_hi;  // high 32 bits of running ray-count (0 unless the session accumulated >2^32 rays)
  uint32_t num_rays;
  uint32_t tri_count;
  float sun_lon;           // (sun.azimuth + 180°) in radians
  float sun_lat;           // (-sun.altitude) in radians
  float sun_half_angle;    // (sun.diameter / 2) in radians
  uint32_t wl_pool_size;   // hashes a global ray index into [0, wl_pool_size)
  uint32_t lat_path;       // see kLatPath* above
  uint32_t lat_dist_type;  // axis_dist.latitude_dist.type (cast to uint32_t)
  float lat_mean_rad;
  float lat_std_rad;
  float lat_rejection_m;
  uint32_t lat_lut_n;  // node count for kLatPathLutInverseCdf (330.2); LUT arrays bound separately
  uint32_t az_type;
  float az_mean_rad;
  float az_std_rad;
  float az_pad;
  uint32_t roll_type;
  float roll_mean_rad;
  float roll_std_rad;
  float roll_pad;
};

// --- PCG hash + stream ----------------------------------------------------

LM_FN uint32_t pcg_hash(uint32_t x) {
  x = x * 747796405u + 2891336453u;
  x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
  return (x >> 22u) ^ x;
}

LM_FN float u01_from_hash(uint32_t h) {
  return static_cast<float>(h >> 8) * (1.0f / 16777216.0f);
}

// Precise acceptance ratio for tight-envelope area-measure sampling near the
// pole (doc/near-pole-area-measure-sampling.md §2). Returns sin(theta)/theta;
// the theta->0 limit is 1.0. The 1e-6f guard only fires when a uniform draw
// floor drives the proposed colatitude toward 0 (see kLatPathRayleigh branch
// in sample_lat_lon_roll). Domain is theta in [0, pi]: within it sin(theta) is
// non-negative, so the returned ratio is in [0, 1] and is a valid probability.
LM_FN float pcg_sinc(float theta) {
  return theta > 1e-6f ? LM_SIN(theta) / theta : 1.0f;
}

struct PcgStream {
  uint32_t seed;
  uint32_t global_idx;
  uint32_t slot;
};

// Single-source constructor for the wl PCG stream — see kWlStreamNonce above
// for the design rationale (seed-domain isolation, not slot isolation).
// Metal + CUDA gen_root_kernel both call this instead of hand-writing the
// three-field init, guaranteeing bit-identical wl streams across backends.
LM_FN PcgStream BuildWlStream(uint32_t mixed_seed, uint32_t global_idx) {
  PcgStream s;
  s.seed = mixed_seed ^ kWlStreamNonce;
  s.global_idx = global_idx;
  s.slot = 0u;
  return s;
}

// --- 64-bit ray-index helpers (task-gpu-rng-ray-index-uint64) -------------
// Host provides `base_lo` (low 32 bits) + `base_hi` (high 32 bits) of the
// running per-session PCG ray-base (see `lumice::SplitPcgRayBase` in
// trace_backend.hpp). Each thread's true 64-bit global index is
// `((uint64_t)base_hi << 32) + base_lo + tid`. `pcg_advance_hi` returns this
// thread's "hi epoch": if `base_lo + tid` overflows a uint32 (wraps at 2^32),
// the ray crossed into the next hi epoch, so hi = base_hi + 1; otherwise
// hi = base_hi.
//
// `pcg_seed_with_high` mixes that hi into a seed by XOR-ing `pcg_hash(hi)`.
// Contract: when hi == 0 (every session under 2^32 rays), it is a pure
// no-op — the resulting seed is bit-identical to the input, so the hot-path
// PCG stream is bit-exact with the pre-fix behavior for the entire in-range
// regime. When hi != 0, the mixed seed diverges across hi epochs, preventing
// two rays with the same (base_lo + tid) mod 2^32 from collapsing onto the
// same PCG stream after wrap (the scrum-267.3 silent-under-sampling failure
// mode). The mixing is applied AFTER `stream.seed = gp.gen_seed` — the
// device-side `stream.global_idx` remains `base_lo + tid` (allowed to wrap;
// its post-wrap value coincides with the correct lo half of the epoch).
LM_FN uint32_t pcg_advance_hi(uint32_t base_lo, uint32_t base_hi, uint32_t tid) {
  uint32_t lo = base_lo + tid;
  uint32_t carry = (lo < base_lo) ? 1u : 0u;
  return base_hi + carry;
}

LM_FN uint32_t pcg_seed_with_high(uint32_t seed, uint32_t hi) {
  if (hi == 0u) {
    return seed;
  }
  return seed ^ pcg_hash(hi);
}

LM_FN float pcg_uniform(LM_THREAD PcgStream& s) {
  uint32_t h = pcg_hash(s.seed ^ pcg_hash(s.global_idx * 1000003u + s.slot));
  s.slot++;
  return u01_from_hash(h);
}

// Box-Muller standard normal. u1 floored to avoid log(0).
LM_FN float pcg_gaussian(LM_THREAD PcgStream& s) {
  float u1 = LM_FMAX(pcg_uniform(s), 1e-7f);
  float u2 = pcg_uniform(s);
  return LM_SQRT(-2.0f * LM_LOG(u1)) * LM_COS(2.0f * LM_PI_F * u2);
}

// Gamma(shape=2, scale=b) via the sum-of-two-Exp(1/b) closed form:
//   theta = -b * (ln u1 + ln u2), u1,u2 ~ U(0,1)
// Used by the Laplacian tight-envelope area-measure sampler
// (doc/near-pole-area-measure-sampling.md §2.1): theta = colatitude ~ Gamma(2,b)
// paired with sin(theta)/theta acceptance is EXACT (M=1) for the target
// p(theta) ∝ exp(-theta/b) * sin(theta). u_i floored at 1e-7 (same as
// pcg_gaussian) to keep log() finite. Consumes 2 uniform slots per call.
LM_FN float pcg_gamma2(LM_THREAD PcgStream& s, float b) {
  float u1 = LM_FMAX(pcg_uniform(s), 1e-7f);
  float u2 = LM_FMAX(pcg_uniform(s), 1e-7f);
  return -b * (LM_LOG(u1) + LM_LOG(u2));
}

// Mirrors RandomNumberGenerator::Get (math.cpp:365-389). mean / std are in
// the SAME unit the host caller uses for the result; SampleSphericalPointsSph
// always pre-converts axis_dist.{*}.mean/std to radians here, so the radian
// envelope semantics hold.
LM_FN float pcg_get_dist(LM_THREAD PcgStream& s, uint32_t dtype, float mean, float std_val) {
  if (dtype == kDistNoRandom) {
    return mean;
  }
  if (dtype == kDistUniform) {
    return (pcg_uniform(s) - 0.5f) * std_val + mean;
  }
  if (dtype == kDistGaussian || dtype == kDistGaussianLegacy) {
    return pcg_gaussian(s) * std_val + mean;
  }
  if (dtype == kDistZigzag) {
    return LM_FABS(std_val * LM_SIN(pcg_uniform(s) * 2.0f * LM_PI_F) + mean);
  }
  // kLaplacian: inverse CDF.
  float u = pcg_uniform(s);
  float sgn = (u < 0.5f) ? -1.0f : 1.0f;
  float arg = LM_FMAX(1.0f - 2.0f * LM_FABS(u - 0.5f), 1e-30f);
  return mean - std_val * sgn * LM_LOG(arg);
}

// Mirrors detail::NormalizeLatitude (math.cpp:542-553).
LM_FN void normalize_latitude(float phi, LM_THREAD float& phi_out, LM_THREAD bool& flip) {
  float theta = LM_PI_2F - phi;
  theta = LM_FMOD(theta, 2.0f * LM_PI_F);
  if (theta < 0.0f) {
    theta += 2.0f * LM_PI_F;
  }
  flip = theta > LM_PI_F;
  if (flip) {
    theta = 2.0f * LM_PI_F - theta;
  }
  phi_out = LM_PI_2F - theta;
}

// Unified area-measure latitude sampler (330.2 core-lut-sampler; design:
// doc/near-pole-area-measure-sampling.md + explore unify-orientation-sampling-cosine-measure).
// Numerical inverse-CDF over a uniform-theta node table: theta_nodes[i] / cdf_nodes[i]
// for i in [0, n_nodes-1], with cdf_nodes STRICTLY increasing (BuildLatLut enforces the
// monotonicity lift; both the search predicate and the interpolation denominator rely on
// it). Given a uniform draw xi, returns colatitude theta = F^-1(xi) by binary search +
// linear interpolation.
//
// RNG-agnostic single source: the host CPU sampler (RandomNumberGenerator) and the device
// gen/transit kernels (PcgStream) each draw their own xi and call THIS one function — the
// mirrored rejection loops disappear. Pointers are LM_DEVICE so the table lives in a Metal
// `device` buffer bound separately (NOT embedded in the setBytes GenRootKernelParams
// struct, which cannot carry device pointers) / a plain CUDA/host pointer.
//
// Node placement is uniform-theta (NOT equal-xi) with a fixed binary search: explore
// exp3/exp5 showed equal-xi O(1) indexing smears the extreme tail and a guide table has a
// warp-stalling worst case, while uniform-theta + binary search is tail-exact at N=256.
// With (n_nodes-1) a power of two (256 intervals) the `hi-lo>1` loop runs a constant 8
// iterations for every xi → warp-uniform, no control divergence. xi is clamped into the
// representable CDF span so the (possibly lifted) endpoints and xi==1.0 resolve cleanly;
// the denom>0 guard makes a degenerate (bit-equal) bin return its lower node instead of NaN.
LM_FN float invert_lat_lut(float xi, LM_DEVICE const float* theta_nodes, LM_DEVICE const float* cdf_nodes,
                           uint32_t n_nodes) {
  xi = LM_CLAMP(xi, cdf_nodes[0], cdf_nodes[n_nodes - 1u]);
  uint32_t lo = 0u;
  uint32_t hi = n_nodes - 1u;
  while (hi - lo > 1u) {
    uint32_t mid = (lo + hi) >> 1u;
    if (cdf_nodes[mid] <= xi) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  float c0 = cdf_nodes[lo];
  float c1 = cdf_nodes[lo + 1u];
  float denom = c1 - c0;
  float w = denom > 0.0f ? (xi - c0) / denom : 0.0f;
  return theta_nodes[lo] + w * (theta_nodes[lo + 1u] - theta_nodes[lo]);
}

// Flip-table bin index for a colatitude sampled from the uniform-theta LUT (330.2).
// theta_nodes are UNIFORMLY spaced over [theta_nodes[0], theta_nodes[n-1]], so the
// containing interval is a direct O(1) computation (no search). Returns an index in
// [0, n_nodes-2]; the caller reads flip_prob[idx] and draws a Bernoulli flip bit. Host and
// device compute the same index this way, keeping the flip semantics single-sourced.
LM_FN uint32_t lat_lut_bin(float theta, LM_DEVICE const float* theta_nodes, uint32_t n_nodes) {
  float span = theta_nodes[n_nodes - 1u] - theta_nodes[0];
  float t = span > 0.0f ? (theta - theta_nodes[0]) / span : 0.0f;
  int idx = static_cast<int>(t * static_cast<float>(n_nodes - 1u));
  idx = idx < 0 ? 0 : idx;
  int last = static_cast<int>(n_nodes) - 2;
  idx = idx > last ? last : idx;
  return static_cast<uint32_t>(idx);
}

// Replicates InitRay_rot + SampleSphericalPointsSph (simulator.cpp:138-150 +
// math.cpp:404/444). All distribution params are pre-converted to radians on
// the host so the kernel does no degree↔radian conversions.
//
// `out_attempts` (scrum-328.2 Step 1 attempt-count observability, plan §5):
// non-null → the caller receives this ray's rejection-loop iteration count
// (1 for direct/analytic paths kFullSphere/kNoRandom/kRayleigh/kGaussLegacy;
// N for kLatPathGenericReject, capped at kMaxRejectionAttempts). The caller
// uses `mean(attempts)` as the empirical `1/accept_ratio` — the ONLY route
// to a per-ray acceptance-rate observation on device (the host-side
// SelectLatPath / ComputeJacobianEnvelope decision is deterministic and
// cannot substitute). null → no write, hot-path cost is one branch predicted
// off; production keeps passing null so the observation is compiled out at
// the call site.
LM_FN void sample_lat_lon_roll(LM_THREAD PcgStream& s, LM_CONSTANT_REF GenRootKernelParams& gp,
                               LM_DEVICE const float* lat_lut_theta, LM_DEVICE const float* lat_lut_cdf,
                               LM_DEVICE const float* lat_lut_flip, LM_THREAD float& out_lon, LM_THREAD float& out_lat,
                               LM_THREAD float& out_roll, LM_THREAD int* out_attempts) {
  float phi = 0.0f;
  bool flip = false;
  float lon = 0.0f;
  int attempts_total = 1;  // direct-path default; kLatPathGenericReject overwrites below
  if (gp.lat_path == kLatPathFullSphere) {
    float u = pcg_uniform(s) * 2.0f - 1.0f;
    u = LM_CLAMP(u, -1.0f, 1.0f);
    phi = LM_ASIN(u);
    lon = pcg_uniform(s) * 2.0f * LM_PI_F;
  } else if (gp.lat_path == kLatPathNoRandom) {
    phi = gp.lat_mean_rad;
  } else if (gp.lat_path == kLatPathRayleigh) {
    // Tight-envelope area-measure sampling (doc/near-pole-area-measure-sampling.md
    // §2). Propose colatitude ~ Rayleigh(sigma) via the 2D-Gaussian norm form
    // (numerically identical to theta = sigma*sqrt(-2 ln u)); accept with
    // sin(theta)/theta (M=1, exact, never clamp). Consumes 2 gaussian slots +
    // 1 uniform slot per attempt; expected ~1.002 attempts for sigma <= 60°
    // (doc §附录). attempts_total is populated so the scrum-328.2 attempt-count
    // observability facility (EnableGenAttemptCountForTest / ReadbackGenAttemptCountForTest)
    // measures the tight-envelope acceptance rate directly.
    int attempts = 0;
    float colatitude = 0.0f;
    bool accept = false;
    do {
      float dx = pcg_gaussian(s) * gp.lat_std_rad;
      float dy = pcg_gaussian(s) * gp.lat_std_rad;
      colatitude = LM_SQRT(dx * dx + dy * dy);
      attempts++;
      if (attempts >= kMaxRejectionAttempts) {
        break;
      }
      accept = pcg_uniform(s) < pcg_sinc(colatitude);
    } while (!accept);
    attempts_total = attempts;
    phi = LM_COPYSIGN(LM_PI_2F - colatitude, gp.lat_mean_rad);
    phi = LM_CLAMP(phi, -LM_PI_2F, LM_PI_2F);
    if (gp.lat_mean_rad < 0.0f) {
      phi = LM_FABS(phi);
      flip = true;
    }
  } else if (gp.lat_path == kLatPathLaplacianTightEnvelope) {
    // Tight-envelope area-measure sampling for Laplacian near-pole
    // (doc/near-pole-area-measure-sampling.md §2.1). Propose colatitude ~
    // Gamma(2, b) via pcg_gamma2 (closed form theta = -b(ln u1 + ln u2)); accept
    // with sin(theta)/theta (M=1, exact, never clamp). Structurally mirrors the
    // kLatPathRayleigh branch above — same copysign/clamp/flip semantics for
    // colatitude → phi (they depend only on the geometric fold, not on which
    // proposal distribution generated theta). Consumes 2 uniform slots per
    // attempt (pcg_gamma2) + 1 uniform slot (accept draw); expected ~1.007
    // attempts for b <= 60° (scrum-328.4 Step 1 calibration, exp4).
    int attempts = 0;
    float colatitude = 0.0f;
    bool accept = false;
    do {
      colatitude = pcg_gamma2(s, gp.lat_std_rad);
      attempts++;
      if (attempts >= kMaxRejectionAttempts) {
        break;
      }
      accept = pcg_uniform(s) < pcg_sinc(colatitude);
    } while (!accept);
    attempts_total = attempts;
    phi = LM_COPYSIGN(LM_PI_2F - colatitude, gp.lat_mean_rad);
    phi = LM_CLAMP(phi, -LM_PI_2F, LM_PI_2F);
    if (gp.lat_mean_rad < 0.0f) {
      phi = LM_FABS(phi);
      flip = true;
    }
  } else if (gp.lat_path == kLatPathGaussLegacy) {
    float raw = pcg_get_dist(s, kDistGaussianLegacy, gp.lat_mean_rad, gp.lat_std_rad);
    normalize_latitude(raw, phi, flip);
  } else if (gp.lat_path == kLatPathLutInverseCdf) {
    // Unified area-measure inverse-CDF LUT (330.2). One uniform draw + fixed binary search (no
    // rejection loop, attempts_total stays 1); flip via the per-bin probability reproduces the
    // pole-crossing azimuth flip. Same single-source invert_lat_lut / lat_lut_bin as the CPU
    // sampler (math.cpp). The three LUT arrays are bound as separate device buffers (they cannot
    // live inside the setBytes-uploaded GenRootKernelParams struct).
    float xi = pcg_uniform(s);
    float colatitude = invert_lat_lut(xi, lat_lut_theta, lat_lut_cdf, gp.lat_lut_n);
    phi = LM_PI_2F - colatitude;  // colatitude-from-zenith (theta_z in [0,pi]) -> latitude in [-pi/2,pi/2]
    uint32_t bin = lat_lut_bin(colatitude, lat_lut_theta, gp.lat_lut_n);
    flip = pcg_uniform(s) < lat_lut_flip[bin];
  } else {
    // kLatPathGenericReject (math.cpp:503-517).
    int attempts = 0;
    bool accept = false;
    do {
      float raw = pcg_get_dist(s, gp.lat_dist_type, gp.lat_mean_rad, gp.lat_std_rad);
      normalize_latitude(raw, phi, flip);
      attempts++;
      if (attempts >= kMaxRejectionAttempts) {
        break;
      }
      float accept_u = pcg_uniform(s);
      accept = accept_u < LM_COS(phi) / gp.lat_rejection_m;
    } while (!accept);
    attempts_total = attempts;
  }
  if (gp.lat_path != kLatPathFullSphere) {
    lon = pcg_get_dist(s, gp.az_type, gp.az_mean_rad, gp.az_std_rad);
  }
  float roll = pcg_get_dist(s, gp.roll_type, gp.roll_mean_rad, gp.roll_std_rad);
  if (flip) {
    lon += LM_PI_F;
    roll += LM_PI_F;
  }
  out_lon = lon;
  out_lat = phi;
  out_roll = roll;
  if (out_attempts != nullptr) {
    *out_attempts = attempts_total;
  }
}

// --- Crystal rotation builder ---------------------------------------------

LM_FN void axis_angle_rotation_9(LM_THREAD const float* ax, float theta, LM_THREAD float* out) {
  float c = LM_COS(theta);
  float s = LM_SIN(theta);
  float cc = 1.0f - c;
  out[0] = ax[0] * ax[0] * cc + c;
  out[1] = ax[0] * ax[1] * cc - ax[2] * s;
  out[2] = ax[0] * ax[2] * cc + ax[1] * s;
  out[3] = ax[0] * ax[1] * cc + ax[2] * s;
  out[4] = ax[1] * ax[1] * cc + c;
  out[5] = ax[1] * ax[2] * cc - ax[0] * s;
  out[6] = ax[0] * ax[2] * cc - ax[1] * s;
  out[7] = ax[1] * ax[2] * cc + ax[0] * s;
  out[8] = ax[2] * ax[2] * cc + c;
}

LM_FN void chain_left_mul_9(LM_THREAD float* m, LM_THREAD const float* r) {
  // m <- r * m
  float t[9];
  for (uint32_t i = 0u; i < 3u; i++) {
    for (uint32_t j = 0u; j < 3u; j++) {
      t[i * 3 + j] = r[i * 3 + 0] * m[0 * 3 + j] + r[i * 3 + 1] * m[1 * 3 + j] + r[i * 3 + 2] * m[2 * 3 + j];
    }
  }
  for (uint32_t k = 0u; k < 9u; k++) {
    m[k] = t[k];
  }
}

// Computes R = Rz(lon - π) · Ry(lat - π/2) · Rz(roll) using individual axis
// rotations chained via Rotation::Chain (geo3d.cpp:32-46), matching
// BuildCrystalRotation in simulator.cpp:128-135. Stored row-major:
// mat9[i*3+j] = R_{ij}, identical to Rotation::mat_.
LM_FN void build_crystal_rotation_9(float lon, float lat, float roll, LM_THREAD float* mat9) {
  float ey[3] = { 0.0f, 1.0f, 0.0f };
  float ez[3] = { 0.0f, 0.0f, 1.0f };
  axis_angle_rotation_9(ez, roll, mat9);
  float middle[9];
  axis_angle_rotation_9(ey, lat - LM_PI_2F, middle);
  chain_left_mul_9(mat9, middle);
  float outer[9];
  axis_angle_rotation_9(ez, lon - LM_PI_F, outer);
  chain_left_mul_9(mat9, outer);
}

// d_crystal = R^T · d_world. Mirrors Rotation::ApplyInverse using the row-major
// mat_ layout (geo3d.cpp:77-87) — read column k of R = row k transposed.
LM_FN void apply_inverse_mat9(LM_THREAD const float* mat9, LM_THREAD const float* d_world, LM_THREAD float* d_crystal) {
  d_crystal[0] = mat9[0] * d_world[0] + mat9[3] * d_world[1] + mat9[6] * d_world[2];
  d_crystal[1] = mat9[1] * d_world[0] + mat9[4] * d_world[1] + mat9[7] * d_world[2];
  d_crystal[2] = mat9[2] * d_world[0] + mat9[5] * d_world[1] + mat9[8] * d_world[2];
}

// SampleTrianglePoint (geo3d.cpp:153-168). vtx9 = 3 vertices × 3 coords.
// `vtx9` lives in device memory (the per-session triangle pool) on both MSL
// and CUDA; LM_DEVICE expands to `device` on MSL and to empty on CUDA/host.
LM_FN void sample_triangle(LM_THREAD PcgStream& s, LM_DEVICE const float* vtx9, LM_THREAD float* out_p) {
  float u = pcg_uniform(s);
  float v = pcg_uniform(s);
  if (u + v > 1.0f) {
    u = 1.0f - u;
    v = 1.0f - v;
  }
  for (uint32_t k = 0u; k < 3u; k++) {
    float a = vtx9[k];
    float b = vtx9[3 + k];
    float c = vtx9[6 + k];
    out_p[k] = u * (b - a) + v * (c - a) + a;
  }
}

// SampleSphCapPoint (geo3d.cpp:171-205). Inputs already in radians. Single source
// for the MSL + CUDA gen_root_kernel (296.6): samples an incident direction
// within a cone of half_angle around the sun direction (lon, lat).
LM_FN void sample_sph_cap(LM_THREAD PcgStream& s, float lon, float lat, float half_angle, LM_THREAD float* out_d) {
  float c_cap = LM_COS(half_angle);
  float u = pcg_uniform(s);
  float x = u + (1.0f - u) * c_cap;
  float r = LM_SQRT(LM_FMAX(1.0f - x * x, 0.0f));
  float phi = pcg_uniform(s) * 2.0f * LM_PI_F;
  float y = LM_COS(phi) * r;
  float z = LM_SIN(phi) * r;
  float c_lon = LM_COS(lon);
  float s_lon = LM_SIN(lon);
  float c_lat = LM_COS(lat);
  float s_lat = LM_SIN(lat);
  out_d[0] = c_lon * c_lat * x - s_lon * y - c_lon * s_lat * z;
  out_d[1] = s_lon * c_lat * x + c_lon * y - s_lon * s_lat * z;
  out_d[2] = s_lat * x + c_lat * z;
}

// Device-side bijection on [0, n) — 4-round balanced Feistel with cycle-walk
// for non-power-of-2 n. Used by Metal/CUDA continuation-pool shuffle to remove
// the per-parent-CI correlation that legacy host Fisher-Yates breaks
// (explore-300; task-gpu-backend-recombine-shuffle).
//
// Each thread `i in [0, n)` computes `src = feistel_bijection(i, n, seed)`
// independently — no atomics, no host RNG, no sort. The output is a permutation
// of [0, n). cycle-walk: when the Feistel domain p = next_pow2(n) > n, iterate
// until the result falls in [0, n); for n in (p/2, p], expected iterations < 2.
//
// Small-n special cases: n<=1 returns i (trivial); n==2 returns i^1 (the
// p=h=1 case degenerates to all-zero output under the general Feistel form
// because the half-bit width = 0). The n==2 permutation is seed-INDEPENDENT —
// a 2-element set has exactly one non-trivial permutation (the swap), so this
// is a mathematical limit, not a missing seed dependency.
//
// Supported domain: n up to 2^30 (~1.07e9), far above any continuation-pool
// size (ray counts are in the millions at most). The bit-width loop is capped
// at 30 to keep `1u << bits` clear of the uint32 shift-UB boundary (bits==32).
LM_FN uint32_t feistel_bijection(uint32_t i, uint32_t n, uint32_t seed) {
  if (n <= 1u) {
    return i;
  }
  if (n == 2u) {
    return i ^ 1u;
  }
  // Balanced Feistel needs an EVEN bit-width domain so each half has equal
  // bit-width (else the split L|R wouldn't cover the whole p domain — e.g.
  // n=5 → next_pow2=8 has 3 bits, halves of 1+1=2 bits cover only 4 elements).
  // Round bit-width up to even (equivalent to next power-of-4 domain).
  // Cap at 30 so `1u << bits` never reaches the uint32 shift-UB boundary
  // (bits==32). n > 2^30 is unsupported (see domain note above); it would
  // degrade gracefully to the cycle-walk fallback rather than invoke UB.
  uint32_t bits = 0u;
  while (bits < 30u && (1u << bits) < n) {
    bits++;
  }
  if ((bits & 1u) != 0u) {
    bits++;
  }
  uint32_t half_bits = bits >> 1u;
  uint32_t hm = (1u << half_bits) - 1u;
  uint32_t round_const[4] = { 0x9E3779B9u, 0x85EBCA6Bu, 0xC2B2AE35u, 0x27D4EB2Fu };
  uint32_t cur = i;
  // Cycle-walk: at most O(p/n) iterations expected (p ≤ 4n by pow-of-4 rounding;
  // typical n=10^3-10^4 in our cont-pool use case gives ratio ≈ 1-1.6). Guard
  // bound 64 chosen for statistical safety — verified by Python harness in
  // scratchpad/task-gpu-backend-recombine-shuffle (n ∈ {2..100003}, 5 seeds).
  for (uint32_t guard = 0u; guard < 64u; guard++) {
    uint32_t L = (cur >> half_bits) & hm;
    uint32_t R = cur & hm;
    for (uint32_t k = 0u; k < 4u; k++) {
      uint32_t f = pcg_hash(seed ^ R ^ round_const[k]) & hm;
      uint32_t new_R = L ^ f;
      L = R;
      R = new_R;
    }
    uint32_t out = (L << half_bits) | R;
    if (out < n) {
      return out;
    }
    cur = out;
  }
  // Defensive fallback: mathematically unreachable for supported n (a finite
  // Feistel cycle always returns to [0,n); the Python harness measured a max
  // cycle-walk depth of 26, well under the guard of 64). If it IS reached, the
  // permutation contract is already broken — `cur % n` is NOT a bijection, so
  // the shuffle would silently double-count / drop rays. We still clamp into
  // [0,n) here (rather than return `cur`) so a broken state cannot cause an
  // out-of-bounds gather read on the GPU; the energy imbalance would surface
  // in parity tests.
  return cur % n;
}

// RandomSample (geo3d.cpp:112-150) — categorical CDF with negative-weight clip.
// Mirrors host behavior: non-positive total falls back to bin 0.
LM_FN uint32_t categorical_sample(LM_THREAD const float* weights, uint32_t n, float u_in) {
  float total = 0.0f;
  for (uint32_t i = 0u; i < n; i++) {
    total += LM_FMAX(weights[i], 0.0f);
  }
  if (total <= 0.0f) {
    return 0u;
  }
  float target = u_in * total;
  float cumsum = 0.0f;
  for (uint32_t i = 0u; i < n; i++) {
    cumsum += LM_FMAX(weights[i], 0.0f);
    if (cumsum > target) {
      return i;
    }
  }
  return n - 1u;
}

}  // namespace lm_pcg
// NOLINTEND(readability-identifier-naming)

#endif  // LM_PCG_SHARED_H_
