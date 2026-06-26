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
LM_CONSTANT uint32_t kLatPathRayleigh = 2u;       // kGaussian near-pole optimization
LM_CONSTANT uint32_t kLatPathGaussLegacy = 3u;    // kGaussianLegacy
LM_CONSTANT uint32_t kLatPathGenericReject = 4u;  // kGaussian/kUniform/kZigzag/kLaplacian

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

// --- GenRootKernelParams --------------------------------------------------
// Single source for both `gen_root_kernel` (Metal-only, device root sampler)
// and `transit_root_kernel` / `transit_multi_ms_kernel` (Metal + CUDA, device-
// resident frame transit between MS layers). The sun_* / wl_pool_size fields
// are populated by BuildGenRootParams but unused by the transit form.
struct GenRootKernelParams {
  uint32_t gen_seed;      // PCG master seed (== spec.seed [XOR transit/gate nonce])
  uint32_t gen_ray_base;  // running ray-count before this dispatch (PCG global_idx base)
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

struct PcgStream {
  uint32_t seed;
  uint32_t global_idx;
  uint32_t slot;
};

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

// Replicates InitRay_rot + SampleSphericalPointsSph (simulator.cpp:138-150 +
// math.cpp:404/444). All distribution params are pre-converted to radians on
// the host so the kernel does no degree↔radian conversions.
LM_FN void sample_lat_lon_roll(LM_THREAD PcgStream& s, LM_CONSTANT_REF GenRootKernelParams& gp,
                               LM_THREAD float& out_lon, LM_THREAD float& out_lat, LM_THREAD float& out_roll) {
  float phi = 0.0f;
  bool flip = false;
  float lon = 0.0f;
  if (gp.lat_path == kLatPathFullSphere) {
    float u = pcg_uniform(s) * 2.0f - 1.0f;
    u = LM_CLAMP(u, -1.0f, 1.0f);
    phi = LM_ASIN(u);
    lon = pcg_uniform(s) * 2.0f * LM_PI_F;
  } else if (gp.lat_path == kLatPathNoRandom) {
    phi = gp.lat_mean_rad;
  } else if (gp.lat_path == kLatPathRayleigh) {
    float dx = pcg_gaussian(s) * gp.lat_std_rad;
    float dy = pcg_gaussian(s) * gp.lat_std_rad;
    float colatitude = LM_SQRT(dx * dx + dy * dy);
    phi = LM_COPYSIGN(LM_PI_2F - colatitude, gp.lat_mean_rad);
    phi = LM_CLAMP(phi, -LM_PI_2F, LM_PI_2F);
    if (gp.lat_mean_rad < 0.0f) {
      phi = LM_FABS(phi);
      flip = true;
    }
  } else if (gp.lat_path == kLatPathGaussLegacy) {
    float raw = pcg_get_dist(s, kDistGaussianLegacy, gp.lat_mean_rad, gp.lat_std_rad);
    normalize_latitude(raw, phi, flip);
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
