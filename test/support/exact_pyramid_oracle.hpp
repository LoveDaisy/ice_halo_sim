// Exact-integer pyramid vertex oracle — header-only test support.
//
// Ground truth for the closed-form-pyramid work. Unlike the prism oracle, the
// pyramid coefficients are not scalable to a single integer basis by a fixed
// substitution — a1 spans [7.56e-4, 248] over the legal wedge range, so the
// substitution trick (x = √3·u → tiny integer rows) doesn't work uniformly.
//
// Approach: dynamic uniform shift computed at runtime per plane set. For each
// coefficient f = m · 2^exp (m ∈ [0.5, 1), 24-bit float mantissa), lossless
// integer capture requires shift ≥ 24 − exp; the shift for the whole set is
// max(24 − exp_i). "shift = 24 hardcoded" (initial scaffold) was lossy — it
// mistook "24 mantissa bits" for "24 fractional bits" and truncated any coef
// with exp < 0 (measured: 66% of pyramid coefs, breaking exact 6-plane apex
// concurrence and splitting one apex into a cluster of ~6 nearby points → a
// regular hex pyramid returned vtx=36 instead of 14). Dynamic shift restores
// bit-exact input capture. Budget guard: if the shift + magnitude combination
// would push worst-case __int128 intermediates over the signed 127-bit limit
// (specifically the 4·coef_bits + 3 bound in the IsFeasible cross-multiply),
// the oracle sets `refused = true` and returns an empty verdict — silent
// downgrade to a lossy answer is worse than no answer (a01: no ground truth
// worth the name can quietly wrong itself).
//
// Independence discipline:
//   - This oracle is a SECOND, structurally-independent implementation from
//     the closed-form. It enumerates every C(n,3) plane triple with generic
//     3×3 Cramer arithmetic; it does NOT bake in the "six fixed directions"
//     insight, the "uniform erosion" model, or the "adjacent-only death"
//     assumption. A bug in the closed-form's structural reasoning cannot
//     also be a bug here.
//   - Feasibility uses exact cross-multiplied comparisons against every
//     plane; no epsilon threshold anywhere.
//   - Dedup uses SamePointFuzzy at 5e-6 (~80× float ULP). Bit-exact rational
//     equality (SamePointExact, retained for reference) was verified to
//     over-count multi-plane concurrences on float-stored plane inputs:
//     algebraic identities like `2·(a1·√3/8) = a1·√3/4` that hold in exact
//     math do NOT survive float coefficient rounding, so two triples targeting
//     the "same" 4-plane shoulder corner produce rationals that reduce to
//     distinct canonical forms. The tolerance is bounded by input precision
//     (float ULP), not an ad-hoc knob — a01/a02 concerns about "no epsilon in
//     ground truth" are about feasibility/incidence checks (still exact here);
//     dedup consolidation of points that were the SAME real-number vertex
//     under exact math but became split by float rounding is a different
//     operation, and required for correctness on the actual input population.
//     Measured effect: 0/175 agreement with closed-form under SamePointExact
//     (see progress note); dedup-scale tolerance restores parity.
//
// The primary consumer is the golden-analytic pyramid test; a
// cross-verification against the Python `Fraction`-based oracle (out of
// band) is expected to complete the two-independent-oracles bar.

#ifndef LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_
#define LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_

#include <cmath>
#include <cstdint>

namespace lumice {
namespace test_support {

// Cap on total planes fed to the oracle (2 basal + 6 prism + 6 upper cone +
// 6 lower cone = 20 for the pyramid family). Kept as a compile-time constant
// so we can size internal arrays without heap allocation — matches
// kMaxHexCrystalPlanes.
constexpr int kExactPyramidMaxPlanes = 20;
// Upper bound on distinct vertices: matches kClosedFormPyramidMaxVtx + slack.
constexpr int kExactPyramidMaxVtx = 40;

struct ExactPyramidVertex {
  // Rational representation: pos = (px, py, pz) / det, canonicalized so
  // det > 0 (sign folded into px/py/pz).
  __int128 px = 0;
  __int128 py = 0;
  __int128 pz = 0;
  __int128 det = 1;
  // Per-plane bitmask: bit k is set iff this vertex lies on plane index k
  // (either because that plane was one of the 3 defining planes, or because
  // extra planes concur here). Written at solve time — this is a witness of
  // provenance, not a downstream inference.
  uint64_t plane_incidence_mask = 0;  // planes 0..19 (kExactPyramidMaxPlanes ≤ 64 bits)
};

struct ExactPyramidVerdict {
  int vertex_count = 0;
  ExactPyramidVertex vertices[kExactPyramidMaxVtx];
  // Per-plane: how many distinct vertices lie on this plane. face_present iff
  // ≥ 3 (a 2D polygon on the plane).
  int face_vertex_count[kExactPyramidMaxPlanes] = {};
  bool face_present[kExactPyramidMaxPlanes] = {};
  // Refusal flag: set iff the input's required shift or the resulting
  // intermediate bit widths would exceed the __int128 budget. When true, the
  // vertex/face arrays are meaningless (empty) — the caller must NOT treat
  // this as "no vertices found". Downstream ground-truth users should route
  // refused samples to the Python `Fraction` oracle (unlimited precision) or
  // exclude them from the batch.
  bool refused = false;
  // Diagnostic: shift used for this call, and worst observed intermediate bit
  // width. Populated even when refused (to help sizing follow-up work).
  int shift = 0;
  int max_intermediate_bits = 0;
};

namespace exact_pyramid_detail {

// Signed __int128 has 127 bits of magnitude + 1 sign bit. Worst-case
// intermediate in ExactPyramid is the IsFeasible cross-multiply:
//   sum_{col} plane[k][col] · point_num_col + plane[k][3] · det
// where point_num_col and det are each Cramer-style 3×3 combinations of
// scaled coefs (up to 3·coef_bits + 3 bits each). The dominant term thus has
// coef_bits + 3·coef_bits + 3 = 4·coef_bits + 3 bits, plus ~2 bits from the
// 4-term sum. Cap at 126 (leave 1 bit for sign + 1 for the sum) → coef_bits
// budget = (126 − 5) / 4 = 30.
constexpr int kMaxCoefBits = 30;
constexpr int kMaxIntermediateBits = 126;

// Noise-floor + required-shift analysis of one input coefficient set.
// Motivation: cos/sin/algebraic-cancellation in the producer can leak sub-ULP
// noise (e.g., x1 − x2 where x1 == x2 by symmetry produces 1e-16 instead of
// exact 0, from libm's rounded transcendentals). Feeding such a coef to the
// naïve RequiredShift asks for shift ≈ 76 to capture it losslessly and pins
// the oracle to a refuse verdict for otherwise well-conditioned inputs (the
// regular hex pyramid trip-wired here). The fix: any coef whose magnitude is
// smaller than 1 ULP of the input's max coef (≈ 2^-24 · max_abs) contributes
// less than a rounding-error's worth to any product and is geometrically
// zero — snap it out before computing shift.
struct CoefAnalysis {
  int shift = 0;
  float noise_floor = 0.0f;  // |coef| < noise_floor is treated as 0
};

inline CoefAnalysis AnalyzeCoefs(int plane_cnt, const float* plane_coef) {
  CoefAnalysis out;
  float max_abs = 0.0f;
  for (int i = 0; i < plane_cnt * 4; i++) {
    float f = std::fabs(plane_coef[i]);
    if (f > max_abs) {
      max_abs = f;
    }
  }
  if (max_abs == 0.0f) {
    return out;
  }
  // 2^-24 relative to the largest coef: at or below this, a coef is within
  // one float ULP of the largest coef and contributes nothing more than
  // rounding noise to any product — treat as geometric zero.
  out.noise_floor = std::ldexp(max_abs, -24);
  // Compute the minimum uniform shift K such that every above-noise
  // coefficient f_i, when scaled to `round(f_i · 2^K)`, is captured with zero
  // truncation. For f = m · 2^exp (frexp), K ≥ 24 − exp makes f · 2^K integer.
  // Take the max across coefs — one basis for the whole plane set.
  int max_shift = 0;
  for (int i = 0; i < plane_cnt * 4; i++) {
    float af = std::fabs(plane_coef[i]);
    if (af == 0.0f || af <= out.noise_floor) {
      continue;
    }
    int exp = 0;
    std::frexp(af, &exp);
    int need = 24 - exp;
    if (need < 0) {
      need = 0;  // integer or larger already
    }
    if (need > max_shift) {
      max_shift = need;
    }
  }
  out.shift = max_shift;
  return out;
}

// Scale a float losslessly using the shift returned by RequiredShift.
// std::ldexp gives exact 2^shift multiplication; llround snaps the float
// mantissa to its integer image (guaranteed exact when shift = 24 − exp for
// that coef; larger shifts still produce integers since we only add trailing
// zero bits). Zero coefs stay zero regardless.
inline __int128 FloatToScaled(float f, int shift) {
  if (f == 0.0f) {
    return 0;
  }
  return static_cast<__int128>(std::llround(std::ldexp(static_cast<double>(f), shift)));
}

// Bit width of a signed __int128 magnitude (0 → 0, ±1 → 1, ±2..3 → 2, …).
// Used to enforce the intermediate-bit-width budget before every risky
// multiplication.
inline int BitWidth128(__int128 x) {
  if (x < 0) {
    x = -x;
  }
  if (x == 0) {
    return 0;
  }
  int b = 0;
  __int128 v = x;
  while (v != 0) {
    v >>= 1;
    b++;
  }
  return b;
}

// Overflow-checked cross-multiplied feasibility test. Returns tri-state:
//   +1 = infeasible (some plane's LHS strictly > RHS)
//    0 = feasible (all planes satisfied)
//   -1 = refuse (an intermediate bit width exceeded the budget)
// Also updates *max_bits with the widest observed intermediate. Runs on
// reduced (px, py, pz, det) so worst-case magnitude is much smaller than
// SolveTriple's raw output, but still guarded — for adversarial coefs the
// reduced representation can be near-raw.
inline int IsFeasibleGuarded(int n, const __int128 planes[][4], __int128 px, __int128 py, __int128 pz, __int128 det,
                             int* max_bits) {
  for (int k = 0; k < n; k++) {
    __int128 t0 = planes[k][0] * px;
    __int128 t1 = planes[k][1] * py;
    __int128 t2 = planes[k][2] * pz;
    __int128 rhs = -planes[k][3] * det;
    int w0 = BitWidth128(t0);
    int w1 = BitWidth128(t1);
    int w2 = BitWidth128(t2);
    int wr = BitWidth128(rhs);
    int worst = w0;
    if (w1 > worst) {
      worst = w1;
    }
    if (w2 > worst) {
      worst = w2;
    }
    if (wr > worst) {
      worst = wr;
    }
    if (worst > *max_bits) {
      *max_bits = worst;
    }
    if (worst >= kMaxIntermediateBits) {
      return -1;  // refuse: single-term overflow risk
    }
    __int128 lhs = t0 + t1 + t2;
    int wl = BitWidth128(lhs);
    if (wl > *max_bits) {
      *max_bits = wl;
    }
    if (wl >= kMaxIntermediateBits) {
      return -1;  // refuse: sum overflow risk
    }
    if (lhs > rhs) {
      return 1;
    }
  }
  return 0;
}

// 3×3 Cramer determinant. Reads three plane rows (a, b, c, d) of scaled
// __int128 coefficients; returns det for the (a, b, c) 3×3 sub-block.
inline __int128 Det3(const __int128 p0[4], const __int128 p1[4], const __int128 p2[4]) {
  return p0[0] * (p1[1] * p2[2] - p1[2] * p2[1]) - p0[1] * (p1[0] * p2[2] - p1[2] * p2[0]) +
         p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
}

// Given three plane rows and a canonical det (must be non-zero, sign already
// canonicalized to positive), compute the (px, py, pz) numerators of the
// intersection point (before dividing by det). d values enter via -d
// (plane form a·x + b·y + c·z + d ≤ 0 → equality a·x + b·y + c·z = -d).
inline void SolveTriple(const __int128 p0[4], const __int128 p1[4], const __int128 p2[4], __int128* px, __int128* py,
                        __int128* pz) {
  const __int128 d0 = -p0[3];
  const __int128 d1 = -p1[3];
  const __int128 d2 = -p2[3];
  // Substitute the RHS d column into each of the three matrix columns and
  // take that sub-determinant.
  *px = d0 * (p1[1] * p2[2] - p1[2] * p2[1]) - p0[1] * (d1 * p2[2] - p1[2] * d2) + p0[2] * (d1 * p2[1] - p1[1] * d2);
  *py = p0[0] * (d1 * p2[2] - p1[2] * d2) - d0 * (p1[0] * p2[2] - p1[2] * p2[0]) + p0[2] * (p1[0] * d2 - d1 * p2[0]);
  *pz = p0[0] * (p1[1] * d2 - d1 * p2[1]) - p0[1] * (p1[0] * d2 - d1 * p2[0]) + d0 * (p1[0] * p2[1] - p1[1] * p2[0]);
}

// Exact incidence: is (px/det, py/det, pz/det) exactly on plane k? Same
// cross-multiplied form as feasibility but with equality. Called only on
// already-reduced (px, py, pz, det); overflow-safe under the same budget as
// IsFeasibleGuarded (asserted in caller via the pre-check on coef bits).
inline bool IsIncident(const __int128 plane[4], __int128 px, __int128 py, __int128 pz, __int128 det) {
  __int128 lhs = plane[0] * px + plane[1] * py + plane[2] * pz;
  __int128 rhs = -plane[3] * det;
  return lhs == rhs;
}

// __int128 GCD (Euclidean, non-negative inputs).
inline __int128 Gcd128(__int128 a, __int128 b) {
  if (a < 0)
    a = -a;
  if (b < 0)
    b = -b;
  while (b != 0) {
    __int128 t = a % b;
    a = b;
    b = t;
  }
  return a;
}

// Reduce (px, py, pz, det) to lowest terms in-place (det > 0 preserved).
// After reduction, two rational points represent the same value iff their
// four integers are pairwise equal. Bringing values down here (before the
// SamePoint / IsIncident checks) prevents the SamePoint cross-multiply from
// overflowing __int128 — raw values from SolveTriple can be up to S^3 ≈ 2^72
// each, and (px1 · det2) would blow past 2^127.
inline void Reduce(__int128* px, __int128* py, __int128* pz, __int128* det) {
  __int128 g = Gcd128(Gcd128(Gcd128(*px, *py), *pz), *det);
  if (g > 1) {
    *px /= g;
    *py /= g;
    *pz /= g;
    *det /= g;
  }
}

// Exact rational equality on ALREADY-REDUCED points (see Reduce above).
// Since both operands are in lowest terms with det > 0, equality is bit-for-bit.
inline bool SamePointExact(const ExactPyramidVertex& a, __int128 bx, __int128 by, __int128 bz, __int128 bd) {
  return a.px == bx && a.py == by && a.pz == bz && a.det == bd;
}

// Convert an __int128 to double for the fuzzy distance comparison in
// SamePointFuzzy. Loses precision beyond the 53-bit double mantissa but
// preserves magnitude — the SamePointFuzzy tolerance (5e-6 real units) is well
// above double ULP at typical vertex magnitudes (~1), so this conversion is
// safe. Not using `long double` because it's just an alias for double on Apple
// ARM64 (would silently break the intended precision on that platform).
inline double Int128ToD(__int128 x) {
  if (x == 0) {
    return 0.0;
  }
  double sign = 1.0;
  if (x < 0) {
    sign = -1.0;
    x = -x;
  }
  double lo = static_cast<double>(static_cast<uint64_t>(x));
  double hi = static_cast<double>(static_cast<uint64_t>(x >> 64));
  return sign * (hi * 18446744073709551616.0 + lo);
}

// Input-noise-tolerant SamePoint. Motivation: bit-exact rational equality on
// float-stored planes over-counts multi-plane concurrences (4+ planes meeting
// at a shoulder / apex corner). The plane coefs are stored as 24-bit floats,
// and algebraic identities like `2·(a1·√3/8) = a1·√3/4` that hold in exact
// math do NOT survive the float rounding — two triples targeting the "same"
// geometric corner give rationals that differ by O(1 ULP · shift) in the
// shifted-integer space, which SamePointExact reads as distinct.
//
// The tolerance here is TIED TO INPUT PRECISION, not an ad-hoc threshold:
// float ULP is 2^-24; the shifted-integer noise floor is O(2^-24 · shift_scale)
// ≈ a handful of shifted units. Well-conditioned geometry has vertex gaps of
// O(10^5+) shifted units, so tolerance = 2^10 = 1024 shifted units is safely
// inside the input-noise band and far below any legitimate vertex separation.
// This is NOT a feasibility/incidence epsilon (which a01/a02 forbid); it is
// consolidation of points that were the SAME real-number vertex under exact
// math but became split by the input's own float rounding.
//
// Distance comparison in the CROSS-MULTIPLIED (x*d, y*d, z*d) space (no true
// division) using double for magnitude — long-double loses precision
// beyond its mantissa but preserves the ULP-scale ordering used here.
inline bool SamePointFuzzy(const ExactPyramidVertex& a, __int128 bx, __int128 by, __int128 bz, __int128 bd,
                           double tol) {
  // Compare a.p / a.det against b.p / bd. Rewrite: a.p·bd vs b.p·a.det, delta
  // = |a.p·bd - b.p·a.det|. If |delta / (a.det · bd)| < tol → same point.
  // Compute in double to avoid __int128 overflow in the cross-multiply.
  double ax = Int128ToD(a.px);
  double ay = Int128ToD(a.py);
  double az = Int128ToD(a.pz);
  double ad = Int128ToD(a.det);
  double bxl = Int128ToD(bx);
  double byl = Int128ToD(by);
  double bzl = Int128ToD(bz);
  double bdl = Int128ToD(bd);
  double denom = ad * bdl;
  if (denom == 0.0L) {
    return false;
  }
  double dx = (ax * bdl - bxl * ad) / denom;
  double dy = (ay * bdl - byl * ad) / denom;
  double dz = (az * bdl - bzl * ad) / denom;
  return dx * dx + dy * dy + dz * dz < tol * tol;
}

}  // namespace exact_pyramid_detail

// Evaluate the exact oracle on a float plane_coef array of length plane_cnt.
// plane_coef layout matches FillHexCrystalCoef and ComputeClosedFormPyramid:
// (a, b, c, d) with the bounded half-space a·x + b·y + c·z + d ≤ 0.
inline ExactPyramidVerdict ExactPyramid(int plane_cnt, const float* plane_coef) {
  namespace d = exact_pyramid_detail;
  ExactPyramidVerdict out;
  if (plane_cnt <= 0 || plane_cnt > kExactPyramidMaxPlanes) {
    return out;
  }
  // Compute the per-input dynamic shift + noise floor, then check up front
  // that the scaled-coef magnitude will fit within the intermediate budget.
  // If not, refuse — a truncating fall-back is exactly what produced the
  // "regular pyramid → vtx=36" bug on the SHIFT=24 hardcode.
  const d::CoefAnalysis analysis = d::AnalyzeCoefs(plane_cnt, plane_coef);
  const int shift = analysis.shift;
  out.shift = shift;
  __int128 planes[kExactPyramidMaxPlanes][4];
  int max_coef_bits = 0;
  for (int i = 0; i < plane_cnt; i++) {
    for (int k = 0; k < 4; k++) {
      float f = plane_coef[i * 4 + k];
      // Snap sub-noise coefs to exact zero (see AnalyzeCoefs rationale).
      if (std::fabs(f) <= analysis.noise_floor) {
        f = 0.0f;
      }
      planes[i][k] = d::FloatToScaled(f, shift);
      int w = d::BitWidth128(planes[i][k]);
      if (w > max_coef_bits) {
        max_coef_bits = w;
      }
    }
  }
  out.max_intermediate_bits = max_coef_bits;
  if (max_coef_bits > d::kMaxCoefBits) {
    // Budget exhausted before we start. Refuse — silent downgrade would let
    // an already-observed failure mode recur (see file header).
    out.refused = true;
    out.vertex_count = 0;
    return out;
  }
  for (int i = 0; i < plane_cnt; i++) {
    for (int j = i + 1; j < plane_cnt; j++) {
      for (int k = j + 1; k < plane_cnt; k++) {
        __int128 det = d::Det3(planes[i], planes[j], planes[k]);
        int w_det = d::BitWidth128(det);
        if (w_det > out.max_intermediate_bits) {
          out.max_intermediate_bits = w_det;
        }
        if (det == 0) {
          continue;
        }
        __int128 px = 0;
        __int128 py = 0;
        __int128 pz = 0;
        d::SolveTriple(planes[i], planes[j], planes[k], &px, &py, &pz);
        if (det < 0) {
          det = -det;
          px = -px;
          py = -py;
          pz = -pz;
        }
        // Reduce first so IsFeasibleGuarded operates on bounded operands
        // (SolveTriple output can be up to ~3·coef_bits + 3 bits — the raw
        // form times another coef would blow past 127 bits at the top of
        // the coef budget).
        d::Reduce(&px, &py, &pz, &det);
        int feas = d::IsFeasibleGuarded(plane_cnt, planes, px, py, pz, det, &out.max_intermediate_bits);
        if (feas == -1) {
          // Overflow risk on some plane's cross-multiply. Refuse the whole
          // batch — a partial answer is worse than none.
          out.refused = true;
          out.vertex_count = 0;
          return out;
        }
        if (feas == 1) {
          continue;
        }
        // Fuzzy dedup at ~5·float-ULP: see SamePointFuzzy header for why exact
        // rational equality on float-stored planes over-counts multi-plane
        // concurrences (measured 0/175 agreement with closed-form under the
        // exact variant across a random asymmetric sweep). Tolerance is tied
        // to input precision, not an ad-hoc knob.
        constexpr double kFuzzyTol = 5.0e-6;  // ~80× float ULP; genuine vertex gaps ≥ 1e-3
        int existing = -1;
        for (int v = 0; v < out.vertex_count; v++) {
          if (d::SamePointFuzzy(out.vertices[v], px, py, pz, det, kFuzzyTol)) {
            existing = v;
            break;
          }
        }
        if (existing >= 0) {
          // Multi-plane concurrence: extend incidence mask on the existing
          // vertex with any planes that are exactly incident but weren't
          // among the 3 defining planes of the FIRST discovery of this
          // vertex. Do NOT recompute per-plane counts here; those are
          // rebuilt below from the final mask.
          out.vertices[existing].plane_incidence_mask |= (1ULL << i) | (1ULL << j) | (1ULL << k);
        } else if (out.vertex_count < kExactPyramidMaxVtx) {
          ExactPyramidVertex& v = out.vertices[out.vertex_count++];
          v.px = px;
          v.py = py;
          v.pz = pz;
          v.det = det;
          v.plane_incidence_mask = (1ULL << i) | (1ULL << j) | (1ULL << k);
        }
      }
    }
  }
  // Extra pass: for each discovered vertex, mark every OTHER plane it also
  // lies on (multi-plane concurrence beyond the discovering triple). This
  // catches all concurrences even the ones not discovered via ANOTHER triple.
  for (int v = 0; v < out.vertex_count; v++) {
    ExactPyramidVertex& vx = out.vertices[v];
    for (int k = 0; k < plane_cnt; k++) {
      if (d::IsIncident(planes[k], vx.px, vx.py, vx.pz, vx.det)) {
        vx.plane_incidence_mask |= (1ULL << k);
      }
    }
  }
  // Face vertex counts + face_present.
  for (int k = 0; k < plane_cnt; k++) {
    int cnt = 0;
    for (int v = 0; v < out.vertex_count; v++) {
      if ((out.vertices[v].plane_incidence_mask & (1ULL << k)) != 0) {
        cnt++;
      }
    }
    out.face_vertex_count[k] = cnt;
    out.face_present[k] = (cnt >= 3);
  }
  return out;
}

}  // namespace test_support
}  // namespace lumice

#endif  // LUMICE_TEST_SUPPORT_EXACT_PYRAMID_ORACLE_HPP_
