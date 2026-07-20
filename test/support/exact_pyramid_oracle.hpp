// Exact-integer pyramid vertex oracle — header-only test support.
//
// Ground truth for the closed-form-pyramid work. Unlike the prism oracle, the
// pyramid coefficients are not scalable to a single integer basis by a fixed
// substitution — a1 spans [7.56e-4, 248] over the legal wedge range, so the
// substitution trick (x = √3·u → tiny integer rows) doesn't work uniformly.
//
// Approach here: pressure-tested design (see bench_geom_closedform.cpp's
// BM_PyramidOracleBitWidthPressure). Uniform SHIFT=24 across all coefficients
// (matching float mantissa exactly) keeps every float coef captured with zero
// truncation and bounds the worst-case triple-product intermediate at ~2^109
// bits — well inside __int128 with 19-bit headroom. This is the "float-tier"
// oracle: it operates on the float coefficients the closed-form actually
// stores, so it tests exactly the level of precision the closed-form
// commits to. Any double-precision internals of the closed-form that don't
// affect its float output are outside this oracle's scope by design.
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
//   - Two vertices are "the same" iff their exact rational (px/det, py/det,
//     pz/det) representations satisfy cross-multiplied equality. No distance,
//     no tolerance.
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
};

namespace exact_pyramid_detail {

constexpr int kShift = 24;
constexpr __int128 kShiftScale = static_cast<__int128>(1) << kShift;

// Scale a float to an exact __int128 by SHIFT=24. Since float has 24 bit
// mantissa, this is lossless for values in [-2^40, 2^40] (mantissa · 2^SHIFT
// still fits in ~64 bits, with the __int128 giving plenty of headroom).
inline __int128 FloatToScaled(float x) {
  return static_cast<__int128>(std::llround(static_cast<double>(x) * static_cast<double>(kShiftScale)));
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

// Exact feasibility: is (px/det, py/det, pz/det) inside every plane's
// half-space a·x + b·y + c·z + d ≤ 0? Cross-multiply by det > 0 to keep
// integer.
//   a·(px/det) + b·(py/det) + c·(pz/det) ≤ -d
//   a·px + b·py + c·pz ≤ -d · det
inline bool IsFeasible(int n, const __int128 planes[][4], __int128 px, __int128 py, __int128 pz, __int128 det) {
  for (int k = 0; k < n; k++) {
    __int128 lhs = planes[k][0] * px + planes[k][1] * py + planes[k][2] * pz;
    __int128 rhs = -planes[k][3] * det;
    if (lhs > rhs) {
      return false;
    }
  }
  return true;
}

// Exact incidence: is (px/det, py/det, pz/det) exactly on plane k? Same
// cross-multiplied form as IsFeasible but with equality.
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
inline bool SamePoint(const ExactPyramidVertex& a, __int128 bx, __int128 by, __int128 bz, __int128 bd) {
  return a.px == bx && a.py == by && a.pz == bz && a.det == bd;
}

}  // namespace exact_pyramid_detail

// Evaluate the exact oracle on a float plane_coef array of length plane_cnt.
// plane_coef layout matches FillHexCrystalCoef and ComputeClosedFormPyramid:
// (a, b, c, d) with the bounded half-space a·x + b·y + c·z + d ≤ 0.
inline ExactPyramidVerdict ExactPyramid(int plane_cnt, const float* plane_coef) {
  using namespace exact_pyramid_detail;
  ExactPyramidVerdict out;
  if (plane_cnt <= 0 || plane_cnt > kExactPyramidMaxPlanes) {
    return out;
  }
  __int128 planes[kExactPyramidMaxPlanes][4];
  for (int i = 0; i < plane_cnt; i++) {
    for (int k = 0; k < 4; k++) {
      planes[i][k] = FloatToScaled(plane_coef[i * 4 + k]);
    }
  }
  for (int i = 0; i < plane_cnt; i++) {
    for (int j = i + 1; j < plane_cnt; j++) {
      for (int k = j + 1; k < plane_cnt; k++) {
        __int128 det = Det3(planes[i], planes[j], planes[k]);
        if (det == 0) {
          continue;
        }
        __int128 px, py, pz;
        SolveTriple(planes[i], planes[j], planes[k], &px, &py, &pz);
        if (det < 0) {
          det = -det;
          px = -px;
          py = -py;
          pz = -pz;
        }
        if (!IsFeasible(plane_cnt, planes, px, py, pz, det)) {
          continue;
        }
        // Reduce to lowest terms BEFORE any SamePoint / IsIncident check.
        // Prevents downstream cross-multiply from overflowing __int128.
        Reduce(&px, &py, &pz, &det);
        int existing = -1;
        for (int v = 0; v < out.vertex_count; v++) {
          if (SamePoint(out.vertices[v], px, py, pz, det)) {
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
          out.vertices[existing].plane_incidence_mask |= (1ull << i) | (1ull << j) | (1ull << k);
        } else if (out.vertex_count < kExactPyramidMaxVtx) {
          ExactPyramidVertex& v = out.vertices[out.vertex_count++];
          v.px = px;
          v.py = py;
          v.pz = pz;
          v.det = det;
          v.plane_incidence_mask = (1ull << i) | (1ull << j) | (1ull << k);
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
      if (IsIncident(planes[k], vx.px, vx.py, vx.pz, vx.det)) {
        vx.plane_incidence_mask |= (1ull << k);
      }
    }
  }
  // Face vertex counts + face_present.
  for (int k = 0; k < plane_cnt; k++) {
    int cnt = 0;
    for (int v = 0; v < out.vertex_count; v++) {
      if (out.vertices[v].plane_incidence_mask & (1ull << k)) {
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
