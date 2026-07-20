// Exact-integer prism corner oracle — header-only test support.
//
// Ground truth for the closed-form-prism refactor (scrum
// geometry-closed-form-representation, subtask closed-form-prism). Substituting
// x = √3·u leaves every prism half-plane constraint rational in dist[]; each
// float is exactly a dyadic rational, so scaling by a common power of two makes
// the whole feasibility predicate exact in __int128 with ZERO tolerance.
//
// After the substitution the six constraints reduce to
//   i=0: ( 4, 0, D0)   i=1: ( 2, 2, D1)   i=2: (-2, 2, D2)
//   i=3: (-4, 0, D3)   i=4: (-2,-2, D4)   i=5: ( 2,-2, D5)
// with Dᵢ = round(distᵢ · 2^SHIFT) (SHIFT = 30, exact for the float mantissa).
//
// This is a legacy-of-explore promotion: the C++ oracle used to live in
// bench/bench_geom_closedform.cpp as ExactPrismCornerCount. It was moved here
// so there is exactly ONE __int128 implementation. bench_geom_closedform.cpp
// now includes this header and delegates its ExactPrismCornerCount wrapper
// (doc/numerical-robustness.md rule 4: one predicate, one owner).
//
// A second, INDEPENDENT oracle (Python `Fraction`, general three-plane
// enumeration that does NOT bake the 6-direction structure into its logic) is
// used out-of-band for cross-verification. Two independent oracles are the
// point: a shared misconception cannot fool both.
//
// Where this header lives — and where it should NOT stay:
// It sits under test/support/ because the primary consumer is the golden-analytic
// test (test/golden-analytic/core/test_closed_form_prism.cpp) and bench is only a
// secondary consumer (a one-line wrapper). This creates a bench→test/support
// dependency, which is inverted vs. the usual "test depends on production". If a
// THIRD non-test consumer ever appears (a real tool, a production check), the
// right move is to reassess whether the oracle belongs in a neutral shared
// location (src/support/, or a fresh shared/) — not to keep piling reverse
// includes onto test/.

#ifndef LUMICE_TEST_SUPPORT_EXACT_PRISM_ORACLE_HPP_
#define LUMICE_TEST_SUPPORT_EXACT_PRISM_ORACLE_HPP_

#include <cmath>
#include <cstdint>

namespace lumice {
namespace test_support {

constexpr int kExactPrismSideCnt = 6;

// The oracle's per-sample verdict.
struct ExactPrismVerdict {
  // Total number of distinct feasible corners in 2D.
  int corner_count = 0;
  // Per-side-face bitmask of the corners lying on that face. Bit k of
  // face_corner_mask[i] is set iff corner index k (in the order the oracle
  // discovered it, 0..corner_count-1) lies on side face i by construction —
  // i.e. face i is one of the two planes whose intersection defined that
  // corner. The mask is written at solve time (not reverse-derived from the
  // final corner set), so it's an independent witness of the corner's
  // provenance, not a downstream inference.
  uint16_t face_corner_mask[kExactPrismSideCnt] = {};
};

// Evaluate the exact oracle from six side-face distances.
inline ExactPrismVerdict ExactPrism(const float dist[kExactPrismSideCnt]) {
  ExactPrismVerdict out;

  // SHIFT = 30 keeps |dist| < 2^33 exactly representable and leaves ample
  // headroom in __int128 for the 2×2 solves and the six substitutions below.
  constexpr int kShift = 30;
  __int128 s_pow2 = static_cast<__int128>(1) << kShift;
  __int128 dist_int[kExactPrismSideCnt];
  for (int i = 0; i < kExactPrismSideCnt; i++) {
    dist_int[i] = static_cast<__int128>(std::llround(static_cast<double>(dist[i]) * static_cast<double>(s_pow2)));
  }

  // (a, b, c) rows after scaling by 4·2^SHIFT — faces 0/3 carry the 4 because
  // their constraint is u ≤ d/4, the four oblique faces reduce to u±v ≤ d/2.
  // Getting this scaling wrong makes 0/3 redundant and silently turns every
  // hexagon into a quadrilateral — do not touch without re-deriving.
  struct ExactLine {
    __int128 a, b, c;
  };
  ExactLine lines[kExactPrismSideCnt] = {
    { 4, 0, dist_int[0] },  { 2, 2, dist_int[1] },   { -2, 2, dist_int[2] },
    { -4, 0, dist_int[3] }, { -2, -2, dist_int[4] }, { 2, -2, dist_int[5] },
  };

  __int128 vx[16];
  __int128 vy[16];
  __int128 vd[16];  // corner = (vx/vd, vy/vd), vd > 0 after canonicalization
  int cnt = 0;

  for (int i = 0; i < kExactPrismSideCnt; i++) {
    for (int j = i + 1; j < kExactPrismSideCnt; j++) {
      __int128 det = lines[i].a * lines[j].b - lines[j].a * lines[i].b;
      if (det == 0) {
        continue;  // parallel (opposite face pair) — exactly zero, not "small".
      }
      __int128 px = lines[i].c * lines[j].b - lines[j].c * lines[i].b;
      __int128 py = lines[i].a * lines[j].c - lines[j].a * lines[i].c;
      if (det < 0) {
        det = -det;
        px = -px;
        py = -py;
      }
      bool feasible = true;
      for (int m = 0; m < kExactPrismSideCnt; m++) {
        // (a·px + b·py)/det ≤ c   ⇔   a·px + b·py ≤ c·det   (det > 0)
        if (lines[m].a * px + lines[m].b * py > lines[m].c * det) {
          feasible = false;
          break;
        }
      }
      if (!feasible) {
        continue;
      }
      bool dup = false;
      int dup_idx = -1;
      for (int v = 0; v < cnt; v++) {
        // Exact cross-multiplied equality — no distance, no tolerance.
        if (vx[v] * det == px * vd[v] && vy[v] * det == py * vd[v]) {
          dup = true;
          dup_idx = v;
          break;
        }
      }
      if (dup) {
        // Multi-way concurrency: mark the extra pair on the existing corner.
        out.face_corner_mask[i] |= static_cast<uint16_t>(1u << dup_idx);
        out.face_corner_mask[j] |= static_cast<uint16_t>(1u << dup_idx);
      } else if (cnt < 16) {
        vx[cnt] = px;
        vy[cnt] = py;
        vd[cnt] = det;
        out.face_corner_mask[i] |= static_cast<uint16_t>(1u << cnt);
        out.face_corner_mask[j] |= static_cast<uint16_t>(1u << cnt);
        cnt++;
      }
    }
  }
  out.corner_count = cnt;
  return out;
}

}  // namespace test_support
}  // namespace lumice

#endif  // LUMICE_TEST_SUPPORT_EXACT_PRISM_ORACLE_HPP_
