// Exact-integer prism corner oracle — header-only test support.
//
// Ground truth for the closed-form-prism refactor. Substituting
// x = √3·u leaves every prism half-plane constraint rational in dist[]; each
// float is exactly a dyadic rational, so scaling by a common power of two makes
// the whole feasibility predicate exact in int64_t with ZERO tolerance.
//
// After the substitution the six constraints reduce to
//   i=0: ( 4, 0, D0)   i=1: ( 2, 2, D1)   i=2: (-2, 2, D2)
//   i=3: (-4, 0, D3)   i=4: (-2,-2, D4)   i=5: ( 2,-2, D5)
// with Dᵢ = round(distᵢ · 2^SHIFT) (SHIFT = 24, exact for the float mantissa).
//
// Bit-width budget (int64 replacement path — this file used to require
// __int128 to keep MSVC out of the header):
//   Inputs after SHIFT=24: |Dᵢ| < 2^25 (dist scaled by 2^24, |dist| ≤ ~1),
//   line coefficients |a|,|b| ≤ 4. Pairwise 2×2 solves:
//     det   = a_i·b_j - a_j·b_i,  |det|   ≤ 32 = 2^5
//     px/py = c_i·b_j - c_j·b_i,  |px/py| ≤ 2^25·8 = 2^28
//   Feasibility check computes a_m·px + b_m·py against c_m·det:
//     |a·px + b·py| ≤ 4·2^28 + 4·2^28 = 2^31; |c·det| ≤ 2^25·2^5 = 2^30.
//   Dedup cross-multiply vx·det vs px·vd: both operands ≤ 2^28, product ≤
//     2^56 — fits int64 with a 7-bit safety margin. All arithmetic is guarded
//     with `MulGuarded`/`AddGuarded` to refuse rather than wrap on any
//     unexpected input distribution.
//
// This header used to be an empty shell under MSVC because the previous
// implementation required __int128. The int64 replacement removes that gate;
// the oracle now compiles on every platform, so the primary consumer
// (test/golden-analytic/core/test_closed_form_prism.cpp) can run on Windows
// too.
//
// This is a legacy-of-explore promotion: the C++ oracle used to live in
// bench/bench_geom_closedform.cpp as ExactPrismCornerCount. It was moved here
// so there is exactly ONE implementation. bench_geom_closedform.cpp
// includes this header and delegates its ExactPrismCornerCount wrapper
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

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
  // Refused: an arithmetic step exceeded the int64 guard budget (should not
  // happen on the fixed pools per the bit-width analysis in the file header,
  // but the guard is enforced so unexpected inputs refuse rather than wrap).
  bool refused = false;
};

namespace exact_prism_detail {

// Bit-width budget: cap operand magnitudes at 56 bits so a subsequent
// multiply (up to 56 + 5 = 61 bits, worst case per file-header analysis) or
// sum stays within signed int64. Exceed → refuse.
constexpr int kMaxOperandBits = 56;

inline int BitWidth64(int64_t x) {
  if (x == 0)
    return 0;
  uint64_t u = (x < 0) ? static_cast<uint64_t>(-(x + 1)) + 1u : static_cast<uint64_t>(x);
  int w = 0;
  while (u != 0) {
    u >>= 1;
    w++;
  }
  return w;
}

inline int64_t AddGuarded(int64_t x, int64_t y, bool* overflow) {
  const int wx = BitWidth64(x), wy = BitWidth64(y);
  if (std::max(wx, wy) + 1 > 62) {
    *overflow = true;
    return 0;
  }
  const int64_t s = x + y;
  if (BitWidth64(s) > kMaxOperandBits)
    *overflow = true;
  return s;
}

inline int64_t MulGuarded(int64_t x, int64_t y, bool* overflow) {
  if (x == 0 || y == 0)
    return 0;
  const int wx = BitWidth64(x), wy = BitWidth64(y);
  if (wx + wy > kMaxOperandBits) {
    *overflow = true;
    return 0;
  }
  return x * y;
}

}  // namespace exact_prism_detail

// Evaluate the exact oracle from six side-face distances.
inline ExactPrismVerdict ExactPrism(const float dist[kExactPrismSideCnt]) {
  namespace d = exact_prism_detail;
  ExactPrismVerdict out;

  // SHIFT = 24 matches float mantissa exactly (llround into int64 is safe
  // because |dist·2^24| stays well below 2^62 for any realistic |dist|).
  // Bit-width analysis in the file header treats SHIFT = 24 as the working
  // number; keep them in sync.
  constexpr int kShift = 24;
  const int64_t s_pow2 = static_cast<int64_t>(1) << kShift;
  int64_t dist_int[kExactPrismSideCnt];
  for (int i = 0; i < kExactPrismSideCnt; i++) {
    dist_int[i] = static_cast<int64_t>(std::llround(static_cast<double>(dist[i]) * static_cast<double>(s_pow2)));
  }

  // (a, b, c) rows after scaling by 4·2^SHIFT — faces 0/3 carry the 4 because
  // their constraint is u ≤ d/4, the four oblique faces reduce to u±v ≤ d/2.
  // Getting this scaling wrong makes 0/3 redundant and silently turns every
  // hexagon into a quadrilateral — do not touch without re-deriving.
  struct ExactLine {
    int64_t a, b, c;
  };
  ExactLine lines[kExactPrismSideCnt] = {
    { 4, 0, dist_int[0] },  { 2, 2, dist_int[1] },   { -2, 2, dist_int[2] },
    { -4, 0, dist_int[3] }, { -2, -2, dist_int[4] }, { 2, -2, dist_int[5] },
  };

  int64_t vx[16];
  int64_t vy[16];
  int64_t vd[16];  // corner = (vx/vd, vy/vd), vd > 0 after canonicalization
  int cnt = 0;

  bool overflow = false;
  auto refuse = [&]() {
    out.refused = true;
    out.corner_count = 0;
    for (int i = 0; i < kExactPrismSideCnt; i++)
      out.face_corner_mask[i] = 0;
  };

  for (int i = 0; i < kExactPrismSideCnt; i++) {
    for (int j = i + 1; j < kExactPrismSideCnt; j++) {
      int64_t aibj = d::MulGuarded(lines[i].a, lines[j].b, &overflow);
      int64_t ajbi = d::MulGuarded(lines[j].a, lines[i].b, &overflow);
      int64_t det = d::AddGuarded(aibj, -ajbi, &overflow);
      if (overflow) {
        refuse();
        return out;
      }
      if (det == 0) {
        continue;  // parallel (opposite face pair) — exactly zero, not "small".
      }
      int64_t cibj = d::MulGuarded(lines[i].c, lines[j].b, &overflow);
      int64_t cjbi = d::MulGuarded(lines[j].c, lines[i].b, &overflow);
      int64_t px = d::AddGuarded(cibj, -cjbi, &overflow);
      int64_t aicj = d::MulGuarded(lines[i].a, lines[j].c, &overflow);
      int64_t ajci = d::MulGuarded(lines[j].a, lines[i].c, &overflow);
      int64_t py = d::AddGuarded(aicj, -ajci, &overflow);
      if (overflow) {
        refuse();
        return out;
      }
      if (det < 0) {
        det = -det;
        px = -px;
        py = -py;
      }
      bool feasible = true;
      for (int m = 0; m < kExactPrismSideCnt; m++) {
        // (a·px + b·py)/det ≤ c   ⇔   a·px + b·py ≤ c·det   (det > 0)
        int64_t apx = d::MulGuarded(lines[m].a, px, &overflow);
        int64_t bpy = d::MulGuarded(lines[m].b, py, &overflow);
        int64_t lhs = d::AddGuarded(apx, bpy, &overflow);
        int64_t rhs = d::MulGuarded(lines[m].c, det, &overflow);
        if (overflow) {
          refuse();
          return out;
        }
        if (lhs > rhs) {
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
        int64_t vxd = d::MulGuarded(vx[v], det, &overflow);
        int64_t pxvd = d::MulGuarded(px, vd[v], &overflow);
        int64_t vyd = d::MulGuarded(vy[v], det, &overflow);
        int64_t pyvd = d::MulGuarded(py, vd[v], &overflow);
        if (overflow) {
          refuse();
          return out;
        }
        if (vxd == pxvd && vyd == pyvd) {
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
