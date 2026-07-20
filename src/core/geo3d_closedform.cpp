#include "core/geo3d_closedform.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>

#include "core/math.hpp"

namespace lumice {

namespace {

// Generic 2×2 solver for a·x = b (with a in row-major layout). Do NOT
// hand-expand per case: hand-expanded Cramer here silently mis-scales or
// sign-flips one row (the failure mode is subtle because the two-plane residual
// is only checked at kFloatEps tolerance — a factor-of-2 error still passes at
// the outer feasibility check). Returns true iff the system is non-degenerate.
bool Solve2x2(double a00, double a01, double a10, double a11, double b0, double b1, double* out_x, double* out_y) {
  double det = a00 * a11 - a01 * a10;
  if (det == 0.0) {
    return false;
  }
  *out_x = (b0 * a11 - b1 * a01) / det;
  *out_y = (a00 * b1 - a10 * b0) / det;
  return true;
}

}  // namespace

ClosedFormPrismResult ComputeClosedFormPrism(float h, const float dist[6]) {
  ClosedFormPrismResult r{};

  // Face-number constants (kPrism: {1..8}).
  for (int i = 0; i < kClosedFormPrismFaceCnt; i++) {
    r.face_number[i] = i + 1;
  }

  // Face normals — parametric, no solve.
  //   slot 0: upper basal   (0, 0, +1)
  //   slot 1: lower basal   (0, 0, -1)
  //   slot 2+i: side face i (cos θᵢ, sin θᵢ, 0)
  r.face_normal[0] = 0.0f;
  r.face_normal[1] = 0.0f;
  r.face_normal[2] = 1.0f;
  r.face_normal[3] = 0.0f;
  r.face_normal[4] = 0.0f;
  r.face_normal[5] = -1.0f;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    r.face_normal[(2 + i) * 3 + 0] = static_cast<float>(std::cos(theta));
    r.face_normal[(2 + i) * 3 + 1] = static_cast<float>(std::sin(theta));
    r.face_normal[(2 + i) * 3 + 2] = 0.0f;
  }

  // Plane coefficients (a, b, c, d) with the half-space a·x + b·y + c·z + d ≤ 0.
  // Matches FillHexCrystalCoef's convention (see geo3d.cpp lines 346-370).
  //   Basal:   (0, 0, ±1, -h/2)
  //   Side i:  0.5·(cos θᵢ, sin θᵢ, 0), constant -distᵢ·√3/8
  float h_half = 0.5f * h;
  r.plane_coef[0] = 0.0f;
  r.plane_coef[1] = 0.0f;
  r.plane_coef[2] = 1.0f;
  r.plane_coef[3] = -h_half;
  r.plane_coef[4] = 0.0f;
  r.plane_coef[5] = 0.0f;
  r.plane_coef[6] = -1.0f;
  r.plane_coef[7] = -h_half;
  double k_r = math::kSqrt3 / 4.0;  // r_i = (√3/4)·dist[i]
  double k_d = math::kSqrt3 / 8.0;  // d_i = -dist[i]·√3/8
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    r.plane_coef[(2 + i) * 4 + 0] = 0.5f * static_cast<float>(std::cos(theta));
    r.plane_coef[(2 + i) * 4 + 1] = 0.5f * static_cast<float>(std::sin(theta));
    r.plane_coef[(2 + i) * 4 + 2] = 0.0f;
    r.plane_coef[(2 + i) * 4 + 3] = -static_cast<float>(k_d * static_cast<double>(dist[i]));
  }

  r.inset_at_top = 0.0f;
  r.inset_at_bottom = 0.0f;
  r.corner_cnt = 0;

  // Zero-volume short circuit — matches FillHexCrystalCoef's degenerate path
  // (geo3d.cpp:395-399 emits an empty plane set when h < kFloatEps).
  if (h <= math::kFloatEps) {
    for (int i = 0; i < kClosedFormPrismFaceCnt; i++) {
      r.face_present[i] = false;
    }
    return r;
  }

  // Precompute side-face 2D data in double.
  double cs[kClosedFormPrismSideCnt];
  double sn[kClosedFormPrismSideCnt];
  double r_side[kClosedFormPrismSideCnt];
  double scale = 0.0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    cs[i] = std::cos(theta);
    sn[i] = std::sin(theta);
    r_side[i] = k_r * static_cast<double>(dist[i]);
    scale = std::max(scale, std::fabs(r_side[i]));
  }
  // Relative feasibility / dedup tolerance. Empirically the 9-orders-of-magnitude
  // stable window ends around here — at 4-way concurrencies (a corner where
  // two solve-pairs collapse to the same geometric point), the double-precision
  // residuals between the two candidates are of order 10·kFloatEps, so a
  // pure 1·kFloatEps threshold occasionally fails to merge them. 5·kFloatEps
  // aligns with production's dedup scale (SolveConvexPolyhedronVtxD uses
  // 5e-5·char_len — src/core/math.cpp:807-811) and keeps distinct corners in
  // the well-conditioned regime safely apart (~0.01+ scale at σ = 0.2).
  double tol = 5.0 * static_cast<double>(math::kFloatEps) * std::max(scale, 1.0);

  // Step 1: enumerate C(6,2) − 3 = 12 candidate corners; test feasibility and
  // dedup by exact-equality on (x, y) then relative distance.
  struct Corner {
    double x, y;
  };
  std::array<Corner, kClosedFormPrismMaxCorners> corners{};
  int corner_cnt_local = 0;

  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    for (int j = i + 1; j < kClosedFormPrismSideCnt; j++) {
      if (j == i + 3) {
        continue;  // opposite faces: parallel by construction, no threshold needed.
      }
      double px = 0.0;
      double py = 0.0;
      bool solved = Solve2x2(cs[i], sn[i], cs[j], sn[j], r_side[i], r_side[j], &px, &py);
      // For fixed 60°-spaced directions the determinant is ±sin 60° = ±√3/2 ≠ 0,
      // strictly, so Solve2x2 cannot fail here — this catches typos in the
      // solver itself (plan §7 risk 1).
      assert(solved);
      (void)solved;

      // Self-consistency assert: the solved point must satisfy its two defining
      // planes to within tolerance. This is the fastest tripwire for solver
      // typos — a mis-scaled Cramer expansion trips it on the first sample.
      assert(std::fabs(cs[i] * px + sn[i] * py - r_side[i]) <= tol);
      assert(std::fabs(cs[j] * px + sn[j] * py - r_side[j]) <= tol);

      bool feasible = true;
      for (int m = 0; m < kClosedFormPrismSideCnt; m++) {
        if (m == i || m == j) {
          continue;
        }
        if (cs[m] * px + sn[m] * py > r_side[m] + tol) {
          feasible = false;
          break;
        }
      }
      if (!feasible) {
        continue;
      }
      bool dup = false;
      for (int v = 0; v < corner_cnt_local; v++) {
        double dx = corners[v].x - px;
        double dy = corners[v].y - py;
        if (std::sqrt(dx * dx + dy * dy) <= tol) {
          dup = true;
          break;
        }
      }
      if (!dup && corner_cnt_local < kClosedFormPrismMaxCorners) {
        corners[corner_cnt_local].x = px;
        corners[corner_cnt_local].y = py;
        corner_cnt_local++;
      }
    }
  }

  // Step 2: face_present[i] = at least 2 distinct feasible corners lie on side i.
  // Basal presence: iff any side face is present (a non-empty 2D cross section extrudes
  // both basal faces). The zero-volume short-circuit above already handled h ≈ 0.
  bool side_present[kClosedFormPrismSideCnt] = { false, false, false, false, false, false };
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    int on = 0;
    for (int v = 0; v < corner_cnt_local; v++) {
      if (std::fabs(cs[i] * corners[v].x + sn[i] * corners[v].y - r_side[i]) <= tol) {
        on++;
      }
    }
    side_present[i] = (on >= 2);
  }

  bool any_side_present = false;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    if (side_present[i]) {
      any_side_present = true;
      break;
    }
  }
  r.face_present[0] = any_side_present;  // upper basal
  r.face_present[1] = any_side_present;  // lower basal
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    r.face_present[2 + i] = side_present[i];
  }

  if (!any_side_present) {
    return r;
  }

  // Step 3: emit CCW corner ring by walking present side faces in i-increasing
  // order. Adjacent present faces (P[k], P[(k+1) % n]) define corner k; opposite
  // pairs are impossible here because a bounded polygon cannot be defined by
  // two parallel faces alone. For any adjacent-in-list pair (i, j) with j > i,
  // if j != i + 3 the intersection is well-defined and (by convexity + both
  // faces being present) already feasible — it's one of the corners collected above.
  int present_idx[kClosedFormPrismSideCnt];
  int present_n = 0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    if (side_present[i]) {
      present_idx[present_n++] = i;
    }
  }

  int emitted = 0;
  for (int k = 0; k < present_n; k++) {
    int i = present_idx[k];
    int j = present_idx[(k + 1) % present_n];
    // Opposite-pair guard: convex bounded polygon cannot survive as {i, i+3} alone;
    // if we ever hit this it's a bug (assert, don't silently emit garbage).
    assert(std::abs(i - j) != 3);
    double px = 0.0;
    double py = 0.0;
    bool solved = Solve2x2(cs[i], sn[i], cs[j], sn[j], r_side[i], r_side[j], &px, &py);
    assert(solved);
    (void)solved;
    r.corner_x[emitted] = static_cast<float>(px);
    r.corner_y[emitted] = static_cast<float>(py);
    emitted++;
  }
  r.corner_cnt = emitted;
  return r;
}

}  // namespace lumice
