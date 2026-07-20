#include "core/geo3d_closedform.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>

#include "core/math.hpp"

namespace lumice {

namespace {

// ============================================================================
// Shared 2D helpers — used by both prism and pyramid closed-form paths.
// ============================================================================

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

// Bit flags recorded by SolveHexCrossSection into an optional out-parameter.
// Purpose: give the golden-analytic tests a way to assert that shoulder-point /
// apex / face-drop samples exercise the SAME structural branches as the
// well-conditioned "regular" samples, elevating the "no special case for these
// three phenomena" property from a code-review claim to a CI-enforced check.
//
// Naming discipline (do not conflate with face_corner_mask): this is a
// PATH TAG, not a witness. face_corner_mask (used elsewhere in this module and
// in exact_prism_oracle.hpp) records WHICH FACES define each corner; a path tag
// records WHICH BRANCHES the solver walked. Different objects, different
// semantics.
//
// Anchor granularity: coarse structural branches of SolveHexCrossSection only,
// never per-comparison floating-point outcomes (which would drift on legitimate
// numerical differences and turn any set-equality assertion into a false
// alarm).
enum SolveHexCrossSectionPathTag : uint16_t {
  kPathTagAnyDirDegenerate = 1u << 0,  // any direction i had dist[i] ≤ 0 (fully eroded)
  kPathTagDedupHit = 1u << 1,          // at least one candidate corner was deduped
  kPathTagBounded = 1u << 2,           // at least 3 present directions bounded a polygon
  kPathTagAllDirsPresent = 1u << 3,    // all 6 directions bound a face (full hexagon)
  kPathTagEmpty = 1u << 4,             // feasible region is empty (no corners survived)
};

// SolveHexCrossSection — the 2D half-plane intersection over the six fixed
// prism directions θᵢ = i·60° with per-direction offset `r_side_dist[i]` (in
// the SAME units the caller uses: for prism, r_side_dist[i] = (√3/4)·dist[i];
// for pyramid at height z the caller passes (√3/4)·(dist[i] − m(z))).
//
// Signature deliberately mirrors what BOTH the prism entry (dist[6]) and the
// pyramid entry (eroded dist at some height z) need. Optional out_path_tag is
// nullptr in production callers → zero write-path overhead; test callers pass
// non-null to observe the branch bitmask.
struct HexCrossSection {
  // CCW-ordered feasible corners (kClosedFormPrismMaxCorners is the loose
  // upper bound; corner_cnt is the exact count).
  int corner_cnt = 0;
  double corner_x[kClosedFormPrismMaxCorners]{};
  double corner_y[kClosedFormPrismMaxCorners]{};
  // side_present[i] iff at least 2 distinct feasible corners lie on direction i.
  bool side_present[kClosedFormPrismSideCnt]{};
  bool any_side_present = false;
};

HexCrossSection SolveHexCrossSection(const double r_side_dist[kClosedFormPrismSideCnt],
                                     uint16_t* out_path_tag = nullptr) {
  HexCrossSection out;
  uint16_t tag = 0;

  double cs[kClosedFormPrismSideCnt];
  double sn[kClosedFormPrismSideCnt];
  double scale = 0.0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    double theta = static_cast<double>(i) * math::kPi_3;
    cs[i] = std::cos(theta);
    sn[i] = std::sin(theta);
    scale = std::max(scale, std::fabs(r_side_dist[i]));
    if (r_side_dist[i] <= 0.0) {
      tag |= kPathTagAnyDirDegenerate;
    }
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
      bool solved = Solve2x2(cs[i], sn[i], cs[j], sn[j], r_side_dist[i], r_side_dist[j], &px, &py);
      // For fixed 60°-spaced directions the determinant is ±sin 60° = ±√3/2 ≠ 0,
      // strictly, so Solve2x2 cannot fail here — this catches typos in the
      // solver itself.
      assert(solved);
      (void)solved;

      // Self-consistency assert: the solved point must satisfy its two defining
      // planes to within tolerance. This is the fastest tripwire for solver
      // typos — a mis-scaled Cramer expansion trips it on the first sample.
      assert(std::fabs(cs[i] * px + sn[i] * py - r_side_dist[i]) <= tol);
      assert(std::fabs(cs[j] * px + sn[j] * py - r_side_dist[j]) <= tol);

      bool feasible = true;
      for (int m = 0; m < kClosedFormPrismSideCnt; m++) {
        if (m == i || m == j) {
          continue;
        }
        if (cs[m] * px + sn[m] * py > r_side_dist[m] + tol) {
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
      if (dup) {
        tag |= kPathTagDedupHit;
        continue;
      }
      if (corner_cnt_local < kClosedFormPrismMaxCorners) {
        corners[corner_cnt_local].x = px;
        corners[corner_cnt_local].y = py;
        corner_cnt_local++;
      }
    }
  }

  // face_present[i] = at least 2 distinct feasible corners lie on side i.
  int present_n = 0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    int on = 0;
    for (int v = 0; v < corner_cnt_local; v++) {
      if (std::fabs(cs[i] * corners[v].x + sn[i] * corners[v].y - r_side_dist[i]) <= tol) {
        on++;
      }
    }
    out.side_present[i] = (on >= 2);
    if (out.side_present[i]) {
      present_n++;
    }
  }

  bool any_side_present = false;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    if (out.side_present[i]) {
      any_side_present = true;
      break;
    }
  }
  out.any_side_present = any_side_present;

  if (!any_side_present) {
    tag |= kPathTagEmpty;
    if (out_path_tag != nullptr) {
      *out_path_tag = tag;
    }
    return out;
  }
  tag |= kPathTagBounded;
  if (present_n == kClosedFormPrismSideCnt) {
    tag |= kPathTagAllDirsPresent;
  }

  // Emit CCW corner ring by walking present side faces in i-increasing order.
  // Adjacent-in-list pair (P[k], P[(k+1) % n]) defines corner k; opposite pairs
  // are impossible because a bounded polygon cannot be defined by two parallel
  // faces alone. For any adjacent-in-list (i, j) with j != i+3 the intersection
  // is well-defined and (by convexity + both faces being present) feasible.
  int present_idx[kClosedFormPrismSideCnt];
  int p_n = 0;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    if (out.side_present[i]) {
      present_idx[p_n++] = i;
    }
  }
  int emitted = 0;
  for (int k = 0; k < p_n; k++) {
    int i = present_idx[k];
    int j = present_idx[(k + 1) % p_n];
    assert(std::abs(i - j) != 3);
    double px = 0.0;
    double py = 0.0;
    bool solved = Solve2x2(cs[i], sn[i], cs[j], sn[j], r_side_dist[i], r_side_dist[j], &px, &py);
    assert(solved);
    (void)solved;
    out.corner_x[emitted] = px;
    out.corner_y[emitted] = py;
    emitted++;
  }
  out.corner_cnt = emitted;

  if (out_path_tag != nullptr) {
    *out_path_tag = tag;
  }
  return out;
}

// ============================================================================
// Prism-specific closed-form.
// ============================================================================

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

  // Build the 2D cross-section input: r_side[i] = (√3/4) · dist[i].
  double r_side[kClosedFormPrismSideCnt];
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    r_side[i] = k_r * static_cast<double>(dist[i]);
  }
  HexCrossSection xs = SolveHexCrossSection(r_side);

  // Basal presence: iff any side face is present (a non-empty 2D cross section
  // extrudes both basal faces). The zero-volume short-circuit above already
  // handled h ≈ 0.
  r.face_present[0] = xs.any_side_present;
  r.face_present[1] = xs.any_side_present;
  for (int i = 0; i < kClosedFormPrismSideCnt; i++) {
    r.face_present[2 + i] = xs.side_present[i];
  }
  for (int c = 0; c < xs.corner_cnt; c++) {
    r.corner_x[c] = static_cast<float>(xs.corner_x[c]);
    r.corner_y[c] = static_cast<float>(xs.corner_y[c]);
  }
  r.corner_cnt = xs.corner_cnt;
  return r;
}

}  // namespace lumice
