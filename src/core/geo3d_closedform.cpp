#include "core/geo3d_closedform.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>

#include "core/geo3d.hpp"
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
// Pyramid-specific helpers — anonymous-namespace.
// ============================================================================
//
// Model (verified empirically, ~3000 sample sweep of the plane sets from
// FillHexCrystalCoef):
//   At every height z the horizontal cross section is the same hexagonal
//   half-plane intersection over the six fixed 60°-spaced directions with
//   per-direction offset (√3/4)·(dist[i] − m(z)), where m(z) is a scalar
//   piecewise-linear "inset" function:
//     m(z) = 0                       if |z| ≤ h2/2
//          = (z − h2/2) / a1         if z > h2/2   (upper cone)
//          = (−h2/2 − z) / a2        if z < −h2/2  (lower cone)
//   with a1, a2 = √3/4 / tan(wedge_alpha).
//
// The "natural" apex on each side is at m = m_apex_side; the actual basal cut
// on that side is at fraction h1 or h3 of the way from the shoulder to the
// natural apex:
//   z_top = h2/2 + a1 · (h1 · m_apex_upper)
//   z_bot = −h2/2 − a2 · (h3 · m_apex_lower)
//
// m_apex_side is the LP-max of m subject to (u, v, m) satisfying every
// direction's constraint  cos(t_i)·u + sin(t_i)·v + m ≤ (√3/4)·dist[i].
// That LP is a 3D linear program whose solution is at a 3-face concurrence —
// exactly one of the C(6,3)=20 triples of directions. Step 1(b) empirically
// established that the winning triple is NOT necessarily adjacent-only; ~49%
// of samples showed far-direction dependence. Hence enumerate all 20 triples.

constexpr double kInsetK = 0.25 * 1.7320508075688772935;  // √3/4 = k_r

struct ApexLPResult {
  double m = 0.0;
  double u = 0.0;
  double v = 0.0;
};

// LP solve for max m such that cos(t_i)·u + sin(t_i)·v + m ≤ dist_scaled[i]
// for all i = 0..5, where dist_scaled[i] = (√3/4)·dist[i]. Solution is at a
// 3-face concurrence: enumerate C(6,3) = 20 triples, solve each 3×3 system
// for (u, v, m), keep the LARGEST feasible m and record its (u, v). Returns
// {0, 0, 0} if the LP is infeasible.
ApexLPResult MaxFeasibleInsetLP(const double dist_scaled[kClosedFormPyramidSideCnt]) {
  double cs[kClosedFormPyramidSideCnt];
  double sn[kClosedFormPyramidSideCnt];
  double scale = 0.0;
  for (int i = 0; i < kClosedFormPyramidSideCnt; i++) {
    double t = static_cast<double>(i) * math::kPi_3;
    cs[i] = std::cos(t);
    sn[i] = std::sin(t);
    scale = std::max(scale, std::fabs(dist_scaled[i]));
  }
  double tol = 5.0 * static_cast<double>(math::kFloatEps) * std::max(scale, 1.0);
  ApexLPResult best;
  bool found = false;

  for (int i = 0; i < kClosedFormPyramidSideCnt; i++) {
    for (int j = i + 1; j < kClosedFormPyramidSideCnt; j++) {
      for (int k = j + 1; k < kClosedFormPyramidSideCnt; k++) {
        // 3×3 system:
        //   [cs_i sn_i 1] [u]   [d_i]
        //   [cs_j sn_j 1] [v] = [d_j]
        //   [cs_k sn_k 1] [m]   [d_k]
        double det = cs[i] * (sn[j] - sn[k]) - sn[i] * (cs[j] - cs[k]) + (cs[j] * sn[k] - cs[k] * sn[j]);
        if (std::fabs(det) < 1e-12) {
          continue;
        }
        double di = dist_scaled[i], dj = dist_scaled[j], dk = dist_scaled[k];
        double u = (di * (sn[j] - sn[k]) - sn[i] * (dj - dk) + (dj * sn[k] - dk * sn[j])) / det;
        double v = (cs[i] * (dj - dk) - di * (cs[j] - cs[k]) + (cs[j] * dk - cs[k] * dj)) / det;
        double m = (cs[i] * (sn[j] * dk - sn[k] * dj) - sn[i] * (cs[j] * dk - cs[k] * dj) +
                    di * (cs[j] * sn[k] - cs[k] * sn[j])) /
                   det;
        if (m < -tol) {
          continue;
        }
        bool feasible = true;
        for (int r = 0; r < kClosedFormPyramidSideCnt; r++) {
          if (r == i || r == j || r == k) {
            continue;
          }
          if (cs[r] * u + sn[r] * v + m > dist_scaled[r] + tol) {
            feasible = false;
            break;
          }
        }
        if (!feasible) {
          continue;
        }
        if (!found || m > best.m) {
          best.m = m;
          best.u = u;
          best.v = v;
          found = true;
        }
      }
    }
  }
  if (!found) {
    return {};
  }
  best.m = std::max(best.m, 0.0);
  return best;
}

// Enumerate corner-death z-events in the upper cone half-space (m ∈ (0, m_max]).
// Each triple gives one m*, translated to z* = h2/2 + a1·m*. Events with
// m ∉ (tol, m_max] are dropped. Returns the count written; out_z holds the
// (sorted, deduped) z values. out_z must have capacity for C(6,3)=20 events.
int EnumerateConeDeathZ(const double dist_scaled[kClosedFormPyramidSideCnt], double m_max, double h2_2, double a,
                        bool upper_side, double out_z[20]) {
  double cs[kClosedFormPyramidSideCnt];
  double sn[kClosedFormPyramidSideCnt];
  double scale = 0.0;
  for (int i = 0; i < kClosedFormPyramidSideCnt; i++) {
    double t = static_cast<double>(i) * math::kPi_3;
    cs[i] = std::cos(t);
    sn[i] = std::sin(t);
    scale = std::max(scale, std::fabs(dist_scaled[i]));
  }
  double tol = 5.0 * static_cast<double>(math::kFloatEps) * std::max(scale, 1.0);
  double zs[20];
  int cnt = 0;
  for (int i = 0; i < kClosedFormPyramidSideCnt; i++) {
    for (int j = i + 1; j < kClosedFormPyramidSideCnt; j++) {
      for (int k = j + 1; k < kClosedFormPyramidSideCnt; k++) {
        double det = cs[i] * (sn[j] - sn[k]) - sn[i] * (cs[j] - cs[k]) + (cs[j] * sn[k] - cs[k] * sn[j]);
        if (std::fabs(det) < 1e-12) {
          continue;
        }
        double di = dist_scaled[i], dj = dist_scaled[j], dk = dist_scaled[k];
        double m = (cs[i] * (sn[j] * dk - sn[k] * dj) - sn[i] * (cs[j] * dk - cs[k] * dj) +
                    di * (cs[j] * sn[k] - cs[k] * sn[j])) /
                   det;
        if (m < tol || m > m_max - tol) {
          continue;  // outside the eroded z-range of this cone
        }
        zs[cnt++] = upper_side ? (h2_2 + a * m) : (-h2_2 - a * m);
      }
    }
  }
  // Dedup + sort.
  std::sort(zs, zs + cnt);
  int out_n = 0;
  for (int i = 0; i < cnt; i++) {
    if (out_n > 0 && std::fabs(zs[i] - out_z[out_n - 1]) < 1e-9) {
      continue;
    }
    out_z[out_n++] = zs[i];
  }
  return out_n;
}

// Compute the scalar inset at height z under the piecewise-linear model.
// a_upper / a_lower use −1.0 sentinel for "cone absent"; when absent the
// corresponding half-space is unreachable so this function is called only
// for z inside the presented range.
double ComputeInset(double z, double h2_2, double a_upper, double a_lower) {
  if (z > h2_2 && a_upper > 0) {
    return (z - h2_2) / a_upper;
  }
  if (z < -h2_2 && a_lower > 0) {
    return (-h2_2 - z) / a_lower;
  }
  return 0.0;
}

// Simple 3D vertex dedup: append (x, y, z) to pool if not within tol of any
// existing vertex; return its index. Cap = kClosedFormPyramidMaxVtx.
int InsertOrFindVertex(double x, double y, double z, float* pool, int* cnt, double tol) {
  for (int i = 0; i < *cnt; i++) {
    double dx = pool[i * 3 + 0] - x;
    double dy = pool[i * 3 + 1] - y;
    double dz = pool[i * 3 + 2] - z;
    if (dx * dx + dy * dy + dz * dz <= tol * tol) {
      return i;
    }
  }
  assert(*cnt < kClosedFormPyramidMaxVtx && "pyramid vertex pool overflow — expand kClosedFormPyramidMaxVtx");
  int idx = *cnt;
  pool[idx * 3 + 0] = static_cast<float>(x);
  pool[idx * 3 + 1] = static_cast<float>(y);
  pool[idx * 3 + 2] = static_cast<float>(z);
  (*cnt)++;
  return idx;
}

// Append vtx index to face's polygon vertex list (if not already there).
void AppendFaceVtx(ClosedFormPyramidResult* r, int face_slot, int vtx_idx) {
  int* cnt = &r->face_vtx_cnt[face_slot];
  for (int i = 0; i < *cnt; i++) {
    if (r->face_vtx[face_slot][i] == vtx_idx) {
      return;
    }
  }
  assert(*cnt < kClosedFormPyramidMaxFaceVtx &&
         "pyramid face polygon size overflow — expand kClosedFormPyramidMaxFaceVtx");
  r->face_vtx[face_slot][(*cnt)++] = vtx_idx;
}

// Internal: shared pyramid implementation with a1/a2 already resolved (both
// public entry points funnel here so the "given a1/a2, solve" pipeline is
// exercised by both arms — the plan's design point 5).
ClosedFormPyramidResult ComputeClosedFormPyramidInner(double a1, double a2, float h1, float h2, float h3,
                                                      const float dist[6]) {
  ClosedFormPyramidResult r{};

  // Face-number constants: {1, 2, 3..8, 13..18, 23..28}.
  r.face_number[0] = 1;
  r.face_number[1] = 2;
  for (int i = 0; i < 6; i++) {
    r.face_number[2 + i] = 3 + i;
    r.face_number[8 + i] = 13 + i;
    r.face_number[14 + i] = 23 + i;
  }

  bool has_upper = a1 > 0 && h1 > math::kFloatEps;
  bool has_lower = a2 > 0 && h3 > math::kFloatEps;

  r.a1 = has_upper ? static_cast<float>(a1) : -1.0f;
  r.a2 = has_lower ? static_cast<float>(a2) : -1.0f;

  // Zero-volume short-circuit — mirror FillHexCrystalCoef (geo3d.cpp:395).
  const double h2_2 = 0.5 * static_cast<double>(h2);
  if (!has_upper && !has_lower && h2 < math::kFloatEps) {
    return r;
  }

  // Plane coefficients — copy FillHexCrystalCoef layout exactly.
  //   slots 0/1: basal (d filled at end).
  //   slot 2+i: prism.
  //   slot 8+i: upper cone (only if has_upper).
  //   slot 14+i: lower cone (only if has_lower).
  r.plane_coef[0] = 0;
  r.plane_coef[1] = 0;
  r.plane_coef[2] = 1;
  r.plane_coef[3] = 0;
  r.plane_coef[4] = 0;
  r.plane_coef[5] = 0;
  r.plane_coef[6] = -1;
  r.plane_coef[7] = 0;
  using math::kPi_3;
  using math::kPi_6;
  for (int i = 0; i < 6; i++) {
    double x1 = 0.5 * std::cos(-kPi_6 + i * kPi_3);
    double x2 = 0.5 * std::cos(kPi_6 + i * kPi_3);
    double y1 = 0.5 * std::sin(-kPi_6 + i * kPi_3);
    double y2 = 0.5 * std::sin(kPi_6 + i * kPi_3);
    double det = x1 * y2 - x2 * y1;  // = √3/8
    r.plane_coef[(2 + i) * 4 + 0] = static_cast<float>(y2 - y1);
    r.plane_coef[(2 + i) * 4 + 1] = static_cast<float>(x1 - x2);
    r.plane_coef[(2 + i) * 4 + 2] = 0;
    r.plane_coef[(2 + i) * 4 + 3] = static_cast<float>(-static_cast<double>(dist[i]) * det);
    if (has_upper) {
      r.plane_coef[(8 + i) * 4 + 0] = static_cast<float>(a1 * (y2 - y1));
      r.plane_coef[(8 + i) * 4 + 1] = static_cast<float>(a1 * (x1 - x2));
      r.plane_coef[(8 + i) * 4 + 2] = static_cast<float>(det);
      r.plane_coef[(8 + i) * 4 + 3] = static_cast<float>(-(h2_2 + a1 * static_cast<double>(dist[i])) * det);
    }
    if (has_lower) {
      r.plane_coef[(14 + i) * 4 + 0] = static_cast<float>(a2 * (y2 - y1));
      r.plane_coef[(14 + i) * 4 + 1] = static_cast<float>(a2 * (x1 - x2));
      r.plane_coef[(14 + i) * 4 + 2] = static_cast<float>(-det);
      r.plane_coef[(14 + i) * 4 + 3] = static_cast<float>(-(h2_2 + a2 * static_cast<double>(dist[i])) * det);
    }
  }

  // Unit outward face normals.
  r.face_normal[0] = 0;
  r.face_normal[1] = 0;
  r.face_normal[2] = 1;
  r.face_normal[3] = 0;
  r.face_normal[4] = 0;
  r.face_normal[5] = -1;
  for (int slot = 2; slot < kClosedFormPyramidFaceCnt; slot++) {
    double nx = r.plane_coef[slot * 4 + 0];
    double ny = r.plane_coef[slot * 4 + 1];
    double nz = r.plane_coef[slot * 4 + 2];
    double mag = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (mag > 0) {
      r.face_normal[slot * 3 + 0] = static_cast<float>(nx / mag);
      r.face_normal[slot * 3 + 1] = static_cast<float>(ny / mag);
      r.face_normal[slot * 3 + 2] = static_cast<float>(nz / mag);
    }
  }

  // Precompute dist_scaled[i] = (√3/4)·dist[i] — the units used by the LP
  // and by SolveHexCrossSection.
  double dist_scaled[6];
  for (int i = 0; i < 6; i++) {
    dist_scaled[i] = kInsetK * static_cast<double>(dist[i]);
  }

  // Apex heights (LP-max of m); u, v of the apex vertex captured for the
  // apex-collapse special case where SolveHexCrossSection returns empty.
  // Upper and lower use the SAME LP result (both cones have the same
  // horizontal hex — the LP depends only on dist_scaled, not on a1/a2 or
  // h1/h3): compute once, reuse both sides. The apex u/v is identical on
  // both sides (it's a horizontal-plane property).
  ApexLPResult apex{};
  if (has_upper || has_lower) {
    apex = MaxFeasibleInsetLP(dist_scaled);
  }
  double m_apex_upper = has_upper ? apex.m : 0.0;
  double m_apex_lower = has_lower ? apex.m : 0.0;
  double m_at_top = has_upper ? std::min(static_cast<double>(h1) * m_apex_upper, m_apex_upper) : 0.0;
  double m_at_bot = has_lower ? std::min(static_cast<double>(h3) * m_apex_lower, m_apex_lower) : 0.0;
  double z_top = has_upper ? (h2_2 + a1 * m_at_top) : h2_2;
  double z_bot = has_lower ? (-h2_2 - a2 * m_at_bot) : -h2_2;
  r.inset_at_top = static_cast<float>(m_at_top);
  r.inset_at_bottom = static_cast<float>(m_at_bot);
  // Basal d values, matching FillHexCrystalCoef's convention (d = -z_top for
  // upper basal (0,0,1), d = z_bot for lower basal (0,0,-1)).
  r.plane_coef[3] = static_cast<float>(-z_top);
  r.plane_coef[7] = static_cast<float>(z_bot);

  // Zero-volume protection identical to FillHexCrystalCoef's "empty pyramid
  // feasible region" branch (geo3d.cpp:488-494): if BOTH cones exist but
  // the LP gave m_apex=0, the polygon is already collapsed at the shoulder
  // — nothing to build.
  if (has_upper && m_apex_upper <= 0.0 && has_lower && m_apex_lower <= 0.0 && h2 < math::kFloatEps) {
    return r;
  }

  // Assemble candidate z-events. In sorted-unique form.
  double z_events[64];
  int z_n = 0;
  auto add_z = [&](double z) {
    for (int i = 0; i < z_n; i++) {
      if (std::fabs(z_events[i] - z) < 1e-9) {
        return;
      }
    }
    assert(z_n < 64);
    z_events[z_n++] = z;
  };
  add_z(z_bot);
  add_z(z_top);
  if (has_upper && z_top > h2_2 + 1e-12) {
    add_z(h2_2);  // upper shoulder
  }
  if (has_lower && z_bot < -h2_2 - 1e-12) {
    add_z(-h2_2);  // lower shoulder
  }
  if (!has_upper && !has_lower) {
    // Pure prism-shaped body: top = h2/2, bot = -h2/2 (already added).
  } else if (!has_upper) {
    // Only lower cone. Add shoulder at h2/2 (top of prism section = top of body).
    if (h2 > math::kFloatEps) {
      add_z(h2_2);
    }
  } else if (!has_lower) {
    if (h2 > math::kFloatEps) {
      add_z(-h2_2);
    }
  }
  // Cone corner-death events.
  if (has_upper) {
    double d_z[20];
    int n = EnumerateConeDeathZ(dist_scaled, m_at_top, h2_2, a1, /*upper_side=*/true, d_z);
    for (int i = 0; i < n; i++) {
      add_z(d_z[i]);
    }
  }
  if (has_lower) {
    double d_z[20];
    int n = EnumerateConeDeathZ(dist_scaled, m_at_bot, h2_2, a2, /*upper_side=*/false, d_z);
    for (int i = 0; i < n; i++) {
      add_z(d_z[i]);
    }
  }
  std::sort(z_events, z_events + z_n);

  // For each candidate z: solve 2D cross section with eroded dist.
  // Walk corners, resolve to 3D, dedup into vtx pool, associate to faces.
  int vtx_cnt = 0;
  const double kDedupTol =
      5.0 * static_cast<double>(math::kFloatEps) * std::max({ std::fabs(z_top), std::fabs(z_bot), 1.0 });
  bool any_face_present[kClosedFormPyramidFaceCnt]{};

  // A face may participate at multiple z layers (basal + prism + cone edges).
  // For basal (slots 0/1), a vertex counts only at z = z_top / z_bot.
  // For prism (slot 2+i), a vertex counts at any z ∈ [-h2/2, h2/2] AND its
  //   two defining directions include i.
  // For upper cone (slot 8+i), a vertex counts at any z ∈ [h2/2, z_top].
  // For lower cone (slot 14+i), a vertex counts at any z ∈ [z_bot, -h2/2].
  for (int e = 0; e < z_n; e++) {
    double z = z_events[e];
    if (z > z_top + 1e-9 || z < z_bot - 1e-9) {
      continue;
    }
    double m = ComputeInset(z, h2_2, has_upper ? a1 : -1.0, has_lower ? a2 : -1.0);
    double r_side[6];
    for (int i = 0; i < 6; i++) {
      r_side[i] = kInsetK * static_cast<double>(dist[i]) - m;
    }
    HexCrossSection xs = SolveHexCrossSection(r_side);
    if (!xs.any_side_present) {
      // Polygon collapsed to a point at this z — the apex/anti-apex case
      // (happens when m ≈ m_apex, i.e. z = z_top with h1 = 1 or z = z_bot
      // with h3 = 1). Insert a single vertex at (apex.u, apex.v, z) and
      // associate it with EVERY side face in the corresponding half-space:
      // upper cone (if z ≥ h2/2) and/or lower cone (if z ≤ -h2/2), plus the
      // basal face if this z matches z_top or z_bot. Every cone face slot
      // gets the apex as a shared vertex — this is exactly what the
      // production mesh produces at a true apex (a fan of triangles).
      const bool is_upper_apex = has_upper && z >= h2_2 - 1e-9;
      const bool is_lower_apex = has_lower && z <= -h2_2 + 1e-9;
      if (!is_upper_apex && !is_lower_apex) {
        continue;
      }
      int idx = InsertOrFindVertex(apex.u, apex.v, z, r.vtx, &vtx_cnt, kDedupTol);
      if (std::fabs(z - z_top) <= 1e-6) {
        // Basal top exists only if it's a real (>1 vtx) polygon — a single
        // apex vertex means the basal face is degenerate; don't record it.
      }
      if (std::fabs(z - z_bot) <= 1e-6) {
        // Same for basal bottom.
      }
      if (is_upper_apex) {
        for (int i = 0; i < 6; i++) {
          AppendFaceVtx(&r, 8 + i, idx);
          any_face_present[8 + i] = true;
        }
      }
      if (is_lower_apex) {
        for (int i = 0; i < 6; i++) {
          AppendFaceVtx(&r, 14 + i, idx);
          any_face_present[14 + i] = true;
        }
      }
      continue;
    }
    // Determine present_idx[] in i-increasing order.
    int present_idx[6];
    int p_n = 0;
    for (int i = 0; i < 6; i++) {
      if (xs.side_present[i]) {
        present_idx[p_n++] = i;
      }
    }
    // Each xs.corner[k] is the intersection of directions present_idx[k] and
    // present_idx[(k+1) % p_n] (SolveHexCrossSection's emit order). Insert
    // 3D vertex and associate to the faces that pass through it.
    for (int k = 0; k < xs.corner_cnt; k++) {
      double vx = xs.corner_x[k];
      double vy = xs.corner_y[k];
      int idx = InsertOrFindVertex(vx, vy, z, r.vtx, &vtx_cnt, kDedupTol);
      int dir_a = present_idx[k];
      int dir_b = present_idx[(k + 1) % p_n];

      // Basal association.
      if (std::fabs(z - z_top) <= 1e-6) {
        AppendFaceVtx(&r, 0, idx);
        any_face_present[0] = true;
      }
      if (std::fabs(z - z_bot) <= 1e-6) {
        AppendFaceVtx(&r, 1, idx);
        any_face_present[1] = true;
      }
      // Prism association: z inside [-h2/2, h2/2] (inclusive of shoulder).
      if (z >= -h2_2 - 1e-9 && z <= h2_2 + 1e-9) {
        AppendFaceVtx(&r, 2 + dir_a, idx);
        AppendFaceVtx(&r, 2 + dir_b, idx);
        any_face_present[2 + dir_a] = true;
        any_face_present[2 + dir_b] = true;
      }
      // Upper cone: z ∈ [h2/2, z_top].
      if (has_upper && z >= h2_2 - 1e-9 && z <= z_top + 1e-9) {
        AppendFaceVtx(&r, 8 + dir_a, idx);
        AppendFaceVtx(&r, 8 + dir_b, idx);
        any_face_present[8 + dir_a] = true;
        any_face_present[8 + dir_b] = true;
      }
      // Lower cone: z ∈ [z_bot, -h2/2].
      if (has_lower && z >= z_bot - 1e-9 && z <= -h2_2 + 1e-9) {
        AppendFaceVtx(&r, 14 + dir_a, idx);
        AppendFaceVtx(&r, 14 + dir_b, idx);
        any_face_present[14 + dir_a] = true;
        any_face_present[14 + dir_b] = true;
      }
    }
  }
  r.vtx_cnt = vtx_cnt;

  // A face is "present" only if it has ≥ 3 distinct vertices in its polygon
  // (bounds a 2D polygon on its plane).
  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    r.face_present[slot] = any_face_present[slot] && r.face_vtx_cnt[slot] >= 3;
  }
  // If a face isn't present, zero its vtx list.
  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    if (!r.face_present[slot]) {
      r.face_vtx_cnt[slot] = 0;
    }
  }

  // Sort each face's polygon vertices CCW as seen from OUTSIDE (i.e. along
  // the outward face normal). For a face with normal n, project each vertex
  // v onto the plane (subtract v · n · n), pick a 2D orthonormal basis (t1, t2)
  // in the plane (t1 = any vector perpendicular to n, t2 = n × t1), then sort
  // by atan2(v · t2, v · t1). CCW when viewed FROM the +n direction —
  // equivalently the standard convention "outward normal = right-hand rule
  // pointing away from the solid interior".
  for (int slot = 0; slot < kClosedFormPyramidFaceCnt; slot++) {
    if (r.face_vtx_cnt[slot] < 3) {
      continue;
    }
    double nx = r.face_normal[slot * 3 + 0];
    double ny = r.face_normal[slot * 3 + 1];
    double nz = r.face_normal[slot * 3 + 2];
    // Pick t1 = any vector not parallel to n.
    double t1x = 0, t1y = 0, t1z = 0;
    if (std::fabs(nx) < 0.9) {
      t1x = 1;
    } else {
      t1y = 1;
    }
    // t1 = t1 − (t1·n) n; normalize.
    double dot = t1x * nx + t1y * ny + t1z * nz;
    t1x -= dot * nx;
    t1y -= dot * ny;
    t1z -= dot * nz;
    double t1m = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
    t1x /= t1m;
    t1y /= t1m;
    t1z /= t1m;
    // t2 = n × t1.
    double t2x = ny * t1z - nz * t1y;
    double t2y = nz * t1x - nx * t1z;
    double t2z = nx * t1y - ny * t1x;
    // Centroid on face plane.
    double cx = 0, cy = 0, cz = 0;
    int fn = r.face_vtx_cnt[slot];
    for (int k = 0; k < fn; k++) {
      int vi = r.face_vtx[slot][k];
      cx += r.vtx[vi * 3 + 0];
      cy += r.vtx[vi * 3 + 1];
      cz += r.vtx[vi * 3 + 2];
    }
    cx /= fn;
    cy /= fn;
    cz /= fn;
    // Sort by angle.
    struct Item {
      int idx;
      double ang;
    };
    Item items[kClosedFormPyramidMaxFaceVtx];
    for (int k = 0; k < fn; k++) {
      int vi = r.face_vtx[slot][k];
      double dx = r.vtx[vi * 3 + 0] - cx;
      double dy = r.vtx[vi * 3 + 1] - cy;
      double dz = r.vtx[vi * 3 + 2] - cz;
      double a1c = dx * t1x + dy * t1y + dz * t1z;
      double a2c = dx * t2x + dy * t2y + dz * t2z;
      items[k].idx = vi;
      items[k].ang = std::atan2(a2c, a1c);
    }
    std::sort(items, items + fn, [](const Item& x, const Item& y) { return x.ang < y.ang; });
    for (int k = 0; k < fn; k++) {
      r.face_vtx[slot][k] = items[k].idx;
    }
  }

  return r;
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

// ============================================================================
// Pyramid public entry points.
// ============================================================================

ClosedFormPyramidResult ComputeClosedFormPyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3,
                                                 const float dist[6]) {
  // Match FillHexCrystalCoef's legality gate (geo3d.cpp:381-382).
  constexpr float kMinAlpha = 0.1f;
  constexpr float kMaxAlpha = 89.9f;
  double a1 = -1.0;
  double a2 = -1.0;
  if (h1 > math::kFloatEps && upper_alpha >= kMinAlpha && upper_alpha <= kMaxAlpha) {
    a1 = static_cast<double>(math::kSqrt3_4) /
         std::tan(static_cast<double>(upper_alpha) * static_cast<double>(math::kDegreeToRad));
  }
  if (h3 > math::kFloatEps && lower_alpha >= kMinAlpha && lower_alpha <= kMaxAlpha) {
    a2 = static_cast<double>(math::kSqrt3_4) /
         std::tan(static_cast<double>(lower_alpha) * static_cast<double>(math::kDegreeToRad));
  }
  return ComputeClosedFormPyramidInner(a1, a2, h1, h2, h3, dist);
}

ClosedFormPyramidResult ComputeClosedFormPyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4, float h1,
                                                 float h2, float h3, const float dist[6]) {
  // Miller-index path: a1 = i1·c / (2·i4), algebraically direct — no
  // atan→tan roundtrip. Equivalent to the wedge-angle form
  //   a1 = √3/4 / tan(atan(√3/2 · i4/(i1·c))) but preserves float precision
  // exactly (the production geo3d.cpp path takes a lossy trip through both
  // transcendental functions; this closed-form path does not).
  // Legality: i1 = 0 means "no cone this side" (matches CreatePyramidMesh's
  // guard at geo3d.cpp:555, which similarly returns alpha = 0 for i1 = 0).
  double a1 = -1.0;
  double a2 = -1.0;
  if (upper_i1 != 0 && h1 > math::kFloatEps) {
    a1 = static_cast<double>(upper_i1) * static_cast<double>(kIceCrystalC) / (2.0 * static_cast<double>(upper_i4));
    // Sanity gate: reject values outside the legal wedge-angle range's
    // equivalent a1 (a1 → 0 as alpha → 90°; a1 → ∞ as alpha → 0°). Same
    // effective bounds as the wedge path.
    if (!(std::isfinite(a1) && a1 > 0.0)) {
      a1 = -1.0;
    }
  }
  if (lower_i1 != 0 && h3 > math::kFloatEps) {
    a2 = static_cast<double>(lower_i1) * static_cast<double>(kIceCrystalC) / (2.0 * static_cast<double>(lower_i4));
    if (!(std::isfinite(a2) && a2 > 0.0)) {
      a2 = -1.0;
    }
  }
  return ComputeClosedFormPyramidInner(a1, a2, h1, h2, h3, dist);
}

}  // namespace lumice
