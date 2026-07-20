// Closed-form hex-crystal geometry — subtask closed-form-prism of the
// geometry-closed-form-representation refactor.
//
// This header exposes a stateless evaluator for the prism family. It does NOT
// touch the production path (FillHexCrystalCoef / SolveConvexPolyhedronVtxD /
// Crystal); it is a parallel implementation used only by golden-analytic tests
// and benches. Swap-in belongs to a later subtask.
//
// The prism is the degenerate case of the hex family whose inset curve is
// identically zero: the crystal is a 2D half-plane intersection over six fixed
// directions θᵢ = i·60°, extruded in z. Every non-opposite face pair has
// determinant sin 60° = √3/2 exactly, and opposite faces are exactly parallel
// (skipped by construction, no threshold). See doc/crystal-geometry-representation.md
// §4 for the design rationale.

#ifndef LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_
#define LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_

namespace lumice {

// Face count and layout for the prism family.
// Face slots (matching IsLegalFace kPrism: face_number 1..8):
//   slot 0 → upper basal   (face_number 1)
//   slot 1 → lower basal   (face_number 2)
//   slot 2+i → side face i (face_number 3+i, i = 0..5)
constexpr int kClosedFormPrismFaceCnt = 8;
constexpr int kClosedFormPrismSideCnt = 6;
// C(6,2) − 3 (opposite pairs skipped): maximum feasible corners in 2D.
constexpr int kClosedFormPrismMaxCorners = 12;

// Output of the closed-form prism evaluator. Fields are grouped by whether the
// pyramid subtask can reuse the field verbatim; the pyramid will add its own
// inset-curve state on top of the generic fields.
struct ClosedFormPrismResult {
  // ---- Generic (reusable by the pyramid subtask; only the numerical source differs)
  // Plane coefficients (a, b, c, d) so that the bounded half-space is
  //   a·x + b·y + c·z + d ≤ 0
  // Layout mirrors FillHexCrystalCoef's output so a future swap can be a memcpy.
  float plane_coef[kClosedFormPrismFaceCnt * 4];
  // Unit outward normals for each face (parametric, not solved).
  float face_normal[kClosedFormPrismFaceCnt * 3];
  // Face-number constants (parametric, per IsLegalFace kPrism). Filled with 1..8.
  int face_number[kClosedFormPrismFaceCnt];
  // face_present[i] iff face i bounds the body (≥2 distinct feasible corners lie on it).
  bool face_present[kClosedFormPrismFaceCnt];
  // CCW corner ring of the 2D cross section. corner_cnt is the number of
  // distinct side-face corners; entry k is the intersection of the k-th and
  // (k+1)-th present side faces in i-increasing order (wrap at end).
  int corner_cnt;
  float corner_x[kClosedFormPrismMaxCorners];
  float corner_y[kClosedFormPrismMaxCorners];

  // ---- Prism-specific degenerate values (pyramid will replace these with real values)
  // The inset curve — how much the side profile shrinks with |z| — is
  // identically 0 for the prism. Kept as a placeholder so the pyramid subtask
  // can extend this struct without changing the reader shape.
  float inset_at_top;     // = 0 for prism
  float inset_at_bottom;  // = 0 for prism
};

// Evaluate the closed-form prism geometry from crystal height `h` and six side
// distances dist[6] (same distance convention as FillHexCrystalCoef).
//
// Degenerate short-circuit: `h ≤ math::kFloatEps` produces an all-false result
// (corner_cnt = 0, every face_present = false), aligning with
// FillHexCrystalCoef's zero-volume early exit.
//
// This function does not consult the production path.
ClosedFormPrismResult ComputeClosedFormPrism(float h, const float dist[6]);

}  // namespace lumice

#endif  // LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_
