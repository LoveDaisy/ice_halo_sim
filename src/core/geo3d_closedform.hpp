// Closed-form hex-crystal geometry — parallel implementation of the geometry
// generation used by the golden-analytic tests and benches. Does NOT touch the
// production path (FillHexCrystalCoef / SolveConvexPolyhedronVtxD / Crystal);
// swap-in belongs to a later subtask.
//
// The hex-crystal family (prism + pyramid) is characterised by six fixed
// horizontal normal directions θᵢ = i·60°: the prism is the degenerate case
// with inset ≡ 0 (a 2D half-plane intersection extruded in z); the pyramid
// adds a piecewise-linear inset curve m(z) with up to two breakpoints
// (z = ±h2/2). At every height z the cross section is the SAME 2D
// half-plane problem over the six directions with per-direction offset
// (√3/4)·(distᵢ − m(z)) — the closed-form pyramid solver reduces to
// repeated calls to the shared 2D solver at a handful of candidate heights.
// See doc/crystal-geometry-representation.md §4 for the design rationale.
//
// Every non-opposite direction pair has determinant sin 60° = √3/2 exactly;
// opposite pairs are exactly parallel (skipped by construction, no threshold).

#ifndef LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_
#define LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_

#include <cstdint>

namespace lumice {

// Exact θᵢ = i·60° horizontal directions, double precision, shared by every
// production/test station that would otherwise recompute cos/sin at these
// fixed compile-time-known angles. Motivation: libm cos/sin drifts by ~1 ULP
// between macOS and glibc; when the closed-form implementation and its
// golden-analytic reference each call libm independently, the two drift in
// the same direction on any single platform and mask their disagreement, but
// the drifts differ across platforms so a golden-analytic comparison that
// passes on macOS still fails on Linux/glibc. Cure: both parties consume a
// SINGLE canonical constant table, defined here in the header both sides
// #include — no libm on any platform, no per-platform reproducibility gap.
//
// Two phases exist in this module — DO NOT conflate them. The pyramid inner
// evaluator (ComputeClosedFormPyramidInner) walks the six hexagon vertices,
// not the six face normals, so its cos/sin arguments are half-phase shifted:
//   kHexFaceCos/Sin[i]: face-normal phase, θᵢ = i·60°
//   kHexVtxCos/Sin[i]:  vertex phase,      θᵢ = i·60° − 30°
// Identity used by the pyramid inner: the "+30° + i·60°" vertex equals
// kHexVtxCos/Sin[(i + 1) % 6] (mod 360°). So face i's two adjacent hexagon
// vertices are kHexVtxCos/Sin[i] and kHexVtxCos/Sin[(i + 1) % 6] — no third
// table needed.
constexpr double kHexSqrt3Half = 0.86602540378443864676;  // √3/2 as the nearest representable double.

constexpr double kHexFaceCos[6] = { 1.0, 0.5, -0.5, -1.0, -0.5, 0.5 };
constexpr double kHexFaceSin[6] = { 0.0, kHexSqrt3Half, kHexSqrt3Half, 0.0, -kHexSqrt3Half, -kHexSqrt3Half };

constexpr double kHexVtxCos[6] = { kHexSqrt3Half, kHexSqrt3Half, 0.0, -kHexSqrt3Half, -kHexSqrt3Half, 0.0 };
constexpr double kHexVtxSin[6] = { -0.5, 0.5, 1.0, 0.5, -0.5, -1.0 };

// Permanent construction-time protection: table values must lie on the unit
// circle. Guards against typos on future edits (the tables encode structural
// hex-family geometry; changing a value is almost never legitimate).
static_assert(kHexFaceCos[0] * kHexFaceCos[0] + kHexFaceSin[0] * kHexFaceSin[0] == 1.0, "kHexFace[0] not unit");
static_assert(kHexFaceCos[3] * kHexFaceCos[3] + kHexFaceSin[3] * kHexFaceSin[3] == 1.0, "kHexFace[3] not unit");
static_assert(kHexVtxCos[2] * kHexVtxCos[2] + kHexVtxSin[2] * kHexVtxSin[2] == 1.0, "kHexVtx[2] not unit");
static_assert(kHexVtxCos[5] * kHexVtxCos[5] + kHexVtxSin[5] * kHexVtxSin[5] == 1.0, "kHexVtx[5] not unit");
// Half-integer entries: √3/2 squared is not exactly 3/4 in binary, so entries
// with ½ and √3/2 components need a small tolerance.
static_assert((kHexFaceCos[1] * kHexFaceCos[1] + kHexFaceSin[1] * kHexFaceSin[1]) - 1.0 < 1e-15,
              "kHexFace[1] not unit");
static_assert(1.0 - (kHexFaceCos[1] * kHexFaceCos[1] + kHexFaceSin[1] * kHexFaceSin[1]) < 1e-15,
              "kHexFace[1] not unit");
static_assert((kHexFaceCos[2] * kHexFaceCos[2] + kHexFaceSin[2] * kHexFaceSin[2]) - 1.0 < 1e-15,
              "kHexFace[2] not unit");
static_assert(1.0 - (kHexFaceCos[2] * kHexFaceCos[2] + kHexFaceSin[2] * kHexFaceSin[2]) < 1e-15,
              "kHexFace[2] not unit");
static_assert((kHexFaceCos[4] * kHexFaceCos[4] + kHexFaceSin[4] * kHexFaceSin[4]) - 1.0 < 1e-15,
              "kHexFace[4] not unit");
static_assert(1.0 - (kHexFaceCos[4] * kHexFaceCos[4] + kHexFaceSin[4] * kHexFaceSin[4]) < 1e-15,
              "kHexFace[4] not unit");
static_assert((kHexFaceCos[5] * kHexFaceCos[5] + kHexFaceSin[5] * kHexFaceSin[5]) - 1.0 < 1e-15,
              "kHexFace[5] not unit");
static_assert(1.0 - (kHexFaceCos[5] * kHexFaceCos[5] + kHexFaceSin[5] * kHexFaceSin[5]) < 1e-15,
              "kHexFace[5] not unit");
static_assert((kHexVtxCos[0] * kHexVtxCos[0] + kHexVtxSin[0] * kHexVtxSin[0]) - 1.0 < 1e-15, "kHexVtx[0] not unit");
static_assert(1.0 - (kHexVtxCos[0] * kHexVtxCos[0] + kHexVtxSin[0] * kHexVtxSin[0]) < 1e-15, "kHexVtx[0] not unit");
static_assert((kHexVtxCos[1] * kHexVtxCos[1] + kHexVtxSin[1] * kHexVtxSin[1]) - 1.0 < 1e-15, "kHexVtx[1] not unit");
static_assert(1.0 - (kHexVtxCos[1] * kHexVtxCos[1] + kHexVtxSin[1] * kHexVtxSin[1]) < 1e-15, "kHexVtx[1] not unit");
static_assert((kHexVtxCos[3] * kHexVtxCos[3] + kHexVtxSin[3] * kHexVtxSin[3]) - 1.0 < 1e-15, "kHexVtx[3] not unit");
static_assert(1.0 - (kHexVtxCos[3] * kHexVtxCos[3] + kHexVtxSin[3] * kHexVtxSin[3]) < 1e-15, "kHexVtx[3] not unit");
static_assert((kHexVtxCos[4] * kHexVtxCos[4] + kHexVtxSin[4] * kHexVtxSin[4]) - 1.0 < 1e-15, "kHexVtx[4] not unit");
static_assert(1.0 - (kHexVtxCos[4] * kHexVtxCos[4] + kHexVtxSin[4] * kHexVtxSin[4]) < 1e-15, "kHexVtx[4] not unit");

// Coarse-grained structural branches walked by the shared 2D cross-section
// solver. Recorded per invocation as a bitmask; the pyramid evaluator OR-unions
// the tags of every invocation into ClosedFormPyramidResult::path_tag_union.
// Tests use the union to assert that specialized configurations
// (shoulder / apex / face-drop) walk the SAME set of branches as regular
// well-conditioned configurations — turning "no special-case branch" from a
// human code-review claim into a CI-enforceable set-equality check. Anchored on
// structural predicates only (not floating-point comparison results), so
// legitimate numerical drift cannot cause a false alarm.
enum ClosedFormHexPathTag : uint16_t {
  kClosedFormPathTagAnyDirDegenerate = 1u << 0,  // any direction had r_side ≤ 0
  kClosedFormPathTagDedupHit = 1u << 1,          // ≥1 candidate corner deduped
  kClosedFormPathTagBounded = 1u << 2,           // ≥3 present directions bounded a polygon
  kClosedFormPathTagAllDirsPresent = 1u << 3,    // all 6 directions bound a face
  kClosedFormPathTagEmpty = 1u << 4,             // feasible region empty
};

// ============================================================================
// Prism family (2 basal + 6 prism side).
// ============================================================================

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
// pyramid evaluator can reuse the field verbatim; the pyramid uses its own
// (distinct) output struct because a pyramid face has variable corner count
// per face, whereas the prism has a single CCW ring shared by every face.
struct ClosedFormPrismResult {
  // ---- Generic (reusable field semantics; the pyramid evaluator has an
  //      analogous field on its own struct, only the numerical source differs)
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

  // ---- Prism-specific degenerate values (identically zero for prism; the
  //      pyramid evaluator carries these as first-class fields on its struct)
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

// ============================================================================
// Pyramid family (2 basal + 6 prism + 6 upper cone + 6 lower cone).
// ============================================================================

// Face count and layout for the pyramid family.
// Face slots (matching IsLegalFace kPyramid: face_number ∈ {1,2,3..8,13..18,23..28}):
//   slot 0 → upper basal        (face_number 1)
//   slot 1 → lower basal        (face_number 2)
//   slot 2+i → prism side face i (face_number 3+i, i = 0..5)
//   slot 8+i → upper cone face i (face_number 13+i, i = 0..5)
//   slot 14+i → lower cone face i (face_number 23+i, i = 0..5)
constexpr int kClosedFormPyramidFaceCnt = 20;
constexpr int kClosedFormPyramidSideCnt = 6;
// Global vertex pool upper bound. Sized to accommodate:
//   - 6 adjacent-direction pairs × up to 4 z-events (basal cut / shoulder ×2 /
//     basal cut or apex) = 24 baseline vertices
//   - Plus cone corner-death events: up to C(6,3)=20 triples per cone × 2 cones
//     = 40 additional per-triple vertices in extreme irregular configs
//   - Plus 2 apex vertices (one per side)
// A defensive upper bound of 96 leaves comfortable slack; overflow is caught
// with an unconditional runtime assert (kept live even under NDEBUG since the
// consequence is silent memory corruption, not a mere check failure).
constexpr int kClosedFormPyramidMaxVtx = 96;
// Per-face polygon vertex-count upper bound. Basal faces are ≤6-gons in the
// regular regime. Cone faces can inflate under irregular dist (multiple death
// events on the same face). 32 is defensive; overflow is caught by
// unconditional assert.
constexpr int kClosedFormPyramidMaxFaceVtx = 32;

// Output of the closed-form pyramid evaluator. Two independent structs (vs
// reusing ClosedFormPrismResult) because pyramid faces have variable corner
// count — prism's single-CCW-ring layout cannot represent that without either
// forcing prism to carry pyramid's complexity or forcing pyramid to smear its
// per-face polygons into a shared ring.
struct ClosedFormPyramidResult {
  // Plane coefficients (a, b, c, d): a·x + b·y + c·z + d ≤ 0.
  // Layout matches FillHexCrystalCoef's output.
  float plane_coef[kClosedFormPyramidFaceCnt * 4];
  // Unit outward normals for each face (parametric).
  float face_normal[kClosedFormPyramidFaceCnt * 3];
  // Face-number constants (parametric, per IsLegalFace kPyramid).
  int face_number[kClosedFormPyramidFaceCnt];
  // face_present[i]: face slot i bounds the body.
  bool face_present[kClosedFormPyramidFaceCnt];

  // De-duped 3D vertex pool. Each vertex is (x, y, z) packed at vtx[i*3+k].
  int vtx_cnt;
  float vtx[kClosedFormPyramidMaxVtx * 3];

  // Per-face CCW vertex indices into the vtx pool. face_vtx_cnt[slot] is 0
  // when face_present[slot] is false.
  int face_vtx_cnt[kClosedFormPyramidFaceCnt];
  int face_vtx[kClosedFormPyramidFaceCnt][kClosedFormPyramidMaxFaceVtx];

  // Cone slopes (a1 = upper, a2 = lower). Negative sentinel (−1.0f) when the
  // corresponding cone is absent (h1 ≤ 0, h3 ≤ 0, or alpha outside legal
  // range).
  float a1;
  float a2;
  // Inset amount at the basal cut heights (z = z_top, z = z_bottom).
  // = 0 when the basal cut sits on the shoulder z = ±h2/2 (apex not reached).
  float inset_at_top;
  float inset_at_bottom;

  // OR-union of ClosedFormHexPathTag bits across every internal SolveHexCrossSection
  // invocation performed while computing this result. Zero when no invocation
  // occurred (e.g. zero-volume short-circuit). Enables the golden-analytic test
  // to assert that specialized configurations (shoulder / apex / face-drop) walk
  // the SAME set of solver branches as regular configurations.
  uint16_t path_tag_union;
};

// Evaluate the closed-form pyramid geometry — direct-wedge entry point.
// Mirrors CreatePyramidMesh(upper_alpha, lower_alpha, h1, h2, h3, dist)
// (geo3d.cpp:563). alpha is in degrees; the legal range [0.1°, 89.9°] matches
// FillHexCrystalCoef's protection. Out-of-range or h1 ≤ 0 drops the upper
// cone (a1 = −1.0f); same for the lower cone.
ClosedFormPyramidResult ComputeClosedFormPyramid(float upper_alpha, float lower_alpha, float h1, float h2, float h3,
                                                 const float dist[6]);

// Miller-index entry point. Mirrors CreatePyramidMesh(upper_idx1, upper_idx4,
// lower_idx1, lower_idx4, h1, h2, h3, dist) (geo3d.cpp:550). Uses the
// algebraically-direct form a = i1·c/(2·i4) — no atan→tan roundtrip. i1 = 0
// signals "no cone this side" (same as CreatePyramidMesh's convention).
ClosedFormPyramidResult ComputeClosedFormPyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4, float h1,
                                                 float h2, float h3, const float dist[6]);

}  // namespace lumice

#endif  // LUMICE_CORE_GEO3D_CLOSEDFORM_HPP_
