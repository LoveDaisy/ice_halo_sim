#ifndef SRC_CORE_CRYSTAL_H_
#define SRC_CORE_CRYSTAL_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/crystal_kind.hpp"
#include "core/def.hpp"
#include "core/geo3d.hpp"

namespace lumice {

/**
 * @brief Test whether @p face is a legal face number on the given crystal kind.
 *
 * Legal sets (matching the closed-form face-number tables in
 * geo3d_closedform.cpp):
 *   - kPrism:   {1, 2, 3, 4, 5, 6, 7, 8}
 *   - kPyramid: {1, 2, 3..8, 13..18, 23..28}
 *
 * Used by config/raypath_validation.cpp to enforce type-specific face-number
 * bounds before a raypath filter text is committed.
 *
 * @note The implementation deliberately enumerates all CrystalKind values and
 *       uses an unreachable/assert guard for unhandled kinds; new enum values
 *       must be added to the switch to avoid silent relaxation of validation.
 */
bool IsLegalFace(CrystalKind kind, int face);

/**
 * @brief Closed 2-manifold Euler-characteristic gate for a triangle mesh.
 *
 * Checks `V - E + F == 2` with `E = 3F/2` (each edge shared by exactly 2
 * triangles). This is a pure V/F predicate over an already-built mesh.
 *
 * @note It is no longer a factory gate. The closed-form representation decides
 *       validity while constructing (surviving contour corners >= 3, plus a
 *       non-degenerate height interval), so there is no "build a mesh, then
 *       check whether the build went wrong" step left to guard. What remains
 *       here is a test-only predicate for asserting the invariant against
 *       known inputs.
 *
 * @note Necessary but not sufficient: a self-intersecting mesh whose V/F
 *       counts happen to satisfy the Euler formula would pass. See
 *       doc/numerical-robustness.md for the argmax/relative-tolerance
 *       conventions this predicate follows.
 */
bool IsClosedTriMesh(size_t v, size_t f);

// Crystal's closed-form flat POD representation. Populated by the closed-form
// factory paths (CreatePrism / CreatePyramid, once relined onto
// ComputeClosedFormPrism / ComputeClosedFormPyramid — see plan Steps 2/3).
// `face_cnt == 0` sentinels "not populated" (custom-mesh path or
// default-constructed instance).
//
// Slot convention matches the corresponding ClosedFormXxxResult in
// core/geo3d_closedform.hpp:
//   - prism (face_cnt == 8):  slot 0/1 = basal (fn 1/2); slot 2+i = side (fn 3+i)
//   - pyramid (face_cnt == 20): slot 0/1 = basal; slot 2+i = prism side;
//     slot 8+i = upper cone (fn 13+i); slot 14+i = lower cone (fn 23+i).
//
// Max dims are dimensioned to the pyramid worst case. crystal.cpp carries a
// static_assert bridging these constants to the ClosedFormPyramid capacity so
// closed-form growth is caught at compile time, not silently truncated.
constexpr int kCrystalGeomMaxFaces = 20;
// Per-face CCW vertex capacity. The hexagonal pyramid/prism family's real worst
// case is 7 (measured: max present face_vtx_cnt over a 2M-sample fuzz spanning
// wide/negative/near-zero face distances + the known near-coincident-vertex
// inputs; basal ≤6-gon, cone faces inflate to 7 under irregular dist). 12 keeps
// a comfortable margin; the closed-form evaluator's AppendFaceVtx hard-aborts
// (never silently truncates) if a face ever exceeds kClosedFormPyramidMaxFaceVtx
// (== 12, kept in lockstep via the static_assert in crystal.cpp). This directly
// shrinks CrystalGeom's by-value copy cost (face_vtx dominates its footprint).
constexpr int kCrystalGeomMaxVtxPerFace = 12;

struct CrystalGeom {
  int face_cnt = 0;
  // Plane coefficients (a, b, c, d) so a·x + b·y + c·z + d ≤ 0 is the bounded
  // half-space. Layout mirrors FillHexCrystalCoef's output.
  float plane_coef[kCrystalGeomMaxFaces * 4]{};
  // Unit outward normals per face slot.
  float face_normal[kCrystalGeomMaxFaces * 3]{};
  // Face-number constants (parametric, per IsLegalFace convention).
  int face_number[kCrystalGeomMaxFaces]{};
  // face_present[slot]: whether this slot bounds the body.
  bool face_present[kCrystalGeomMaxFaces]{};
  // Per-face CCW vertex count. 0 iff face_present[slot] is false.
  int face_vtx_cnt[kCrystalGeomMaxFaces]{};
  // Per-face CCW vertex coords, packed as (x, y, z) triples in
  // face_vtx[slot * kCrystalGeomMaxVtxPerFace * 3 + k * 3 + xyz].
  float face_vtx[kCrystalGeomMaxFaces * kCrystalGeomMaxVtxPerFace * 3]{};
};

enum class CrystalType {
  kUnknown,
  kPrism,
  kIrregularPrism,
  kPyramid_H3,    // NOLINT(readability-identifier-naming)
  kPyramid_I2H3,  // NOLINT(readability-identifier-naming)
  kPyramid_I4H3,  // NOLINT(readability-identifier-naming)
  kPyramid_A2H3,  // NOLINT(readability-identifier-naming)
  kIrregularPyramid,
  kPyramidStackHalf,
  kCubicPyramid,
  kCustom,
};


class Crystal;
using CrystalPtrS = std::shared_ptr<Crystal>;
using CrystalPtrU = std::unique_ptr<Crystal>;

/**
 * @brief Crystal geometry representation and operations
 * @details This class represents a crystal via its closed-form flat-POD geometry
 *          (`cf_geom_`: planes + face-present mask + per-face CCW corners) plus
 *          the polygon-face data derived from it (normals, plane distances, and
 *          face numbers for slab traversal). The class no longer stores a
 *          triangle mesh: entry-point sampling consumes `cf_geom_` corners
 *          directly, and the only triangulating consumer (C-API geometry export)
 *          builds a `Mesh` on demand via `detail::BuildMeshFromCfGeom`.
 */
class Crystal {
 public:
  /**
   * @brief Create a hexagonal prism crystal
   * @param h Height-to-diameter ratio (h/a), where h is the prism height and a is the base diameter
   * @return A Crystal object representing the prism
   * @note The crystal uses default face distances [1,1,1,1,1,1] for a regular hexagon
   */
  static Crystal CreatePrism(float h);

  /**
   * @brief Create a hexagonal prism crystal with custom face distances
   * @param h Height-to-diameter ratio (h/a)
   * @param fd Face distance array [d1, d2, d3, d4, d5, d6], optional. If nullptr, uses default [1,1,1,1,1,1]
   * @return A Crystal object representing the prism
   * @note Face distance is the ratio of actual face distance to a regular hexagon distance
   * @warning The fd array must contain exactly 6 elements if provided
   */
  static Crystal CreatePrism(float h, const float* fd);

  /**
   * @brief Create a hexagonal pyramid crystal
   * @param h1 Upper pyramid segment relative height (h1/H1), range [0.0, 1.0]
   * @param h2 Prism segment height ratio (h2/a)
   * @param h3 Lower pyramid segment relative height (h3/H3), range [0.0, 1.0]
   * @return A Crystal object representing the pyramid
   * @note Uses default Miller indices [1,0,1] for both upper and lower pyramid segments
   */
  static Crystal CreatePyramid(float h1, float h2, float h3);

  /**
   * @brief Create a hexagonal pyramid crystal with wedge angles
   * @param upper_alpha Upper pyramid wedge angle in degrees (angle between pyramidal face and c-axis)
   * @param lower_alpha Lower pyramid wedge angle in degrees
   * @param h1 Upper pyramid segment relative height (h1/H1)
   * @param h2 Prism segment height ratio (h2/a)
   * @param h3 Lower pyramid segment relative height (h3/H3)
   * @param dist Face distance array [d1, d2, d3, d4, d5, d6], must not be nullptr
   * @return A Crystal object representing the pyramid
   * @note Wedge angles outside [0.1°, 89.9°] cause the corresponding pyramid segment to be skipped
   */
  static Crystal CreatePyramid(float upper_alpha, float lower_alpha,  // wedge angle (degrees)
                               float h1, float h2, float h3,          // height
                               const float* dist);                    // face distance

  /**
   * @brief Create a hexagonal pyramid crystal with wedge angles (default face distances)
   * @param upper_alpha Upper pyramid wedge angle in degrees
   * @param lower_alpha Lower pyramid wedge angle in degrees
   * @param h1 Upper pyramid segment relative height (h1/H1)
   * @param h2 Prism segment height ratio (h2/a)
   * @param h3 Lower pyramid segment relative height (h3/H3)
   * @return A Crystal object representing the pyramid
   */
  static Crystal CreatePyramid(float upper_alpha, float lower_alpha,  // wedge angle (degrees)
                               float h1, float h2, float h3);         // height

  /**
   * @brief Create a hexagonal pyramid crystal with custom Miller indices
   * @param upper_i1 Upper pyramid Miller index i1
   * @param upper_i4 Upper pyramid Miller index i4
   * @param lower_i1 Lower pyramid Miller index i1
   * @param lower_i4 Lower pyramid Miller index i4
   * @param h1 Upper pyramid segment relative height (h1/H1)
   * @param h2 Prism segment height ratio (h2/a)
   * @param h3 Lower pyramid segment relative height (h3/H3)
   * @param dist Face distance array [d1, d2, d3, d4, d5, d6], must not be nullptr
   * @return A Crystal object representing the pyramid
   * @note Miller indices represent (i1, 0, -i1, i4). If i1 == 0, the corresponding pyramid segment is skipped.
   */
  static Crystal CreatePyramid(int upper_i1, int upper_i4, int lower_i1, int lower_i4,  // Miller index
                               float h1, float h2, float h3,                            // height
                               const float* dist);                                      // face distance

  /**
   * @brief Default constructor
   * @note Creates an empty crystal
   */
  Crystal();

  Crystal(const Crystal& other);
  Crystal(Crystal&& other) noexcept;
  ~Crystal() = default;

  Crystal& operator=(const Crystal& other);
  Crystal& operator=(Crystal&& other) noexcept;

  /**
   * @brief Get face number for a given polygon face index
   * @param poly_idx Polygon face index (0..PolygonFaceCount()-1)
   * @return Face number (for raypath symmetry); kInvalidId if @p poly_idx is out of range
   */
  IdType GetFn(IdType poly_idx) const;

  /**
   * @brief Get the face-number period used for symmetry reductions.
   * @return Period (typically 6 for hexagonal prism/pyramid); negative for custom crystals
   *         where symmetry reductions do not apply.
   */
  int FnPeriod() const { return fn_period_; }

  /**
   * @brief Reduce raypath using symmetry
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @return Reduced raypath
   */
  std::vector<IdType> ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const;

  /**
   * @brief Reduce raypath using symmetry with explicit D parameters
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @param sigma_a σ-mirror parameter (0..5); ignored when d_applicable=false
   * @param d_applicable Whether D symmetry should be applied
   * @return Reduced raypath
   */
  std::vector<IdType> ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                    bool d_applicable) const;

  /**
   * @brief Expand raypath using symmetry
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @return Expanded raypaths (all symmetric variants)
   */
  std::vector<std::vector<IdType>> ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const;

  /**
   * @brief Expand raypath using symmetry with explicit D parameters
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @param sigma_a σ-mirror parameter (0..5); ignored when d_applicable=false
   * @param d_applicable Whether D symmetry should be applied
   * @return Expanded raypaths (all symmetric variants)
   */
  std::vector<std::vector<IdType>> ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry, int sigma_a,
                                                 bool d_applicable) const;

  /**
   * @brief Get refractive index for a given wavelength
   * @param wl Wavelength in nanometers
   * @return Refractive index
   */
  float GetRefractiveIndex(float wl) const;

  size_t PolygonFaceCount() const;
  const float* GetPolygonFaceNormal() const;
  const float* GetPolygonFaceDist() const;

  // Closed-form flat POD accessor. Returns cf_geom_.face_cnt == 0 for a
  // default-constructed instance (including the degenerate-factory
  // short-circuit); > 0 for closed-form prism/pyramid crystals.
  const CrystalGeom& CfGeom() const { return cf_geom_; }

  IdType config_id_ = kInvalidId;

 private:
  // Populate poly_face_data_ (normals + plane distances) and poly_face_fn_
  // (per-polygon-face face number) directly from cf_geom_. The closed-form
  // output already carries the parametric face-number and per-slot presence;
  // there is nothing to reverse-engineer (no argmax reversal, no triangle
  // mesh). See crystal.cpp for the design rationale.
  void PopulateFromCfGeom();
  // Shared closed-form prism entry point behind both CreatePrism overloads.
  // Runs ComputeClosedFormPrism → 386.2 validity gate → CrystalGeom adapter →
  // fan-triangulation → PopulateFromCfGeom. Returns an empty crystal when the
  // gate rejects (matches the legacy RejectMalformed downstream contract).
  static Crystal MakePrismClosedForm(float h, const float dist[6], const char* factory);

  // Same shape for the pyramid family, entered from the alpha overload
  // (Miller-index overload delegates through the alpha overload today, so this
  // single entry covers both public paths). Uses the direct-wedge closed-form
  // evaluator; the validity gate is "≥4 present face slots" (see
  // IsValidClosedFormPyramid in crystal.cpp).
  static Crystal MakePyramidClosedForm(float upper_alpha, float lower_alpha, float h1, float h2, float h3,
                                       const float dist[6], const char* factory);

  // Shift prism/pyramid faces so the first non-basal pri index becomes 0.
  // Basal faces (x < 3) are passed through unchanged. Returns the input
  // verbatim when no non-basal face is present.
  std::vector<IdType> PCanonicalShift(const std::vector<IdType>& rp) const;

  int fn_period_ = -1;  // for raypath symmetry

  // Polygon face data for per-plane intersection
  size_t poly_face_cnt_ = 0;
  std::unique_ptr<float[]> poly_face_data_;  // single allocation: n(3*cnt) + d(cnt)
  float* poly_face_n_ = nullptr;             // unit normals, 3 * poly_face_cnt_
  float* poly_face_d_ = nullptr;             // plane distances, poly_face_cnt_

  // Face-number per polygon face — direct parametric constant from
  // cf_geom_.face_number[slot], populated by PopulateFromCfGeom. Fully replaces
  // the previous "poly_face_tri_id_[poly] → fn_map_[tri]" two-hop reversal:
  // fn is now stored directly at its natural key (polygon face) rather than
  // reconstructed from an argmax-selected representative triangle.
  std::unique_ptr<IdType[]> poly_face_fn_;

  // Closed-form flat POD geometry. Populated by CreatePrism/CreatePyramid.
  // face_cnt == 0 for a default-constructed instance. Embedded by value (POD) —
  // copy/move ctors pick it up trivially via the member-wise path in the
  // initializer lists in crystal.cpp.
  CrystalGeom cf_geom_{};
};


namespace detail {
// Internal — not part of public API.

// On-demand triangulation of a CrystalGeom: deduped 3D vertex pool + fixed-fan
// triangle indices, plus a per-triangle "which face slot it came from" table
// (len == mesh.GetTriangleCnt()). Fan rule (v0, v[i-1], v[i]) matches
// detail::BuildEntrySubTris and the T1 analytic oracle. This is the *only*
// triangulating path left; it is a pure function of cf_geom_, called on the
// cold path only (C-API geometry export + white-box tests), never in the
// MakeCrystal hot path.
struct BuiltMesh {
  Mesh mesh;
  std::vector<int> tri_face_slot;
};
BuiltMesh BuildMeshFromCfGeom(const CrystalGeom& g);

// Returns true if roll_mean_deg is a multiple of 30° (within FloatEqual precision).
bool IsRollMeanAtMultipleOf30(const AxisDistribution& d);

// Compute σ-mirror parameter a (0..5) from roll_mean_deg.
// Formula: n = ((int)round(roll_mean / 30.0f) % 6 + 6) % 6; a = (6 - n) % 6.
int ComputeSigmaA(float roll_mean_deg);

// Returns true if D symmetry is applicable for the given axis distribution.
// Requires azimuth full-360° uniform AND roll mean at a multiple of 30°.
bool IsDApplicable(const AxisDistribution& d);
}  // namespace detail


}  // namespace lumice

#endif  // SRC_CORE_CRYSTAL_H_
