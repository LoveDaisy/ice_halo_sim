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
 * Legal sets (matching FillHexFnMap in crystal.cpp):
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
 * triangles). This is the single source of truth for the "is this mesh a
 * legal closed polyhedron" predicate used both by the Crystal factory
 * boundary (crystal.cpp, rejects malformed meshes before they reach
 * BuildPolygonFaceData) and by test code that needs to assert the same
 * invariant against known inputs — declared here so both consumers share one
 * implementation instead of each maintaining a parallel copy that can drift.
 *
 * @note Necessary but not sufficient: a self-intersecting mesh whose V/F
 *       counts happen to satisfy the Euler formula would pass. See
 *       doc/numerical-robustness.md for the argmax/relative-tolerance
 *       conventions this predicate follows.
 */
bool IsClosedTriMesh(size_t v, size_t f);

/**
 * @brief Compute face-number map for a hexagonal crystal mesh.
 *
 * Precondition: @p face_n must contain unit per-triangle normals
 *   (stride = 3 floats per triangle, each normal length == 1).
 *
 * Output values match the face-number convention used by raypath filters:
 *   basal = 1 / 2; prism = 3..8; upper pyramidal = 13..18; lower = 23..28.
 * Returns kInvalidId for triangles whose normals do not match any hex face.
 *
 * @param face_cnt  Number of triangles.
 * @param face_n    Unit per-triangle normals, length = face_cnt * 3.
 * @param fn_map    Output buffer, length = face_cnt.
 */
void FillHexFnMap(size_t face_cnt, const float* face_n, IdType* fn_map);

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
 * @details This class represents a crystal with triangular mesh.
 *          It provides methods for creating crystals and querying geometry data.
 *          The crystal is represented as a triangular mesh for efficient ray-crystal intersection.
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

  /**
   * @brief Construct crystal from vertex and triangle data
   * @param vtx_cnt Number of vertices
   * @param vtx Vertex coordinate array (3 * vtx_cnt elements)
   * @param triangle_cnt Number of triangles
   * @param triangle_idx Triangle index array (3 * triangle_cnt elements)
   */
  Crystal(size_t vtx_cnt, std::unique_ptr<float[]> vtx, size_t triangle_cnt, std::unique_ptr<int[]> triangle_idx);

  /**
   * @brief Construct crystal from mesh
   * @param mesh Mesh object containing vertex and triangle data
   */
  explicit Crystal(Mesh mesh);

  Crystal(const Crystal& other);
  Crystal(Crystal&& other) noexcept;
  ~Crystal() = default;

  Crystal& operator=(const Crystal& other);
  Crystal& operator=(Crystal&& other) noexcept;

  /**
   * @brief Get total number of triangles
   * @return Number of triangles in the crystal mesh
   */
  size_t TotalTriangles() const;

  /**
   * @brief Get total number of vertices
   * @return Number of vertices in the crystal mesh
   */
  size_t TotalVertices() const;

  /**
   * @brief Get triangle vertex coordinates
   * @return Pointer to vertex coordinate array (9 * triangle_cnt elements: 3 vertices * 3 coordinates per triangle)
   */
  const float* GetTriangleVtx() const;

  /**
   * @brief Get triangle normal vectors
   * @return Pointer to normal vector array (3 * triangle_cnt elements: 3 coordinates per triangle)
   */
  const float* GetTriangleNormal() const;

  /**
   * @brief Get triangle areas
   * @return Pointer to area array (triangle_cnt elements)
   */
  const float* GetTirangleArea() const;

  /**
   * @brief Get face number for a given triangle face index
   * @param fid Triangle face index
   * @return Face number (for raypath symmetry); kInvalidId if @p fid is out of range
   */
  IdType GetFn(int fid) const;

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
  const int* GetPolygonFaceTriId() const;

  // Test observability: process-global count of degenerate polygon faces
  // dropped by BuildPolygonFaceData (the count/stride "shrink" path that used
  // to corrupt copy/move). Lets regression tests assert they actually
  // exercised the shrink branch instead of passing vacuously. Thread-safe;
  // reset before a measured sweep, read after.
  static uint64_t DegenerateShrinkCount();
  static void ResetDegenerateShrinkCount();

  IdType config_id_ = kInvalidId;

 private:
  void ComputeCacheData();
  void BuildPolygonFaceData(const float* plane_coef, size_t plane_cnt);

  // Shift prism/pyramid faces so the first non-basal pri index becomes 0.
  // Basal faces (x < 3) are passed through unchanged. Returns the input
  // verbatim when no non-basal face is present.
  std::vector<IdType> PCanonicalShift(const std::vector<IdType>& rp) const;

  Mesh mesh_;

  std::unique_ptr<float[]> cache_data_;
  float* face_v_;     // vertex coordinate of every face. 9 * face_cnt
  float* face_n_;     // normal of every face. 3 * face_cnt
  float* face_area_;  // area of every face. 1 * face_cnt

  std::unique_ptr<IdType[]> fn_map_;  // fid --> fn
  int fn_period_;                     // for raypath symmetry

  // Polygon face data for per-plane intersection
  size_t poly_face_cnt_ = 0;
  std::unique_ptr<float[]> poly_face_data_;  // single allocation: n(3*cnt) + d(cnt) + tri_id(cnt as float)
  float* poly_face_n_ = nullptr;             // unit normals, 3 * poly_face_cnt_
  float* poly_face_d_ = nullptr;             // plane distances, poly_face_cnt_
  int* poly_face_tri_id_ = nullptr;          // representative triangle ID, poly_face_cnt_
};


namespace detail {
// Internal — not part of public API.

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
