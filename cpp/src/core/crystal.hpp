#ifndef SRC_CORE_CRYSTAL_H_
#define SRC_CORE_CRYSTAL_H_

#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "core/def.hpp"
#include "core/geo3d.hpp"

namespace icehalo {

enum class CrystalType {
  kUnknown,
  kPrism,
  kIrregularPrism,
  kPyramid_H3,
  kPyramid_I2H3,
  kPyramid_I4H3,
  kPyramid_A2H3,
  kIrregularPyramid,
  kPyramidStackHalf,
  kCubicPyramid,
  kCustom,
};

namespace v3 {

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
   * @brief Create a hexagonal pyramid crystal with custom Miller indices
   * @param upper_i1 Upper pyramid Miller index i1
   * @param upper_i4 Upper pyramid Miller index i4
   * @param lower_i1 Lower pyramid Miller index i1
   * @param lower_i4 Lower pyramid Miller index i4
   * @param h1 Upper pyramid segment relative height (h1/H1)
   * @param h2 Prism segment height ratio (h2/a)
   * @param h3 Lower pyramid segment relative height (h3/H3)
   * @param dist Face distance array [d1, d2, d3, d4, d5, d6], optional. If nullptr, uses default [1,1,1,1,1,1]
   * @return A Crystal object representing the pyramid
   * @note Miller indices represent (i1, 0, -i1, i4) for the pyramidal face orientation
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
  Crystal(Mesh mesh);

  Crystal(const Crystal& other);
  Crystal(Crystal&& other);
  ~Crystal() = default;

  Crystal& operator=(const Crystal& other);
  Crystal& operator=(Crystal&& other);

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
   * @brief Get triangle edge vectors
   * @return Pointer to edge vector array (6 * triangle_cnt elements: 2 edges * 3 coordinates per triangle)
   */
  const float* GetTriangleEdgeVec() const;

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
   * @brief Get triangle coordinate transformation matrices
   * @return Pointer to transformation matrix array (12 * triangle_cnt elements: 4x3 matrix per triangle)
   */
  const float* GetTriangleCoordTf() const;

  /**
   * @brief Get face number for a given face index
   * @param fid Face index
   * @return Face number (for raypath symmetry)
   */
  IdType GetFn(int fid) const;

  /**
   * @brief Rotate the crystal
   * @param r Rotation to apply
   * @return Reference to this crystal
   */
  Crystal& Rotate(const Rotation& r);

  /**
   * @brief Reduce raypath using symmetry
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @return Reduced raypath
   */
  std::vector<IdType> ReduceRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const;

  /**
   * @brief Expand raypath using symmetry
   * @param rp Raypath (face index sequence)
   * @param symmetry Symmetry flags (P, B, D)
   * @return Expanded raypaths (all symmetric variants)
   */
  std::vector<std::vector<IdType>> ExpandRaypath(const std::vector<IdType>& rp, uint8_t symmetry) const;

  /**
   * @brief Get refractive index for a given wavelength
   * @param wl Wavelength in nanometers
   * @return Refractive index
   */
  float GetRefractiveIndex(float wl) const;

  IdType config_id_ = kInvalidId;

 private:
  void ComputeCacheData();

  Mesh mesh_;

  std::unique_ptr<float[]> cache_data_;
  float* face_v_;         // vertex coordinate of every face. 9 * face_cnt
  float* face_ev_;        // edge vector [v0v1, v0v2]. 6 * face_cnt
  float* face_n_;         // normal of every face. 3 * face_cnt
  float* face_area_;      // area of every face. 1 * face_cnt
  float* face_coord_tf_;  // transform for barycentric coordinate. 12 * face_cnt

  std::unique_ptr<IdType[]> fn_map_;  // fid --> fn
  int fn_period_;                     // for raypath symmetry
};

}  // namespace v3

class Crystal {
 public:
  CrystalType GetType() const;

  int TotalVertexes() const;
  int TotalFaces() const;
  ShortIdType FaceNumber(int idx) const;

  const std::vector<Vec3f>& GetVertexes() const;
  const std::vector<TriangleIdx>& GetFaces() const;
  const std::vector<ShortIdType>& GetFaceNumberTable() const;
  const std::map<ShortIdType, PolygonIdx>& GetMergedFaces() const;

  const float* GetFaceVertex() const;
  const float* GetFaceBaseVector() const;
  const float* GetFaceNorm() const;
  const float* GetFaceArea() const;
  int GetFaceNumberPeriod() const;

  static constexpr float kC = 1.629f;

  /*! @brief Create a regular hexagon prism crystal
   *
   * @param h the height of prism. The diameter of basal face is 1
   * @return a pointer to the crystal
   */
  static CrystalPtrU CreateHexPrism(float h);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param h1 height of top segment. The diameter of middle segment is 1
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static CrystalPtrU CreateHexPyramid(float h1, float h2, float h3);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param i1 Miller index 1. The shape of the pyramid segment is defined by Miller index (a,0,-a,b)
   * @param i4 Miller index 4
   * @param h1 height of top segment.
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static CrystalPtrU CreateHexPyramid(int i1, int i4,                 // Miller index
                                      float h1, float h2, float h3);  // heights

  /*! @brief create a hexagonal pyramid crystal
   *
   * @param upperIdx1 Miller index 1 for top segment.
   * @param upperIdx4 Miller index 4 for top segment.
   * @param lowerIdx1 Miller index 1 for bottom segment.
   * @param lowerIdx4 Miller index 4 for bottom segment.
   * @param h1 height of top segment.
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static CrystalPtrU CreateHexPyramid(int upper_idx1, int upper_idx4,  // upper Miller index
                                      int lower_idx1, int lower_idx4,  // lower Miller index
                                      float h1, float h2, float h3);   // heights

  /**
   * @brief create a hexagonal pyramid crystal
   *
   * @param angle1 wedge angle for upper pyramid. it is the angle between pyramidal face and prism face.
   *        for regular prism crystals, this angle is 90 degree.
   * @param angle2 wedge angle for lower pyramid
   * @param h1 height of top segment
   * @param h2 height of middle segment
   * @param h3 height of lower segment
   * @return a pointer to the crystal.
   */
  static CrystalPtrU CreateHexPyramid(float angle1, float angle2,     // wedge angle
                                      float h1, float h2, float h3);  // heights

  /*! @brief Create a hexagonal half-stacked pyramid crystal
   *
   * @param upperIdx1 Miller index 1 for upper segment.
   * @param upperIdx4 Miller index 4 for upper segment.
   * @param lowerIdx1 Miller index 1 for lower segment.
   * @param lowerIdx4 Miller index 4 for lower segment.
   * @param h1 height for upper segment. The diameter of basal is 1.
   * @param h2 height for lower segment.
   * @param h3 height for prism segment.
   * @return a pointer to the crystal.
   */
  static CrystalPtrU CreateHexPyramidStackHalf(int upper_idx1, int upper_idx4,  // upper Miller index
                                               int lower_idx1, int lower_idx4,  // lower Miller index
                                               float h1, float h2, float h3);   // heights

  /* Cubic pyramid (crystal type of Ic) */
  static CrystalPtrU CreateCubicPyramid(float h1, float h2);

  /*! @brief Create an irregular hexagon prism crystal
   *
   * @param dist defines the distances from origin to each face
   * @param h defines the height / diameter
   * @return
   */
  static CrystalPtrU CreateIrregularHexPrism(const float* dist, float h);

  /*! @brief Create a irregular hexagon pyramid crystal
   *
   * @param dist defines the distance from origin to each face. Must contains 6 numbers. The distance of a
   *             regular hexagon is defined as 1.
   * @param idx defines the Miller index of upper and lower pyramidal segments. Must contains 4 numbers.
   *            idx[0] and idx[1] are for upper segment. idx[2] and idx[3] are for lower segment.
   * @param h defines the height of each segment.
   *          h[0] and h[2] are the heights of upper and lower pyramidal segments, defined as height / H, where
   *          H is the maximum possible height.
   *          h[1] are the heights of middle cylindrical segment, defined as height / a, where a is the
   *          diameter of original basal face.
   * @return
   */
  static CrystalPtrU CreateIrregularHexPyramid(const float* dist, const int* idx, const float* h);

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes of the crystal
   * @param faces the faces of the crystal
   * @return
   */
  static CrystalPtrU CreateCustomCrystal(const std::vector<Vec3f>& pts,           // vertex points
                                         const std::vector<TriangleIdx>& faces);  // face indices

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes
   * @param faces the faces
   * @param face_number_map the face number map
   * @return
   */
  static CrystalPtrU CreateCustomCrystal(const std::vector<Vec3f>& pts,                       // vertex points
                                         const std::vector<TriangleIdx>& faces,               // face indices
                                         const std::vector<ShortIdType>& face_number_table);  // face to face-number

 protected:
  void InitBasicData();
  void InitPrimaryFaceNumber();
  void PruneRedundantFaces();
  void RefineFaceNumber();
  void MergeFaces();

  bool IsCoplanar(size_t idx1, size_t idx2) const;
  bool IsCounterCoplanar(size_t idx1, size_t idx2) const;
  bool IsConnected(size_t idx1, size_t idx2) const;
  bool IsAdjacent(size_t idx1, size_t idx2) const;
  int MatchedVertexes(size_t idx1, size_t idx2) const;

  CrystalType type_;
  std::vector<Vec3f> vertexes_;
  std::vector<TriangleIdx> faces_;
  std::vector<ShortIdType> face_number_table_;
  std::map<ShortIdType, PolygonIdx> merged_faces_;
  int face_number_period_;

  std::unique_ptr<float[]> face_bases_;
  std::unique_ptr<float[]> face_vertexes_;
  std::unique_ptr<float[]> face_norm_;
  std::unique_ptr<float[]> face_area_;

 private:
  /*! @brief Constructor, given vertexes and faces
   *
   * @param vertexes
   * @param faces
   * @param type
   */
  Crystal(std::vector<Vec3f> vertexes,     // vertex points
          std::vector<TriangleIdx> faces,  // face indices
          CrystalType type);               // crystal type

  /*! @brief Constructor, given vertexes, faces and face_number_map
   *
   * @param vertexes
   * @param faces
   * @param face_number_map the normalized face ID, or face number.
   *        see [Face numbers](https://www.atoptics.co.uk/halo/fnum.htm)
   *        and [Pyramidal Crystal Face Numbers](https://www.atoptics.co.uk/halo/fnumpyr.htm)
   * @param type
   */
  Crystal(std::vector<Vec3f> vertexes,                 // vertex points
          std::vector<TriangleIdx> faces,              // face indices
          std::vector<ShortIdType> face_number_table,  // face to face number
          CrystalType type);                           // crystal type
};


struct RayPath {
  static constexpr size_t kDefaultCapacity = 7;

  size_t len;
  size_t capacity;
  ShortIdType* ids;
  bool is_permanent;

  RayPath();
  explicit RayPath(size_t reserve_len);
  RayPath(std::initializer_list<ShortIdType> ids);
  RayPath(const RayPath& other);
  RayPath(RayPath&& other) noexcept;
  ~RayPath();

  void Clear();
  void PrependId(ShortIdType id);
  RayPath MakePermanentCopy() const;

  // convenience for the range-based for loop
  ShortIdType* begin() const noexcept { return ids; }
  ShortIdType* end() const noexcept { return ids + len; }

  RayPath& operator=(const RayPath& other) noexcept;
  RayPath& operator=(RayPath&& other) noexcept;
  RayPath& operator<<(ShortIdType id);
  bool operator==(const RayPath& other) const noexcept;
};


std::vector<RayPath> MakeSymmetryExtension(
    const RayPath& curr_ray_path,       // current ray path. not include crystal id and kInvalidFaceNumber
    const CrystalContext* crystal_ctx,  // crystal
    uint8_t symmetry_flag);             // symmetry


std::pair<RayPath, size_t> NormalizeRayPath(RayPath ray_path, const ProjectContextPtr& proj_ctx, uint8_t symmetry_flag);

}  // namespace icehalo


#endif  // SRC_CORE_CRYSTAL_H_
