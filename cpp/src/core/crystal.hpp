#ifndef SRC_CORE_CRYSTAL_H_
#define SRC_CORE_CRYSTAL_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "core/core_def.hpp"
#include "core/math.hpp"

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

class Crystal {
 public:
  CrystalType GetType() const;

  int TotalVertexes() const;
  int TotalFaces() const;
  ShortIdType FaceNumber(int idx) const;

  const std::vector<math::Vec3f>& GetVertexes() const;
  const std::vector<math::TriangleIdx>& GetFaces() const;
  const std::vector<ShortIdType>& GetFaceNumberTable() const;
  const std::map<ShortIdType, math::PolygonIdx>& GetMergedFaces() const;

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
  static CrystalPtrU CreateCustomCrystal(const std::vector<math::Vec3f>& pts,           // vertex points
                                         const std::vector<math::TriangleIdx>& faces);  // face indices

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes
   * @param faces the faces
   * @param face_number_map the face number map
   * @return
   */
  static CrystalPtrU CreateCustomCrystal(const std::vector<math::Vec3f>& pts,                 // vertex points
                                         const std::vector<math::TriangleIdx>& faces,         // face indices
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
  std::vector<math::Vec3f> vertexes_;
  std::vector<math::TriangleIdx> faces_;
  std::vector<ShortIdType> face_number_table_;
  std::map<ShortIdType, math::PolygonIdx> merged_faces_;
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
  Crystal(std::vector<math::Vec3f> vertexes,     // vertex points
          std::vector<math::TriangleIdx> faces,  // face indices
          CrystalType type);                     // crystal type

  /*! @brief Constructor, given vertexes, faces and face_number_map
   *
   * @param vertexes
   * @param faces
   * @param face_number_map the normalized face ID, or face number.
   *        see [Face numbers](https://www.atoptics.co.uk/halo/fnum.htm)
   *        and [Pyramidal Crystal Face Numbers](https://www.atoptics.co.uk/halo/fnumpyr.htm)
   * @param type
   */
  Crystal(std::vector<math::Vec3f> vertexes,           // vertex points
          std::vector<math::TriangleIdx> faces,        // face indices
          std::vector<ShortIdType> face_number_table,  // face to face number
          CrystalType type);                           // crystal type
};


enum Symmetry : uint8_t {
  kSymmetryNone = 0u,
  kSymmetryPrism = 1u,
  kSymmetryBasal = 2u,
  kSymmetryDirection = 4u,
  kSymmetryRepeatedReflection = 8u,
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
