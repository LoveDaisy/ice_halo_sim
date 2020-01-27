#ifndef SRC_CRYSTAL_H_
#define SRC_CRYSTAL_H_

#include <memory>
#include <utility>
#include <vector>

#include "mymath.h"

namespace icehalo {

enum class CrystalType {
  kUnknown,
  kPrism,
  kPyramid,
  kStackPyramid,
  kCubicPyramid,
  kCustom,
};

class Crystal {
 public:
  ~Crystal();

  CrystalType GetType() const;

  int TotalVertexes() const;
  int TotalFaces() const;
  int FaceNumber(int idx) const;

  const std::vector<math::Vec3f>& GetVertexes();
  const std::vector<math::TriangleIdx>& GetFaces();
  const std::vector<int>& GetFaceNumberMap();

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
  static std::unique_ptr<Crystal> CreateHexPrism(float h);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param h1 height of top segment. The diameter of middle segment is 1
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static std::unique_ptr<Crystal> CreateHexPyramid(float h1, float h2, float h3);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param i1 Miller index 1. The shape of the pyramid segment is defined by Miller index (a,0,-a,b)
   * @param i4 Miller index 4
   * @param h1 height of top segment.
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static std::unique_ptr<Crystal> CreateHexPyramid(int i1, int i4,                 // Miller index
                                                   float h1, float h2, float h3);  // heights

  /*! @brief create a hexagon pyramid crystal
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
  static std::unique_ptr<Crystal> CreateHexPyramid(int upper_idx1, int upper_idx4,  // upper Miller index
                                                   int lower_idx1, int lower_idx4,  // lower Miller index
                                                   float h1, float h2, float h3);   // heights

  /*! @brief Create a hexagon half-stacked pyramid crystal
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
  static std::unique_ptr<Crystal> CreateHexPyramidStackHalf(int upper_idx1, int upper_idx4,  // upper Miller index
                                                            int lower_idx1, int lower_idx4,  // lower Miller index
                                                            float h1, float h2, float h3);   // heights

  /* Cubic pyramid (crystal type of Ic) */
  static std::unique_ptr<Crystal> CreateCubicPyramid(float h1, float h2);

  /*! @brief Create an irregular hexagon prism crystal
   *
   * @param dist defines the distances from origin to each face
   * @param h defines the height / diameter
   * @return
   */
  static std::unique_ptr<Crystal> CreateIrregularHexPrism(const float* dist, float h);

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
  static std::unique_ptr<Crystal> CreateIrregularHexPyramid(const float* dist, const int* idx, const float* h);

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes of the crystal
   * @param faces the faces of the crystal
   * @return
   */
  static std::unique_ptr<Crystal> CreateCustomCrystal(const std::vector<math::Vec3f>& pts,           // vertex points
                                                      const std::vector<math::TriangleIdx>& faces);  // face indices

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes
   * @param faces the faces
   * @param faceIdMap the face number map
   * @return
   */
  static std::unique_ptr<Crystal> CreateCustomCrystal(const std::vector<math::Vec3f>& pts,          // vertex points
                                                      const std::vector<math::TriangleIdx>& faces,  // face indices
                                                      const std::vector<int>& face_number_map);  // face to face number

 protected:
  void InitNorm();
  void InitFaceNumber();
  void InitFaceNumberHex();
  void InitFaceNumberCubic();
  void InitFaceNumberStack();

  static const std::vector<std::pair<math::Vec3f, int>>& GetHexFaceNormToNumberList();
  static const std::vector<std::pair<math::Vec3f, int>>& GetCubicFaceNormToNumberList();

  std::vector<math::Vec3f> vertexes_;
  std::vector<math::TriangleIdx> faces_;
  std::vector<int> face_number_map_;
  CrystalType type_;
  int face_number_period_;

  float* face_bases_;
  float* face_vertexes_;
  float* face_norm_;
  float* face_area_;

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
  Crystal(std::vector<math::Vec3f> vertexes,     // vertex points
          std::vector<math::TriangleIdx> faces,  // face indices
          std::vector<int> face_number_map,      // face to face number
          CrystalType type);                     // crystal type
};

using CrystalPtrU = std::unique_ptr<Crystal>;

}  // namespace icehalo


#endif  // SRC_CRYSTAL_H_
