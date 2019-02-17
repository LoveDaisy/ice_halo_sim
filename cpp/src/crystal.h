#ifndef SRC_CRYSTAL_H_
#define SRC_CRYSTAL_H_

#include "mymath.h"

#include <vector>
#include <utility>
#include <memory>

namespace IceHalo {

enum class CrystalType {
  UNKNOWN,
  PRISM,
  PYRAMID,
  STACK_PYRAMID,
  CUBIC_PYRAMID,
  CUSTOM,
};

class Crystal {
public:
  int totalVertexes() const;
  int totalFaces() const;
  int faceNumber(int idx) const;

  const std::vector<Math::Vec3f>& getVertexes();
  const std::vector<Math::Vec3f>& getNorms();
  const std::vector<Math::TriangleIdx>& getFaces();
  const std::vector<int>& getFaceNumber();

  void copyFaceData(float* data) const;
  void copyFaceAreaData(float* data) const;

  void copyNormData(float *data) const;

  static constexpr float kC = 1.629f;

  /*! @brief Create a regular hexagon prism crystal
   *
   * @param h the height of prism. The diameter of basal face is 1
   * @return a pointer to the crystal
   */
  static std::unique_ptr<Crystal> createHexPrism(float h);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param h1 height of top segment. The diameter of middle segment is 1
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static std::unique_ptr<Crystal> createHexPyramid(float h1, float h2, float h3);

  /*! @brief create a hexagon pyramid crystal
   *
   * @param i1 Miller index 1. The shape of the pyramid segment is defined by Miller index (a,0,-a,b)
   * @param i4 Miller index 4
   * @param h1 height of top segment.
   * @param h2 height of middle segment.
   * @param h3 height of bottom segment.
   * @return a pointer to the crystal.
   */
  static std::unique_ptr<Crystal> createHexPyramid(int i1, int i4, float h1, float h2, float h3);

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
  static std::unique_ptr<Crystal> createHexPyramid(
    int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);

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
  static std::unique_ptr<Crystal> createHexPyramidStackHalf(
    int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);

  /* Cubic pyramid (crystal type of Ic) */
  static std::unique_ptr<Crystal> createCubicPyramid(float ratio1, float ratio2);

  /*! @brief Create an irregular hexagon prism crystal
   *
   * @param dist defines the distances from origin to each face
   * @param h defines the height / diameter
   * @return
   */
  static std::unique_ptr<Crystal> createIrregularHexPrism(float *dist, float h);

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
  static std::unique_ptr<Crystal> createIrregularHexPyramid(float* dist, int* idx, float* h);

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes of the crystal
   * @param faces the faces of the crystal
   * @return
   */
  static std::unique_ptr<Crystal> createCustomCrystal(const std::vector<Math::Vec3f>& pts,
                                                      const std::vector<Math::TriangleIdx>& faces);

  /*! @brief Create a customized crystal
   *
   * @param pts the vertexes
   * @param faces the faces
   * @param faceIdMap the face number map
   * @return
   */
  static std::unique_ptr<Crystal> createCustomCrystal(const std::vector<Math::Vec3f>& pts,
                                                      const std::vector<Math::TriangleIdx>& faces,
                                                      const std::vector<int>& faceIdMap);

protected:
  void initNorms();
  void initFaceNumbers();
  void initFaceNumbersHex();
  void initFaceNumberCubic();
  void initFaceNumberStack();

  static const std::vector<std::pair<Math::Vec3f, int> > hexFaceNormToNumberList;
  static const std::vector<std::pair<Math::Vec3f, int> > cubicFaceNormToNumberList;

  std::vector<Math::Vec3f> vertexes;
  std::vector<Math::Vec3f> norms;
  std::vector<Math::TriangleIdx> faces;
  std::vector<int> faceIdMap;
  CrystalType type;

private:
  /*! @brief Constructor, given vertexes and faces
   *
   * @param vertexes
   * @param faces
   */
  Crystal(const std::vector<Math::Vec3f>& vertexes, const std::vector<Math::TriangleIdx>& faces, CrystalType type);

  /*! @brief Constructor, given vertexes, faces and faceId
   *
   * @param vertexes
   * @param faces
   * @param faceId the normalized face ID, or face number. see [Face numbers](https://www.atoptics.co.uk/halo/fnum.htm)
   *        and [Pyramidal Crystal Face Numbers](https://www.atoptics.co.uk/halo/fnumpyr.htm)
   */
  Crystal(const std::vector<Math::Vec3f>& vertexes, const std::vector<Math::TriangleIdx>& faces,
          const std::vector<int>& faceId, CrystalType type);

};

using CrystalPtrU = std::unique_ptr<Crystal>;
using CrystalPtr = std::shared_ptr<Crystal>;

};  // namespace IceHalo


#endif  // SRC_CRYSTAL_H_
