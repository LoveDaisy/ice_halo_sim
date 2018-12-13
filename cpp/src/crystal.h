#ifndef SRC_CRYSTAL_H_
#define SRC_CRYSTAL_H_

#include "mymath.h"

#include <vector>

namespace IceHalo {

class Crystal {
public:
  Crystal(const std::vector<Math::Vec3f>& vertexes, const std::vector<Math::TriangleIdx>& faces);

  void setVertexes(const std::vector<Math::Vec3f>& vertexes);
  void setFaces(const std::vector<Math::TriangleIdx>& faces);
  void initialize();
  bool isInitialized();

  int vtxNum() const;
  int faceNum() const;

  const std::vector<Math::Vec3f>& getVertexes();
  const std::vector<Math::Vec3f>& getNorms();
  const std::vector<Math::TriangleIdx>& getFaces();

  void copyVertexData(float* data) const;
  void copyFaceData(float* data) const;
  void copyFaceIdxData(int* data) const;
  void copyNormalData(int idx, float* data) const;
  void copyNormalData(float* data) const;

  static constexpr float C_CONSTANT = 1.629f;

  /* Regular hexagon cylinder */
  static Crystal* createHexCylinder(float h);

  /* Regular hexagon pyramid */
  static Crystal* createHexPyramid(float h1, float h2, float h3);
  static Crystal* createHexPyramid(int i1, int i4, float h1, float h2, float h3);
  static Crystal* createHexPyramid(int upperIdx1, int upperIdx4,
                                   int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);

  /* Hexagon stacked pyramid */
  static Crystal* createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4,
                                            float h1, float h2, float h3);

  /* Triangle pyramid */
  static Crystal* createTriPyramid(int i1, int i4, float h1, float h2, float h3);

  /* Cubic pyramid (crystal type of Ic) */
  static Crystal* createCubicPyramid(float ratio1, float ratio2);

  /* Irregular hexagon cylinder */
  static Crystal* createIrregularHexCylinder(float* dist, float h);

  /* Irregular hexagon pyramid */
  static Crystal* createIrregularHexPyramid(float* dist, int* idx, float* h);

private:
  std::vector<Math::Vec3f> vertexes;
  std::vector<Math::Vec3f> norms;
  std::vector<Math::TriangleIdx> faces;

  bool initDone;
};

};  // namespace IceHalo


#endif  // SRC_CRYSTAL_H
