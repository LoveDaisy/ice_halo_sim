#ifndef ICEHALOSIM_CRYSTAL_H
#define ICEHALOSIM_CRYSTAL_H

#include "mymath.h"

#include <vector>

namespace IceHalo {

class Crystal
{
public:
    Crystal(std::vector<Math::Vec3f> &vertexes, std::vector<Math::TriangleIdx> &faces);

    void setVertexes(std::vector<Math::Vec3f> &vertexes);
    void setFaces(std::vector<Math::TriangleIdx> &faces);
    void initialize();
    bool isInitialized();

    int vtxNum() const;
    int faceNum() const;

    const std::vector<Math::Vec3f>& getVertexes();
    const std::vector<Math::Vec3f>& getNorms();
    const std::vector<Math::TriangleIdx>& getFaces();

    void copyVertexData(float *data) const;
    void copyFaceData(float *data) const;
    void copyFaceIdxData(int *data) const;
    void copyNormalData(int idx, float *data) const;
    void copyNormalData(float *data) const;

    static Crystal* createHexCylinder(float h);
    static Crystal* createHexPyramid(float h1, float h2, float h3);
    static Crystal* createHexPyramid(int i1, int i4, float h1, float h2, float h3);
    static Crystal* createHexPyramid(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);
    static Crystal* createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);
    static Crystal* createTriPyramid(int i1, int i4, float h1, float h2, float h3);
    static Crystal* createCubicPyramid(float ratio1, float ratio2);

private:
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::Vec3f> norms;
    std::vector<Math::TriangleIdx> faces;

    bool initDone;
};

};


#endif //ICEHALOSIM_CRYSTAL_H
