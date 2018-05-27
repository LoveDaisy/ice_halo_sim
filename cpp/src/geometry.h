#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <cmath>
#include <set>
#include <random>

template <typename T>
class Vec3
{
public:
    Vec3(T x, T y, T z);

    explicit Vec3(T *data);
    Vec3(const Vec3<T> &v);

    T x() const;
    T y() const;
    T z() const;
    void x(T x);
    void y(T y);
    void z(T z);

    const T* val() const;
    void val(T x, T y, T z);
    void val(T *data);

    Vec3<T> normalized();
    void normalize();

    static Vec3<T> normalized(const Vec3<T> &v);

    static T dot(const Vec3<T> &v1, const Vec3<T> &v2);
    static T norm(const Vec3<T> &v);
    static Vec3<T> cross(const Vec3<T> &v1, const Vec3<T> &v2);

    static Vec3<T> fromVec(const Vec3<T> &v1, const Vec3<T> &v2);

private:
    T _val[3];
};

using Vec3f = Vec3<float>;


class TriangleIdx
{
public:
    TriangleIdx(int id1, int id2, int id3);

    int id1() const;
    int id2() const;
    int id3() const;

    const int * idx() const;

private:
    int _idx[3];
};


class Crystal
{
public:
    Crystal(std::vector<Vec3f> &vertexes, std::vector<TriangleIdx> &faces);

    void setVertexes(std::vector<Vec3f> &vertexes);
    void setFaces(std::vector<TriangleIdx> &faces);
    void initialize();
    bool isInitialized();

    int vtxNum() const;
    int faceNum() const;

    const std::vector<Vec3f>& getVertexes();
    const std::vector<Vec3f>& getNorms();
    const std::vector<TriangleIdx>& getFaces();

    void copyVertexData(float *data) const;
    void copyFaceData(float *data) const;
    void copyFaceIdxData(int *data) const;
    void copyNormalData(int num, const int *idx, float *data) const;

    static Crystal* createHexCylinder(float h);
    static Crystal* createHexPyramid(float h1, float h2, float h3);
    static Crystal* createHexPyramid(int i1, int i4, float h1, float h2, float h3);
    static Crystal* createHexPyramid(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);
    static Crystal* createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3);
    static Crystal* createTriPyramid(int i1, int i4, float h1, float h2, float h3);
    static Crystal* createCubicPyramid(float ratio1, float ratio2);

    static constexpr float PI = 3.14159265f;

private:
    std::vector<Vec3f> vertexes;
    std::vector<Vec3f> norms;
    std::vector<TriangleIdx> faces;

    bool initDone;
};

class OrientationGenerator
{
public:
    enum class Distribution
    {
        UNIFORM,
        GAUSS
    };

public:
    OrientationGenerator(Distribution axDist, float axMean, float axStd,
        Distribution rollDist, float rollMean, float rollStd);
    ~OrientationGenerator() = default;

    void fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot);

private:
    std::mt19937 generator;
    std::normal_distribution<float> gaussDistribution;
    std::uniform_real_distribution<float> uniformDistribution;

    Distribution axDist;
    float axMean;
    float axStd;

    Distribution rollDist;
    float rollMean;
    float rollStd;
};

#endif // GEOMETRY_H
