#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <cmath>
#include <set>
#include <random>


namespace IceHalo {

namespace Math {

constexpr float PI = 3.14159265f;

float dot3(const float *vec1, const float *vec2);
void cross3(const float *vec1, const float *vec2, float* vec);
float norm3(const float *vec);
float diffNorm3(const float *vec1, const float *vec2);
void normalize3(float *vec);
void normalized3(const float *vec, float *vec_n);
void vec3FromTo(const float *vec1, const float *vec2, float *vec);

void rotateBase(const float *ax, float angle, float *vec);

void rotateZ(const float *lon_lat_roll, float *vec, int dataNum = 1);
void rotateZBack(const float *lon_lat_roll, float *vec, int dataNum = 1);


class DummyMatrix
{
public:
    DummyMatrix(float *data, int row, int col);
    ~DummyMatrix() = default;

    const int rowNum;
    const int colNum;

    void transpose();

    static int multiply(const DummyMatrix &a, const DummyMatrix &b, DummyMatrix &c);

private:
    float *data;
};


template <typename T>
class Vec3
{
public:
    explicit Vec3(const T *data);
    Vec3(T x, T y, T z);
    Vec3(const Vec3<T> &v);

    T x() const;
    T y() const;
    T z() const;
    void x(T x);
    void y(T y);
    void z(T z);

    const T* val() const;
    void val(T x, T y, T z);
    void val(const T *data);

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


enum class Distribution
{
    UNIFORM,
    GAUSS
};


class OrientationGenerator
{
public:
    OrientationGenerator();
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

};  // namespace Math



}   // namespace IceHalo

#endif // GEOMETRY_H
