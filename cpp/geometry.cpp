#include "geometry.h"
#include "linearalgebra.h"

#include <cstring>

template <typename T>
Vec3<T>::Vec3(T x, T y, T z)
{
    _val[0] = x;
    _val[1] = y;
    _val[2] = z;
}

template <typename T>
Vec3<T>::Vec3(T *data)
{
    _val[0] = data[0];
    _val[1] = data[1];
    _val[2] = data[2];
}

template <typename T>
Vec3<T>::Vec3(const Vec3<T> &v)
{
    _val[0] = v._val[0];
    _val[1] = v._val[1];
    _val[2] = v._val[2];
}


template <typename T>
const T* Vec3<T>::val() const
{
    return _val;
}

template <typename T>
void Vec3<T>::val(T x, T y, T z)
{
    _val[0] = x;
    _val[1] = y;
    _val[2] = z;
}

template <typename T>
void Vec3<T>::val(T *data)
{
    _val[0] = data[0];
    _val[1] = data[1];
    _val[2] = data[2];
}

template <typename T>
T Vec3<T>::x() const
{
    return _val[0];
}

template <typename T>
T Vec3<T>::y() const
{
    return _val[1];
}

template <typename T>
T Vec3<T>::z() const
{
    return _val[2];
}

template <typename T>
void Vec3<T>::x(T x)
{
    _val[0] = x;
}

template <typename T>
void Vec3<T>::y(T y)
{
    _val[1] = y;
}

template <typename T>
void Vec3<T>::z(T z)
{
    _val[2] = z;
}

template <typename T>
Vec3<T> Vec3<T>::normalized()
{
    return Vec3<T>::normalized(*this);
}

template <typename T>
void Vec3<T>::normalize()
{
    LinearAlgebra::normalize3(_val);
}

template <typename T>
Vec3<T> Vec3<T>::normalized(const Vec3<T> &v)
{
    T data[3];
    LinearAlgebra::normalized3(v._val, data);
    return Vec3<T>(data);
}

template <typename T>
T Vec3<T>::dot(const Vec3<T> &v1, const Vec3<T> &v2)
{
    return LinearAlgebra::dot3(v1._val, v2._val);
}

template <typename T>
T Vec3<T>::norm(const Vec3<T> &v)
{
    return LinearAlgebra::norm3(v._val);
}

template <typename T>
Vec3<T> Vec3<T>::cross(const Vec3<T> &v1, const Vec3<T> &v2)
{
    T data[3];
    LinearAlgebra::cross3(v1._val, v2._val, data);
    return Vec3<T>(data);
}

template <typename T>
Vec3<T> Vec3<T>::fromVec(const Vec3<T> &v1, const Vec3<T> &v2)
{
    T data[3];
    LinearAlgebra::vec3FromTo(v1._val, v2._val, data);
    return Vec3<T>(data);
}

template class Vec3<float>;


TriangleIdx::TriangleIdx(int id1, int id2, int id3)
{
    _idx[0] = id1;
    _idx[1] = id2;
    _idx[2] = id3;
}

int TriangleIdx::id1() const
{
    return _idx[0];
}

int TriangleIdx::id2() const
{
    return _idx[1];
}

int TriangleIdx::id3() const
{
    return _idx[2];
}

const int * TriangleIdx::idx() const
{
    return &_idx[0];
}


Geometry::Geometry(std::vector<Vec3f> &vertexes, std::vector<TriangleIdx> &faces, float n) :
    vertexes(vertexes), faces(faces), initDone(false), n(n)
{
    initialize();
}

Geometry::Geometry(std::vector<Vec3f> &vertexes, std::vector<TriangleIdx> &faces) :
    Geometry(vertexes, faces, 1.31f)
{}


void Geometry::setVertexes(std::vector<Vec3f> &vertexes)
{
    initDone = false;
    this->vertexes = vertexes;
}

void Geometry::setFaces(std::vector<TriangleIdx> &faces)
{
    initDone = false;
    this->faces = faces;
}

void Geometry::initialize()
{
    norms.clear();
    initDone = true;
    
    size_t vtxNum = vertexes.size();
    for (const auto &f : faces)
    {
        const int * idx = f.idx();
        for (int i = 0; i < 3; ++i)
        {
            if (idx[i] < 0 || idx[i] > vtxNum-1)
            {
                initDone = false;
                return;
            }
        }
        Vec3f v1 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[1]]);
        Vec3f v2 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[2]]);
        norms.push_back(Vec3f::cross(v1, v2));
    }
    for (auto &v : norms) 
    {
        v.normalize();
    }
}

const std::vector<Vec3f>& Geometry::getVertexes()
{
    return vertexes;
}

const std::vector<Vec3f>& Geometry::getNorms()
{
    return norms;
}

const std::vector<TriangleIdx>& Geometry::getFaces()
{
    return faces;
}

bool Geometry::isInitialized()
{
    return initDone;
}

int Geometry::vtxNum() const
{
    return static_cast<int>(vertexes.size());
}

int Geometry::faceNum() const
{
    return static_cast<int>(faces.size());
}

float Geometry::refractiveIndex() const
{
    return n;
}

void Geometry::copyVertexData(float *data) const
{
    for (auto i = 0; i < vertexes.size(); ++i)
    {
        memcpy(data + i*3, vertexes[i].val(), 3*sizeof(float));
    }
}

void Geometry::copyFaceData(float *data) const
{
    for (auto i = 0; i < faces.size(); i++)
    {
        const int *idx = faces[i].idx();
        memcpy(data + i*9+0, vertexes[idx[0]].val(), 3*sizeof(float));
        memcpy(data + i*9+3, vertexes[idx[1]].val(), 3*sizeof(float));
        memcpy(data + i*9+6, vertexes[idx[2]].val(), 3*sizeof(float));
    }
}

void Geometry::copyFaceIdxData(int *data) const
{
    for (auto i = 0; i < faces.size(); i++)
    {
        memcpy(data + i*3, faces[i].idx(), 3*sizeof(int));
    }
}

void Geometry::copyNormalData(int num, const int *idx, float *data) const
{
    for (auto i = 0; i < num; ++i)
    {
        if (idx[i] >= faces.size() || idx[i] < 0)
        {
            break;
        }
        memcpy(data + i*3, norms[idx[i]].val(), 3*sizeof(float));
    }
}

Geometry* Geometry::createHexCylindar(float hRatio)
{
    return Geometry::createHexCylindar(hRatio, 1.31f);
}

Geometry* Geometry::createHexCylindar(float hRatio, float n)
{
    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;

    for (int i = 0; i < 6; ++i)
    {
        vertexes.emplace_back(Vec3f(std::cos(2*PI*i/6), std::sin(2*PI*i/6), hRatio/2));
    }
    for (int i = 0; i < 6; ++i)
    {
        vertexes.emplace_back(Vec3f(std::cos(2*PI*i/6), std::sin(2*PI*i/6), -hRatio/2));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i)
    {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    faces.emplace_back(TriangleIdx(6, 8, 7));
    faces.emplace_back(TriangleIdx(6, 9, 8));
    faces.emplace_back(TriangleIdx(9, 11, 10));
    faces.emplace_back(TriangleIdx(9, 6, 11));

    return new Geometry(vertexes, faces, n);
}
