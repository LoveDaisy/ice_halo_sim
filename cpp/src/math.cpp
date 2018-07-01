#include "context.h"
#include "math.h"


namespace IceHalo {

namespace Math {


float dot3(const float *vec1, const float *vec2)
{
    return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
}

void cross3(const float *vec1, const float *vec2, float* vec)
{
    vec[0] = -vec2[1]*vec1[2] + vec1[1]*vec2[2];
    vec[1] = vec2[0]*vec1[2] - vec1[0]*vec2[2];
    vec[2] = -vec2[0]*vec1[1] + vec1[0]*vec2[1];
}

float norm3(const float *vec)
{
    return std::sqrt(dot3(vec, vec));
}

float diffNorm3(const float *vec1, const float *vec2)
{
    float v[3];
    vec3FromTo(vec1, vec2, v);
    return norm3(v);
}

void normalize3(float *vec)
{
    float len = norm3(vec);
    vec[0] /= len;
    vec[1] /= len;
    vec[2] /= len;
}

void normalized3(const float *vec, float *vec_n)
{
    float len = norm3(vec);
    vec_n[0] = vec[0] / len;
    vec_n[1] = vec[1] / len;
    vec_n[2] = vec[2] / len;
}

void vec3FromTo(const float *vec1, const float *vec2, float *vec)
{
    vec[0] = vec2[0] - vec1[0];
    vec[1] = vec2[1] - vec1[1];
    vec[2] = vec2[2] - vec1[2];
}


void rotateBase(const float *ax, float angle, float *vec)
{
    float c = std::cos(angle);
    float s = std::sin(angle);

    float matR[9] = {c + ax[0] * ax[0] * (1-c), ax[1] * ax[0] * (1-c) + ax[2] * s, ax[2] * ax[0] * (1-c) - ax[1] * s,
        ax[0] * ax[1] * (1-c) - ax[2] * s, c + ax[1] * ax[1] * (1-c), ax[2] * ax[1] * (1-c) + ax[0] * s,
        ax[0] * ax[2] * (1-c) + ax[1] * s, ax[1] * ax[2] * (1-c) - ax[0] * s, c + ax[2] * ax[2] * (1-c)};
    float res[9];

    DummyMatrix v(vec, 3, 3);
    DummyMatrix R(matR, 3, 3);
    DummyMatrix vn(res, 3, 3);
    DummyMatrix::multiply(v, R, vn);

    memcpy(vec, res, 9*sizeof(float));
}


void rotateZ(const float *lon_lat_roll, float *vec, int dataNum)
{
    using namespace std;
    float ax[9] = {-sin(lon_lat_roll[0]), cos(lon_lat_roll[0]), 0.0f,
                   -cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]), -sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]), cos(lon_lat_roll[1]),
                   cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
    // float ax[9] = {sin(lon_lat_roll[0]), -cos(lon_lat_roll[0]), 0.0f,
    //                cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]), sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]), -cos(lon_lat_roll[1]),
    //                cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
    float d[3] = {cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
    rotateBase(d, lon_lat_roll[2], ax);

    DummyMatrix matR(ax, 3, 3);
    matR.transpose();

    auto *res = new float[dataNum * 3];
    
    DummyMatrix resVec(res, dataNum, 3);
    DummyMatrix inputVec(vec, dataNum, 3);
    DummyMatrix::multiply(inputVec, matR, resVec);
    memcpy(vec, res, 3 * dataNum * sizeof(float));

    delete[] res;
}


void rotateZBack(const float *lon_lat_roll, float *vec, int dataNum)
{
    using namespace std;
    float ax[9] = {-sin(lon_lat_roll[0]), cos(lon_lat_roll[0]), 0.0f,
                   -cos(lon_lat_roll[0]) * sin(lon_lat_roll[1]), -sin(lon_lat_roll[0]) * sin(lon_lat_roll[1]), cos(lon_lat_roll[1]),
                   cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
    float d[3] = {cos(lon_lat_roll[1]) * cos(lon_lat_roll[0]), cos(lon_lat_roll[1]) * sin(lon_lat_roll[0]), sin(lon_lat_roll[1])};
    rotateBase(d, lon_lat_roll[2], ax);

    DummyMatrix matR(ax, 3, 3);

    // float res[3] = { 0.0f };
    auto *res = new float[dataNum * 3];
    
    DummyMatrix resVec(res, dataNum, 3);
    DummyMatrix inputVec(vec, dataNum, 3);
    DummyMatrix::multiply(inputVec, matR, resVec);
    memcpy(vec, res, 3 * dataNum * sizeof(float));

    delete[] res;
}


DummyMatrix::DummyMatrix(float *data, int row, int col) :
    rowNum(row), colNum(col), data(data)
{ }

int DummyMatrix::multiply(const DummyMatrix &a, const DummyMatrix &b, DummyMatrix &res)
{
    if (a.colNum != b.rowNum)
    {
        return -1;
    }

    for (int r = 0; r < a.rowNum; r++)
    {
        for (int c = 0; c < b.colNum; c++)
        {
            float sum = 0.0f;
            for (int k = 0; k < a.colNum; k++)
            {
                sum += a.data[r*a.colNum + k] * b.data[k*b.colNum + c];
            }
            res.data[r*res.colNum + c] = sum;
        }
    }
    return 0;
}

void DummyMatrix::transpose()
{
    for (int r = 0; r < rowNum; r++)
    {
        for (int c = r+1; c < colNum; c++)
        {
            float tmp;
            tmp = data[r*colNum + c];
            data[r*colNum + c] = data[c*colNum +r];
            data[c*colNum +r] = tmp;
        }
    }
}




template <typename T>
Vec3<T>::Vec3(T x, T y, T z)
{
    _val[0] = x;
    _val[1] = y;
    _val[2] = z;
}

template <typename T>
Vec3<T>::Vec3(const T *data)
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
void Vec3<T>::val(const T *data)
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
    Math::normalize3(_val);
}

template <typename T>
Vec3<T> Vec3<T>::normalized(const Vec3<T> &v)
{
    T data[3];
    Math::normalized3(v._val, data);
    return Vec3<T>(data);
}

template <typename T>
T Vec3<T>::dot(const Vec3<T> &v1, const Vec3<T> &v2)
{
    return Math::dot3(v1._val, v2._val);
}

template <typename T>
T Vec3<T>::norm(const Vec3<T> &v)
{
    return Math::norm3(v._val);
}

template <typename T>
Vec3<T> Vec3<T>::cross(const Vec3<T> &v1, const Vec3<T> &v2)
{
    T data[3];
    Math::cross3(v1._val, v2._val, data);
    return Vec3<T>(data);
}

template <typename T>
Vec3<T> Vec3<T>::fromVec(const Vec3<T> &v1, const Vec3<T> &v2)
{
    T data[3];
    Math::vec3FromTo(v1._val, v2._val, data);
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

}  // namespace Math


Crystal::Crystal(std::vector<Math::Vec3f> &vertexes, std::vector<Math::TriangleIdx> &faces) :
    vertexes(vertexes), faces(faces), initDone(false)
{
    initialize();
}


void Crystal::setVertexes(std::vector<Math::Vec3f> &vertexes)
{
    initDone = false;
    this->vertexes = vertexes;
}

void Crystal::setFaces(std::vector<Math::TriangleIdx> &faces)
{
    initDone = false;
    this->faces = faces;
}

void Crystal::initialize()
{
    norms.clear();
    initDone = true;
    
    int vtxNum = static_cast<int>(vertexes.size());
    for (const auto &f : faces) {
        const int * idx = f.idx();
        for (int i = 0; i < 3; ++i) {
            if (idx[i] < 0 || idx[i] > vtxNum-1) {
                initDone = false;
                return;
            }
        }
        Math::Vec3f v1 = Math::Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[1]]);
        Math::Vec3f v2 = Math::Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[2]]);
        norms.push_back(Math::Vec3f::cross(v1, v2));
    }
    for (auto &v : norms) {
        v.normalize();
    }
}

const std::vector<Math::Vec3f>& Crystal::getVertexes()
{
    return vertexes;
}

const std::vector<Math::Vec3f>& Crystal::getNorms()
{
    return norms;
}

const std::vector<Math::TriangleIdx>& Crystal::getFaces()
{
    return faces;
}

bool Crystal::isInitialized()
{
    return initDone;
}

int Crystal::vtxNum() const
{
    return static_cast<int>(vertexes.size());
}

int Crystal::faceNum() const
{
    return static_cast<int>(faces.size());
}

void Crystal::copyVertexData(float *data) const
{
    for (decltype(vertexes.size()) i = 0; i < vertexes.size(); ++i) {
        memcpy(data + i*3, vertexes[i].val(), 3*sizeof(float));
    }
}

void Crystal::copyFaceData(float *data) const
{
    for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
        const int *idx = faces[i].idx();
        memcpy(data + i*9+0, vertexes[idx[0]].val(), 3*sizeof(float));
        memcpy(data + i*9+3, vertexes[idx[1]].val(), 3*sizeof(float));
        memcpy(data + i*9+6, vertexes[idx[2]].val(), 3*sizeof(float));
    }
}

void Crystal::copyFaceIdxData(int *data) const
{
    for (decltype(faces.size()) i = 0; i < faces.size(); i++) {
        memcpy(data + i*3, faces[i].idx(), 3*sizeof(int));
    }
}

void Crystal::copyNormalData(int idx, float *data) const
{
    if (idx >= static_cast<int>(faces.size()) || idx < 0) {
        return;
    }
    memcpy(data, norms[idx].val(), 3*sizeof(float));
}

void Crystal::copyNormalData(float *data) const
{
    for (decltype(norms.size()) i = 0; i < norms.size(); i++) {
        memcpy(data + i*3, norms[i].val(), 3*sizeof(float));
    }
}

/*
 * parameter: h, defined as c / a, i.e. height / diameter
 */
Crystal* Crystal::createHexCylinder(float h)
{
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;

    vertexes.reserve(6);
    for (int i = 0; i < 6; ++i) {
        vertexes.emplace_back(Math::Vec3f(std::cos(2*PI*i/6), std::sin(2*PI*i/6), h));
    }
    for (int i = 0; i < 6; ++i) {
        vertexes.emplace_back(Math::Vec3f(std::cos(2*PI*i/6), std::sin(2*PI*i/6), -h));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    faces.emplace_back(Math::TriangleIdx(3, 4, 5));
    faces.emplace_back(Math::TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    faces.emplace_back(Math::TriangleIdx(6, 8, 7));
    faces.emplace_back(Math::TriangleIdx(6, 9, 8));
    faces.emplace_back(Math::TriangleIdx(9, 11, 10));
    faces.emplace_back(Math::TriangleIdx(9, 6, 11));

    return new Crystal(vertexes, faces);
}

/*
    top vertex:
        2   1
    3           0
        4   5

    upper vertex:
        8   7
    9           6
        10  11

    lower vertex:
        14  13
    15          12
        16  17

    bottom vertex:
        20  19
    21          18
        22  23
*/
/*
 * parameter: h1, defined as h / H, where h is the actual height of first pyramid, H is the
 *            full height of first pyramid. It will be clamped to between 0.0 and 1.0.
 * parameter: h2, defined as h / a, where h is the actual height of middle cylinder, a is the
 *            diameter of base plate.
 * parameter: h3, defined as h / H, similar to h3.
 */
Crystal* Crystal::createHexPyramid(float h1, float h2, float h3)
{
    const float H = 1.629f;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);

    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h1),
            std::sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h3),
            std::sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    faces.emplace_back(Math::TriangleIdx(3, 4, 5));
    faces.emplace_back(Math::TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(Math::TriangleIdx(18, 20, 19));
    faces.emplace_back(Math::TriangleIdx(18, 21, 20));
    faces.emplace_back(Math::TriangleIdx(21, 23, 22));
    faces.emplace_back(Math::TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}


/*
    top vertex:
        1       0
        2       3
    middle vertex:
        5       4
        6       7
    bottom vertex:
        9       8
        10      11
*/
Crystal *Crystal::createCubicPyramid(float ratio1, float ratio2) {
    ratio1 = std::min(ratio1, 1.f);
    ratio2 = std::min(ratio2, 1.f);

    // float q = 31.55f * PI / 180;
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(12);

    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(PI/4 + PI/2*i) * (1 - ratio1), 
            std::sin(PI/4 + PI/2*i) * (1 - ratio1), 
            ratio1));
    }
    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(PI/4 + PI/2*i), 
            std::sin(PI/4 + PI/2*i), 
            0));
    }
    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(PI/4 + PI/2*i) * (1 - ratio2), 
            std::sin(PI/4 + PI/2*i) * (1 - ratio2), 
            -ratio2));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    for (int i = 0; i < 4; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+4, (i+1)%4));
        faces.emplace_back(Math::TriangleIdx(i+4, (i+1)%4+4, (i+1)%4));
    }
    for (int i = 4; i < 8; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+4, (i+1)%4+4));
        faces.emplace_back(Math::TriangleIdx(i+4, (i+1)%4+8, (i+1)%4+4));
    }
    faces.emplace_back(Math::TriangleIdx(9, 8, 10));
    faces.emplace_back(Math::TriangleIdx(10, 8, 11));

    return new Crystal(vertexes, faces);
}


/*
    top vertex:
        2   1
    3           0
        4   5

    upper vertex:
        8   7
    9           6
        10  11

    lower vertex:
        14  13
    15          12
        16  17

    bottom vertex:
        20  19
    21          18
        22  23
*/
/*
 * parameter: i1, i4, the miller index to describe face 13. Index form is (i1, 0, -i1, i4)
 * parameter: h1, defined as h / H
 * parameter: h2, defined as h / a
 * parameter: h3, similar to h1
 */
Crystal* Crystal::createHexPyramid(int i1, int i4, float h1, float h2, float h3)
{
    float H = 1.629f * i1 / i4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);
    
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h1),
            std::sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h3),
            std::sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    faces.emplace_back(Math::TriangleIdx(3, 4, 5));
    faces.emplace_back(Math::TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(Math::TriangleIdx(18, 20, 19));
    faces.emplace_back(Math::TriangleIdx(18, 21, 20));
    faces.emplace_back(Math::TriangleIdx(21, 23, 22));
    faces.emplace_back(Math::TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}


Crystal* Crystal::createHexPyramid(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3)
{
    float H1 = 1.629f * upperIdx1 / upperIdx4;
    float H3 = 1.629f * lowerIdx1 / lowerIdx4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);
    
    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h1),
            std::sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H1));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * (1 - h3),
            std::sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H3));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    faces.emplace_back(Math::TriangleIdx(3, 4, 5));
    faces.emplace_back(Math::TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(Math::TriangleIdx(18, 20, 19));
    faces.emplace_back(Math::TriangleIdx(18, 21, 20));
    faces.emplace_back(Math::TriangleIdx(21, 23, 22));
    faces.emplace_back(Math::TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}


Crystal* Crystal::createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4,
    float h1, float h2, float h3)
{
    float H1 = 1.629f * upperIdx1 / upperIdx4;
    float H2 = 1.629f * lowerIdx1 / lowerIdx4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h2 = std::max(std::min(h2, 1.0f), 0.0f);

    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(24);
    float r = (1.0f - h2) * (1.0f - h1);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * r, 
            std::sin(2*PI*i/6) * r, 
            h1 * H1 * (1.0f - h1) + h2 * H2 + h3 * 2));
    }
    r = 1.0f - h2;
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6) * r, 
            std::sin(2*PI*i/6) * r, 
            h2 + h3 * 2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            h3 * 2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/6), 
            std::sin(2*PI*i/6), 
            0));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    faces.emplace_back(Math::TriangleIdx(0, 2, 3));
    faces.emplace_back(Math::TriangleIdx(3, 4, 5));
    faces.emplace_back(Math::TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(Math::TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(Math::TriangleIdx(18, 20, 19));
    faces.emplace_back(Math::TriangleIdx(18, 21, 20));
    faces.emplace_back(Math::TriangleIdx(21, 23, 22));
    faces.emplace_back(Math::TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}


/*
    top vertex:
    1
           0
    2

    upper vertex:
    4
           3
    5

    lower vertex:
    7
            6
    8

    bottom vertex:
    10
            9
    11
*/
Crystal *Crystal::createTriPyramid(int i1, int i4, float h1, float h2, float h3) {
    float H = 1.629f / 1.732051f * i1 / i4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);

    std::vector<Math::Vec3f> vertexes;
    std::vector<Math::TriangleIdx> faces;
    vertexes.reserve(12);
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/3) * (1 - h1),
            std::sin(2*PI*i/3) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/3), 
            std::sin(2*PI*i/3), 
            h2));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/3), 
            std::sin(2*PI*i/3), 
            -h2));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Math::Vec3f(
            std::cos(2*PI*i/3) * (1 - h3),
            std::sin(2*PI*i/3) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(Math::TriangleIdx(0, 1, 2));
    for (int i = 0; i < 3; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+3, (i+1)%3));
        faces.emplace_back(Math::TriangleIdx(i+3, (i+1)%3+3, (i+1)%3));
    }
    for (int i = 3; i < 6; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+3, (i+1)%3+3));
        faces.emplace_back(Math::TriangleIdx(i+3, (i+1)%3+6, (i+1)%3+3));
    }
    for (int i = 6; i < 9; ++i) {
        faces.emplace_back(Math::TriangleIdx(i, i+3, (i+1)%3+6));
        faces.emplace_back(Math::TriangleIdx(i+3, (i+1)%3+9, (i+1)%3+6));
    }
    faces.emplace_back(Math::TriangleIdx(10, 9, 11));

    return new Crystal(vertexes, faces);
}


namespace Math {

OrientationGenerator::OrientationGenerator() :
    axDist(Distribution::UNIFORM), axMean(0), axStd(0),
    rollDist(Distribution::UNIFORM), rollMean(0), rollStd(0)
{ }

OrientationGenerator::OrientationGenerator(Distribution axDist, float axMean, float axStd,
        Distribution rollDist, float rollMean, float rollStd) :
    axDist(axDist), axMean(axMean), axStd(axStd),
    rollDist(rollDist), rollMean(rollMean), rollStd(rollStd)
{
    unsigned int seed = static_cast<unsigned int>(std::__1::chrono::system_clock::now().time_since_epoch().count());
    // unsigned int seed = 2345;
    generator.seed(seed);
}

void OrientationGenerator::fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot)
{
    // float tmpSunDir[3] = { 0.0f };
    // float sunRotation[3] = {
    //     atan2(-sunDir[1], -sunDir[0]),
    //     asin(-sunDir[2]),
    //     0.0f
    // };
    // float h = 1.0f - cos(0.25f * Crystal::PI / 180.0f);

    // printf("SUN_ROT:%+.4f,%+.4f,%+.4f\n", sunRotation[0], sunRotation[1], sunRotation[2]);

    for (int i = 0; i < num; i++) {
        float lon, lat, roll;

        switch (axDist) {
            case Distribution::UNIFORM : {
                float v[3] = {gaussDistribution(generator),
                              gaussDistribution(generator),
                              gaussDistribution(generator)};
                Math::normalize3(v);
                lon = atan2(v[1], v[0]);
                lat = asin(v[2] / Math::norm3(v));
            }
                break;
            case Distribution::GAUSS :
                lon = uniformDistribution(generator) * 2 * Crystal::PI;
                lat = gaussDistribution(generator) * axStd;
                lat += axMean;
                if (lat > Crystal::PI / 2) {
                    lat = Crystal::PI - lat;
                }
                if (lat < -Crystal::PI / 2) {
                    lat = -Crystal::PI - lat;
                }
                break;
        }

        switch (rollDist) {
            case Distribution::GAUSS :
                roll = gaussDistribution(generator) * rollStd + rollMean;
                break;
            case Distribution::UNIFORM :
                roll = (uniformDistribution(generator) - 0.5f) * rollStd + rollMean;
                break;
        }

        mainAxRot[i*3+0] = lon;
        mainAxRot[i*3+1] = lat;
        mainAxRot[i*3+2] = roll;

        memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
        Math::rotateZ(mainAxRot + i * 3, rayDir + i * 3);
    }
}

}   // namespace Math

}   // namespace IceHalo