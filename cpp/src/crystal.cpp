#include "crystal.h"

#include <cstring>
#include <algorithm>

namespace IceHalo {

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
    using namespace Math;

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
        Vec3f v1 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[1]]);
        Vec3f v2 = Vec3f::fromVec(vertexes[idx[0]], vertexes[idx[2]]);
        norms.push_back(Vec3f::cross(v1, v2));
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
    using namespace Math;

    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;

    vertexes.reserve(6);
    for (int i = 0; i < 6; ++i) {
        vertexes.emplace_back(Vec3f(cos(2 * PI * i / 6), sin(2 * PI * i / 6), h));
    }
    for (int i = 0; i < 6; ++i) {
        vertexes.emplace_back(Vec3f(cos(2 * PI * i / 6), sin(2 * PI * i / 6), -h));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    faces.emplace_back(TriangleIdx(6, 8, 7));
    faces.emplace_back(TriangleIdx(6, 9, 8));
    faces.emplace_back(TriangleIdx(9, 11, 10));
    faces.emplace_back(TriangleIdx(9, 6, 11));

    return new Crystal(vertexes, faces);
}

/*
 * parameter: h1, defined as h / H, where h is the actual height of first pyramid, H is the
 *            full height of first pyramid. It will be clamped to between 0.0 and 1.0.
 * parameter: h2, defined as h / a, where h is the actual height of middle cylinder, a is the
 *            diameter of base plate.
 * parameter: h3, defined as h / H, similar to h3.
 */
Crystal* Crystal::createHexPyramid(float h1, float h2, float h3)
{
    using namespace Math;

    float H = C_CONSTANT;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);

    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h1),
            sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h3),
            sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(TriangleIdx(18, 20, 19));
    faces.emplace_back(TriangleIdx(18, 21, 20));
    faces.emplace_back(TriangleIdx(21, 23, 22));
    faces.emplace_back(TriangleIdx(21, 18, 23));

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
Crystal *Crystal::createCubicPyramid(float ratio1, float ratio2)
{
    using namespace Math;

    ratio1 = std::min(ratio1, 1.f);
    ratio2 = std::min(ratio2, 1.f);

    // float q = 31.55f * PI / 180;
    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(12);

    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Vec3f(
            cos(PI/4 + PI/2*i) * (1 - ratio1), 
            sin(PI/4 + PI/2*i) * (1 - ratio1), 
            ratio1));
    }
    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Vec3f(
            cos(PI/4 + PI/2*i), 
            sin(PI/4 + PI/2*i), 
            0));
    }
    for (int i = 0; i < 4; i++) {
        vertexes.emplace_back(Vec3f(
            cos(PI/4 + PI/2*i) * (1 - ratio2), 
            sin(PI/4 + PI/2*i) * (1 - ratio2), 
            -ratio2));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    for (int i = 0; i < 4; ++i) {
        faces.emplace_back(TriangleIdx(i, i+4, (i+1)%4));
        faces.emplace_back(TriangleIdx(i+4, (i+1)%4+4, (i+1)%4));
    }
    for (int i = 4; i < 8; ++i) {
        faces.emplace_back(TriangleIdx(i, i+4, (i+1)%4+4));
        faces.emplace_back(TriangleIdx(i+4, (i+1)%4+8, (i+1)%4+4));
    }
    faces.emplace_back(TriangleIdx(9, 8, 10));
    faces.emplace_back(TriangleIdx(10, 8, 11));

    return new Crystal(vertexes, faces);
}

/*
 * parameter: i1, i4, the miller index to describe face 13. Index form is (i1, 0, -i1, i4)
 * parameter: h1, defined as h / H
 * parameter: h2, defined as h / a
 * parameter: h3, similar to h1
 */
Crystal* Crystal::createHexPyramid(int i1, int i4, float h1, float h2, float h3)
{
    using namespace Math;

    float H = C_CONSTANT * i1 / i4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);
    
    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h1),
            sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h3),
            sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(TriangleIdx(18, 20, 19));
    faces.emplace_back(TriangleIdx(18, 21, 20));
    faces.emplace_back(TriangleIdx(21, 23, 22));
    faces.emplace_back(TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}

Crystal* Crystal::createHexPyramid(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4, float h1, float h2, float h3)
{
    using namespace Math;

    float H1 = C_CONSTANT * upperIdx1 / upperIdx4;
    float H3 = C_CONSTANT * lowerIdx1 / lowerIdx4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);
    
    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(24);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h1),
            sin(2*PI*i/6) * (1 - h1),
            h2 + h1 * H1));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            -h2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * (1 - h3),
            sin(2*PI*i/6) * (1 - h3),
            -h2 - h3 * H3));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(TriangleIdx(18, 20, 19));
    faces.emplace_back(TriangleIdx(18, 21, 20));
    faces.emplace_back(TriangleIdx(21, 23, 22));
    faces.emplace_back(TriangleIdx(21, 18, 23));

    return new Crystal(vertexes, faces);
}

Crystal* Crystal::createHexPyramidStackHalf(int upperIdx1, int upperIdx4, int lowerIdx1, int lowerIdx4,
                                            float h1, float h2, float h3)
{
    using namespace Math;

    float H1 = C_CONSTANT * upperIdx1 / upperIdx4;
    float H2 = C_CONSTANT * lowerIdx1 / lowerIdx4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h2 = std::max(std::min(h2, 1.0f), 0.0f);

    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(24);
    float r = (1.0f - h2) * (1.0f - h1);
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * r, 
            sin(2*PI*i/6) * r, 
            h1 * H1 * (1.0f - h1) + h2 * H2 + h3 * 2));
    }
    r = 1.0f - h2;
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6) * r, 
            sin(2*PI*i/6) * r, 
            h2 + h3 * 2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            h3 * 2));
    }
    for (int i = 0; i < 6; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/6), 
            sin(2*PI*i/6), 
            0));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    faces.emplace_back(TriangleIdx(0, 2, 3));
    faces.emplace_back(TriangleIdx(3, 4, 5));
    faces.emplace_back(TriangleIdx(3, 5, 0));
    for (int i = 0; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+6, (i+1)%6));
    }
    for (int i = 6; i < 12; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+6));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+12, (i+1)%6+6));
    }
    for (int i = 12; i < 18; ++i) {
        faces.emplace_back(TriangleIdx(i, i+6, (i+1)%6+12));
        faces.emplace_back(TriangleIdx(i+6, (i+1)%6+18, (i+1)%6+12));
    }
    faces.emplace_back(TriangleIdx(18, 20, 19));
    faces.emplace_back(TriangleIdx(18, 21, 20));
    faces.emplace_back(TriangleIdx(21, 23, 22));
    faces.emplace_back(TriangleIdx(21, 18, 23));

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
Crystal* Crystal::createTriPyramid(int i1, int i4, float h1, float h2, float h3) 
{
    using namespace Math;

    float H = C_CONSTANT / 1.732051f * i1 / i4;
    h1 = std::max(std::min(h1, 1.0f), 0.0f);
    h3 = std::max(std::min(h3, 1.0f), 0.0f);

    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(12);
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/3) * (1 - h1),
            sin(2*PI*i/3) * (1 - h1),
            h2 + h1 * H));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/3), 
            sin(2*PI*i/3), 
            h2));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/3), 
            sin(2*PI*i/3), 
            -h2));
    }
    for (int i = 0; i < 3; i++) {
        vertexes.emplace_back(Vec3f(
            cos(2*PI*i/3) * (1 - h3),
            sin(2*PI*i/3) * (1 - h3),
            -h2 - h3 * H));
    }

    faces.emplace_back(TriangleIdx(0, 1, 2));
    for (int i = 0; i < 3; ++i) {
        faces.emplace_back(TriangleIdx(i, i+3, (i+1)%3));
        faces.emplace_back(TriangleIdx(i+3, (i+1)%3+3, (i+1)%3));
    }
    for (int i = 3; i < 6; ++i) {
        faces.emplace_back(TriangleIdx(i, i+3, (i+1)%3+3));
        faces.emplace_back(TriangleIdx(i+3, (i+1)%3+6, (i+1)%3+3));
    }
    for (int i = 6; i < 9; ++i) {
        faces.emplace_back(TriangleIdx(i, i+3, (i+1)%3+6));
        faces.emplace_back(TriangleIdx(i+3, (i+1)%3+9, (i+1)%3+6));
    }
    faces.emplace_back(TriangleIdx(10, 9, 11));

    return new Crystal(vertexes, faces);
}


/*
 * parameter: dist, defines the distance from origin of each face. Must contains 6 numbers.
 *            Starts from face 3.
 * parameter: h, cylinder height, h = height / a
 */
Crystal* Crystal::createIrregularHexCylinder(float *dist, float h)
{
    /* Use a naive algorithm to determine the profile of prism face 
     * 1. For each line pair L1 and L2, get their intersection point p12;
     * 2. For all half planes, check if p12 is in the plane;
     *    2.1 If p12 is in all half planes, put it into a set P;
     *    2.2 Else drop this point;
     * 3. Construct a convex hull from point set P.
     */
    using namespace Math;

    for (int i = 0; i < 6; i++) {
        dist[i] *= static_cast<float>(sqrt(3)) / 2;
    }

    /* Half plane is expressed as: a*x + b*y + c <= 0 */
    float a[6] = { static_cast<float>(sqrt(3)), 0, -static_cast<float>(sqrt(3)), 
                   -static_cast<float>(sqrt(3)), 0, static_cast<float>(sqrt(3)) };
    float b[6] = { 1, 1, 1, -1, -1, -1 };
    float c[6] = { -2*dist[0], -dist[1], -2*dist[2], -2*dist[3], -dist[4], -2*dist[5] };

    std::vector<Vec3f> pts;
    findInnerPoints(6, a, b, c, pts);
    sortAndRemoveDuplicatedPts(pts);
    buildConvexHull(pts);

    size_t ptsNum = pts.size();
    std::vector<Vec3f> vertexes;
    std::vector<TriangleIdx> faces;
    vertexes.reserve(ptsNum * 2);
    for (int i = 0; i < ptsNum; i++) {
        vertexes.emplace_back(Vec3f(pts[i].x(), pts[i].y(), h));
    }
    for (int i = 0; i < ptsNum; i++) {
        vertexes.emplace_back(Vec3f(pts[i].x(), pts[i].y(), -h));
    }

    for (int i = 2; i < ptsNum; i++) {
        faces.emplace_back(TriangleIdx(i, i-1, 0));
    }
    for (int i = 0; i < ptsNum; i++) {
        faces.emplace_back(TriangleIdx(i, i + ptsNum, (i + 1) % ptsNum + ptsNum));
        faces.emplace_back(TriangleIdx((i + 1) % ptsNum, i, (i + 1) % ptsNum + ptsNum));
    }
    for (int i = 2; i < ptsNum; i++) {
        faces.emplace_back(TriangleIdx(i + ptsNum, (i - 1 + ptsNum) % ptsNum + ptsNum, ptsNum));
    }

    return new Crystal(vertexes, faces);
}


/* Sort (ascend by y/x) and remove duplicated points */
void Crystal::sortAndRemoveDuplicatedPts(std::vector<Math::Vec3f> &pts)
{
    using namespace Math;

    /* Move the point with largest x to head */
    for (auto iter = pts.begin(), headIter = pts.begin(); iter != pts.end(); iter++) {
        if (iter->x() > headIter->x()) {
            std::iter_swap(iter, headIter);
        }
    }

    /* Remove duplicated head */
    for (auto iter = pts.begin(), headIter = pts.begin(); iter != pts.end(); ) {
        if (iter != headIter && (*iter) == (*headIter)) {
            iter = pts.erase(iter);
        } else {
            iter++;
        }
    }
    
    /* Sort by dy/dx */
    std::sort(pts.begin() + 1, pts.end(), [&pts](const Vec3f &p1, const Vec3f &p2){
        float dx1 = pts.begin()->x() - p1.x();
        float dx2 = pts.begin()->x() - p2.x();
        float dy1 = pts.begin()->y() - p1.y();
        float dy2 = pts.begin()->y() - p2.y();

        if (abs(dx1) > FLOAT_EPS && abs(dx2) > FLOAT_EPS && dy1 / dx1 < dy2 / dx2) {
            return true;
        }
        if (abs(dx1) < FLOAT_EPS) {
            return dy1 < 0;
        }
        if (abs(dx2) < FLOAT_EPS) {
            return dy2 >= 0;
        }
        return false;
    });

    /* Remove duplicated points */
    for (auto iter = pts.begin(), lastIter = pts.begin(); iter != pts.end(); ) {
        if (iter != lastIter && (*iter) == (*lastIter)) {
            iter = pts.erase(iter);
        } else {
            lastIter = iter;
            iter++;
        }
    }
}


/* Given a sorted point set, remove all inner points and those convex hull points are remained */
void Crystal::buildConvexHull(std::vector<Math::Vec3f> &pts)
{
    if (pts.size() <= 3) {
        return;
    }

    using namespace Math;

    for (auto iter1 = pts.begin() + 2; iter1 != pts.end(); iter1++) {
        for (auto iter2 = pts.begin() + 1; iter2 != iter1 - 1; ) {
            auto v1 = Vec3f::fromVec(*(iter2-1), *iter2);
            auto v2 = Vec3f::fromVec(*iter2, *(iter2+1));
            auto c = Vec3f::cross(v1, v2);

            if (c.z() <= FLOAT_EPS) {
                iter2 = pts.erase(iter2);
            } else {
                iter2++;
            }
        }
    }
}


/* Irregular hexagon pyramid 
 * parameter: dist, defines the distance from origin of each face. Must contains 6 numbers. The distance of a
 *            normal hexagon is defined as 1.
 * parameter: idx, defines the Miller index of upper and lower pyramidal segments. Must contains 4 numbers.
 * parameter: h, defines the height of each segment.
 *            h[0] and h[2] are the heights of upper and lower pyramidal segments, defined as height / H, where
 *            H is the maximum possible height.
 *            h[1] are the heights of middle cylindrical segment, defined as height / a, where a is the
 *            diameter of original circumcircle.
 */
Crystal* Crystal::createIrregularHexPyramid(float *dist, int *idx, float *h)
{
    /* There are 18 faces. The crystal is constructed of the intersection of all these 18 half-spaces.
     * 1. Find all inner point as vertexes.
     * 2. Find all co-planner points. 
     * 3. For points in each face, construct a triangular division.
     */
    using namespace Math;

    constexpr int CONSTRAINT_NUM = 18;
    constexpr float SQRT3 = 1.73205080757f;
    float a[CONSTRAINT_NUM] = { SQRT3, 0.0f, -SQRT3, -SQRT3, 0.0f, SQRT3 };

    std::vector<Vec3f> pts;
    return nullptr;
}

};
