#include "linearalgebra.h"

#include <cmath>
#include <cstring>
#include <cstdio>

float LinearAlgebra::dot3(const float *vec1, const float *vec2)
{
    return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
}

void LinearAlgebra::cross3(const float *vec1, const float *vec2, float* vec)
{
    vec[0] = -vec2[1]*vec1[2] + vec1[1]*vec2[2];
    vec[1] = vec2[0]*vec1[2] - vec1[0]*vec2[2];
    vec[2] = -vec2[0]*vec1[1] + vec1[0]*vec2[1];
}

float LinearAlgebra::norm3(const float *vec)
{
    return std::sqrt(dot3(vec, vec));
}

float LinearAlgebra::diffNorm3(const float *vec1, const float *vec2)
{
    float v[3];
    vec3FromTo(vec1, vec2, v);
    return norm3(v);
}

void LinearAlgebra::normalize3(float *vec)
{
    float len = norm3(vec);
    vec[0] /= len;
    vec[1] /= len;
    vec[2] /= len;
}

void LinearAlgebra::normalized3(const float *vec, float *vec_n)
{
    float len = norm3(vec);
    vec_n[0] = vec[0] / len;
    vec_n[1] = vec[1] / len;
    vec_n[2] = vec[2] / len;
}

void LinearAlgebra::vec3FromTo(const float *vec1, const float *vec2, float *vec)
{
    vec[0] = vec2[0] - vec1[0];
    vec[1] = vec2[1] - vec1[1];
    vec[2] = vec2[2] - vec1[2];
}

// void LinearAlgebra::rotateByAxisAngle(const float *ax, float angle, float *vec)
// {
//     float c = std::cos(angle);
//     float s = std::sin(angle);

//     float matR[9] = {c + ax[0] * ax[0] * (1-c), ax[1] * ax[0] * (1-c) + ax[2] * s, ax[2] * ax[0] * (1-c) - ax[1] * s,
//         ax[0] * ax[1] * (1-c) - ax[2] * s, c + ax[1] * ax[1] * (1-c), ax[2] * ax[1] * (1-c) + ax[0] * s,
//         ax[0] * ax[2] * (1-c) + ax[1] * s, ax[1] * ax[2] * (1-c) - ax[0] * s, c + ax[2] * ax[2] * (1-c)};
//     float res[3] = {0.0f};

//     DummyMatrix v(vec, 1, 3);
//     DummyMatrix R(matR, 3, 3);
//     DummyMatrix vn(res, 1, 3);
//     DummyMatrix::multiply(v, R, vn);

//     memcpy(vec, res, 3*sizeof(float));
// }

void LinearAlgebra::rotateBase(const float *ax, float angle, float *vec)
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


void LinearAlgebra::rotateZ(const float *lon_lat_roll, float *vec, int dataNum)
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


void LinearAlgebra::rotateZBack(const float *lon_lat_roll, float *vec, int dataNum)
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