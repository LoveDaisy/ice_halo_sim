#ifndef LINEARALGEBRA_H
#define LINEARALGEBRA_H


class LinearAlgebra
{
public:

    static float dot3(const float *vec1, const float *vec2);
    static void cross3(const float *vec1, const float *vec2, float* vec);
    static float norm3(const float *vec);
    static void normalize3(float *vec);
    static void normalized3(const float *vec, float *vec_n);
    static void vec3FromTo(const float *vec1, const float *vec2, float *vec);

    static void rotateByAxisAngle(const float *ax, float angle, float *vec);
    static void rotateByAxisAngle(const float *ax, float angle, int num, float *vec);

    static void rotateZ(float lon, float lat, float roll, int num, float *vec);
    static void rotateZBack(float lon, float lat, float roll, int num, float *vec);
};

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

#endif // LINEARALGEBRA_H