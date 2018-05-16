#include "Halide.h"

#include <limits>

namespace {

const int DEFAULT_NUM = 1024*1024;
const float MAX_FLOAT = std::numeric_limits<float>::max();

class RayPropagate : public Halide::Generator<RayPropagate> {
public:
    Input<Buffer<float>>    dir{"dir", 2};
    Input<Buffer<float>>    pt{"pt", 2};
    Input<Buffer<float>>    face{"face", 2};

    Output<Buffer<float>>   newPt{"newPt", 2};
    Output<Buffer<int>>     newFaceId{"newFaceId", 1};

    Var x{"x"}, y{"y"}, yf{"yf"}, z{"z"};

    void generate() {
        Func face_base{"face_base"};
        face_base(x, yf, z) = select(z == 0, face(x+3, yf) - face(x, yf),
                                             face(x+6, yf) - face(x, yf));

        Expr a1 = face_base(0, yf, 0)*face_base(1, yf, 1)*face(2, yf) + face_base(1, yf, 0)*face_base(2, yf, 1)*face(0, yf) +
            face_base(2, yf, 0)*face_base(0, yf, 1)*face(1, yf) - face_base(2, yf, 0)*face_base(1, yf, 1)*face(0, yf) -
            face_base(1, yf, 0)*face_base(0, yf, 1)*face(2, yf) - face_base(0, yf, 0)*face_base(2, yf, 1)*face(1, yf);
        Expr b1 = pt(0, y)*face_base(1, yf, 0)*face_base(2, yf, 1) + pt(1, y)*face_base(2, yf, 0)*face_base(0, yf, 1) +
            pt(2, y)*face_base(0, yf, 0)*face_base(1, yf, 1) - pt(0, y)*face_base(2, yf, 0)*face_base(1, yf, 1) -
            pt(1, y)*face_base(0, yf, 0)*face_base(2, yf, 1) - pt(2, y)*face_base(1, yf, 0)*face_base(0, yf, 1);
        Expr c1 = dir(0, y)*face_base(1, yf, 0)*face_base(2, yf, 1) + dir(1, y)*face_base(2, yf, 0)*face_base(0, yf, 1) +
            dir(2, y)*face_base(0, yf, 0)*face_base(1, yf, 1) - dir(0, y)*face_base(2, yf, 0)*face_base(1, yf, 1) -
            dir(1, y)*face_base(0, yf, 0)*face_base(2, yf, 1) - dir(2, y)*face_base(1, yf, 0)*face_base(0, yf, 1);
        Expr t = select(abs(c1) > 1e-6f, (a1 - b1) / c1, -1);
        // Expr t = (a1 - b1) / c1;

        Expr a2 = dir(0, y)*pt(1, y)*face_base(2, yf, 1) + dir(1, y)*pt(2, y)*face_base(0, yf, 1) +
            dir(2, y)*pt(0, y)*face_base(1, yf, 1) - dir(0, y)*pt(2, y)*face_base(1, yf, 1) -
            dir(1, y)*pt(0, y)*face_base(2, yf, 1) - dir(2, y)*pt(1, y)*face_base(0, yf, 1);
        Expr b2 = dir(0, y)*face_base(1, yf, 1)*face(2, yf) + dir(1, y)*face_base(2, yf, 1)*face(0, yf) +
            dir(2, y)*face_base(0, yf, 1)*face(1, yf) - dir(0, y)*face_base(2, yf, 1)*face(1, yf) -
            dir(1, y)*face_base(0, yf, 1)*face(2, yf) - dir(2, y)*face_base(1, yf, 1)*face(0, yf);
        Expr alpha = select(abs(c1) > 1e-6f, (a2 + b2) / c1, -1);
        // Expr alpha = (a2 + b2) / c1;

        Expr a3 = dir(0, y)*pt(1, y)*face_base(2, yf, 0) + dir(1, y)*pt(2, y)*face_base(0, yf, 0) +
            dir(2, y)*pt(0, y)*face_base(1, yf, 0) - dir(0, y)*pt(2, y)*face_base(1, yf, 0) -
            dir(1, y)*pt(0, y)*face_base(2, yf, 0) - dir(2, y)*pt(1, y)*face_base(0, yf, 0);
        Expr b3 = dir(0, y)*face_base(1, yf, 0)*face(2, yf) + dir(1, y)*face_base(2, yf, 0)*face(0, yf) +
            dir(2, y)*face_base(0, yf, 0)*face(1, yf) - dir(0, y)*face_base(2, yf, 0)*face(1, yf) -
            dir(1, y)*face_base(0, yf, 0)*face(2, yf) - dir(2, y)*face_base(1, yf, 0)*face(0, yf);
        Expr beta = select(abs(c1) > 1e-6f, -(a3 + b3) / c1, -1);
        // Expr beta =  -(a3 + b3) / c1;

        Func t_all{"t_all"};
        t_all(y, yf) = select(alpha >= 0 && beta >= 0 && alpha + beta <= 1 && t > 3e-6f, t, MAX_FLOAT);

        RDom r(0, face.dim(1).extent());
        Tuple argmin_t = argmin(t_all(y, r));

        newFaceId(y) = select(argmin_t[1] < MAX_FLOAT, argmin_t[0], -1);
        newPt(x, y) = select(newFaceId(y) >= 0, argmin_t[1] * dir(x, y) + pt(x, y), 0);
    }

    void schedule() {
        if (auto_schedule) {
            // Provide estimates on the input
            dir.dim(1).set_bounds_estimate(0, DEFAULT_NUM);
            dir.dim(0).set_bounds_estimate(0, 3);

            pt.dim(1).set_bounds_estimate(0, DEFAULT_NUM);
            pt.dim(0).set_bounds_estimate(0, 3);

            face.dim(1).set_bounds_estimate(0, 128);
            face.dim(0).set_bounds_estimate(0, 9);

            // Provide estimates on the pipeline output
            newFaceId.estimate(y, 0, DEFAULT_NUM);
            newPt.estimate(x, 0, 3).estimate(y, 0, DEFAULT_NUM);

        } else {
            // TODO
        }
    }
};

}

HALIDE_REGISTER_GENERATOR(RayPropagate, ray_prop)