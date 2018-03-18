#include "Halide.h"

namespace {

class RayHit : public Halide::Generator<RayHit> {
public:
    Input<Buffer<float>>    dir{"dir", 2};
    Input<Buffer<float>>    norm{"norm", 2};

    Input<float>            n{"n"};

    Output<Buffer<float>>   dir_reflect{"dir_reflect", 2};
    Output<Buffer<float>>   dir_refract{"dir_refract", 2};
    Output<Buffer<float>>   reflect_w{"reflect_w", 1};

    Var x{"x"}, y{"y"};

    void generate() {
        RDom r(0, 3);
        Func dot_tmp{"dot_tmp"};
        dot_tmp(x, y) = dir(x, y) * norm(x, y);
        Expr c = sum(dot_tmp(r, y));
        Expr s = sqrt(1.0f - c * c);
        Expr abs_c = abs(c);

        Expr n1n2 = select(c > 0, n, 1.0f / n);
        Expr dsqrt = sqrt(max(1.0f - (n1n2 * s) * (n1n2 * s), 0));
        Expr Rs = (n1n2 * abs_c -  dsqrt) / (n1n2 * abs_c + dsqrt);
        Expr Rp = (n1n2 * dsqrt -  abs_c) / (n1n2 * dsqrt + abs_c);

        reflect_w(y) = (Rs * Rs + Rp * Rp) / 2.0f;

        dir_reflect(x, y) = dir(x, y) - 2 * c * norm(x, y);
        
        Expr d = (1.0f - n1n2 * n1n2) / (c * c) + n1n2 * n1n2;
        dir_refract(x, y) = select(d <= 0, dir_reflect(x, y),
            n1n2 * dir(x, y) - (n1n2 - sqrt(d)) * c * norm(x, y));
    }

    void schedule() {
        if (auto_schedule) {
            // Provide estimates on the input
            dir.dim(1).set_bounds_estimate(0, 1024*1024);
            dir.dim(0).set_bounds_estimate(0, 3);

            norm.dim(1).set_bounds_estimate(0, 1024*1024);
            norm.dim(0).set_bounds_estimate(0, 3);

            n.set_estimate(1.3f);

            // Provide estimates on the pipeline output
            dir_reflect.estimate(x, 0, 3).estimate(y, 0, 1024*1024);
            dir_refract.estimate(x, 0, 3).estimate(y, 0, 1024*1024);
            reflect_w.estimate(y, 0, 1024*1024);

        } else {
            // TODO
        }
    }
};

}

HALIDE_REGISTER_GENERATOR(RayHit, ray_hit)