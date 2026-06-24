// Per-platform shim macros shared by all `*_shared.h` headers in this directory.
// Provides a function-qualifier macro (LM_FN) and a small set of math built-in
// macros so a single function body compiles under host C++, CUDA (nvcc), and
// MSL (xcrun metal).
//
// Surface contract: scalar math only. No pointers, address-spaces, atomics, or
// threadgroup constructs are allowed in headers that consume these macros.
#ifndef LM_SHIMS_H_
#define LM_SHIMS_H_

#if defined(__METAL_VERSION__)
// MSL — compiled by `xcrun metal -c`.
#define LM_FN inline
#define LM_SQRT(x) metal::sqrt(x)
#define LM_ATAN2(y, x) metal::atan2((y), (x))
#define LM_ASIN(x) metal::asin(x)
#define LM_ACOS(x) metal::acos(x)
#define LM_TAN(x) metal::tan(x)
#define LM_CLAMP(x, a, b) metal::clamp((x), (a), (b))
#define LM_PI_2F (M_PI_2_F)
#elif defined(__CUDACC__)
// CUDA — compiled by nvcc.
#define LM_FN __host__ __device__ inline
#define LM_SQRT(x) sqrtf(x)
#define LM_ATAN2(y, x) atan2f((y), (x))
#define LM_ASIN(x) asinf(x)
#define LM_ACOS(x) acosf(x)
#define LM_TAN(x) tanf(x)
#define LM_CLAMP(x, a, b) fminf(fmaxf((x), (a)), (b))
#define LM_PI_2F 1.5707963267948966f
#else
// Host C++.
#include <algorithm>
#include <cmath>
#define LM_FN inline
#define LM_SQRT(x) std::sqrt(x)
#define LM_ATAN2(y, x) std::atan2((y), (x))
#define LM_ASIN(x) std::asin(x)
#define LM_ACOS(x) std::acos(x)
#define LM_TAN(x) std::tan(x)
#define LM_CLAMP(x, a, b) std::clamp((x), (a), (b))
#define LM_PI_2F 1.5707963267948966f
#endif

#endif  // LM_SHIMS_H_
