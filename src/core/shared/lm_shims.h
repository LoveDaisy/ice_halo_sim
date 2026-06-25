// Per-platform shim macros shared by all `*_shared.h` headers in this directory.
// Provides a function-qualifier macro (LM_FN), scalar-math built-ins, and a
// small set of address-space qualifier macros so a single function body
// compiles under host C++, CUDA (nvcc), and MSL (xcrun metal).
//
// Surface contract:
//   - Scalar math + plain pointers / references are allowed via the macros
//     below. No atomics, no threadgroup constructs, no MSL [[attributes]].
//   - Address-space macros (LM_THREAD / LM_DEVICE / LM_CONSTANT_REF) expand to
//     the MSL qualifier on MSL and to empty on CUDA/host — pcg_shared.h uses
//     these to bridge MSL `thread` / `device const` / `constant&` parameters
//     to plain pointers/refs on CUDA and host.
#ifndef LM_SHIMS_H_
#define LM_SHIMS_H_

#if defined(__METAL_VERSION__)
// MSL — compiled by `xcrun metal -c`.
// LM_CONSTANT — program-scope numeric constant qualifier. MSL requires the
// `constant` address space for any variable defined at namespace/file scope;
// host C++ / CUDA accept plain `constexpr`.
#define LM_FN inline
#define LM_CONSTANT constant constexpr
// Address-space macros (parameters / pointers / references).
#define LM_THREAD thread
#define LM_DEVICE device
#define LM_CONSTANT_REF constant
// Math built-ins.
#define LM_SQRT(x) metal::sqrt(x)
#define LM_ATAN2(y, x) metal::atan2((y), (x))
#define LM_ASIN(x) metal::asin(x)
#define LM_ACOS(x) metal::acos(x)
#define LM_TAN(x) metal::tan(x)
#define LM_COS(x) metal::cos(x)
#define LM_SIN(x) metal::sin(x)
#define LM_LOG(x) metal::log(x)
#define LM_FABS(x) metal::fabs(x)
#define LM_FMOD(x, y) metal::fmod((x), (y))
#define LM_COPYSIGN(x, y) metal::copysign((x), (y))
#define LM_FMAX(x, y) metal::max((x), (y))
#define LM_FMIN(x, y) metal::min((x), (y))
#define LM_CLAMP(x, a, b) metal::clamp((x), (a), (b))
#define LM_PI_F (M_PI_F)
#define LM_PI_2F (M_PI_2_F)
#elif defined(__CUDACC__)
// CUDA — compiled by nvcc.
#define LM_FN __host__ __device__ inline
#define LM_CONSTANT constexpr
// Address-space macros — empty on CUDA: kernel-argument pointers / refs are
// untagged and the called __device__ function takes plain pointers/refs.
#define LM_THREAD
#define LM_DEVICE
#define LM_CONSTANT_REF
// Math built-ins.
#define LM_SQRT(x) sqrtf(x)
#define LM_ATAN2(y, x) atan2f((y), (x))
#define LM_ASIN(x) asinf(x)
#define LM_ACOS(x) acosf(x)
#define LM_TAN(x) tanf(x)
#define LM_COS(x) cosf(x)
#define LM_SIN(x) sinf(x)
#define LM_LOG(x) logf(x)
#define LM_FABS(x) fabsf(x)
#define LM_FMOD(x, y) fmodf((x), (y))
#define LM_COPYSIGN(x, y) copysignf((x), (y))
#define LM_FMAX(x, y) fmaxf((x), (y))
#define LM_FMIN(x, y) fminf((x), (y))
#define LM_CLAMP(x, a, b) fminf(fmaxf((x), (a)), (b))
#define LM_PI_F 3.14159265358979323846f
#define LM_PI_2F 1.5707963267948966f
#else
// Host C++.
#include <algorithm>
#include <cmath>
#define LM_FN inline
#define LM_CONSTANT constexpr
#define LM_THREAD
#define LM_DEVICE
#define LM_CONSTANT_REF
#define LM_SQRT(x) std::sqrt(x)
#define LM_ATAN2(y, x) std::atan2((y), (x))
#define LM_ASIN(x) std::asin(x)
#define LM_ACOS(x) std::acos(x)
#define LM_TAN(x) std::tan(x)
#define LM_COS(x) std::cos(x)
#define LM_SIN(x) std::sin(x)
#define LM_LOG(x) std::log(x)
#define LM_FABS(x) std::fabs(x)
#define LM_FMOD(x, y) std::fmod((x), (y))
#define LM_COPYSIGN(x, y) std::copysign((x), (y))
#define LM_FMAX(x, y) std::fmax((x), (y))
#define LM_FMIN(x, y) std::fmin((x), (y))
#define LM_CLAMP(x, a, b) std::clamp((x), (a), (b))
#define LM_PI_F 3.14159265358979323846f
#define LM_PI_2F 1.5707963267948966f
#endif

#endif  // LM_SHIMS_H_
