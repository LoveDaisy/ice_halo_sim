// accum_shared.h — single-source XYZ accumulation kernel
//
// Surface contract
//   AccumXyzToPixel(buf, pix_flat, cmf_x, cmf_y, cmf_z, weight)
//     Atomically adds weight * (cmf_x, cmf_y, cmf_z) to a 3-channel XYZ
//     pixel at `pix_flat` inside a W*H*3 float buffer.
//
//   NeumaierAdd(sum, comp, delta)  [HOST only]
//     Compensated summation for cross-batch host-side accumulation. Device
//     accumulation uses a single atomicAdd (parity-proven). NeumaierAdd is
//     reserved for `RenderConsumer` cross-batch reduction where the
//     accumulation chain length (100-1000 batches) is the principal precision
//     hazard.
//
// Backend variants
//   MSL (`__METAL_VERSION__`):   buf is `device atomic_float*`
//   CUDA (`__CUDACC__`):         buf is `float*`        (atomicAdd intrinsic)
//   Host C++:                    buf is `float*`        (plain +=, used by
//                                                       parity oracles; not
//                                                       the cross-batch path)
//
// Note: the parameter type cannot be unified across all three backends because
// MSL requires the `device atomic_float*` qualifier for atomic_fetch_add.
// Function body logic is identical across the three variants.

#ifndef LM_ACCUM_SHARED_H_
#define LM_ACCUM_SHARED_H_

#if defined(__METAL_VERSION__)
// ===== MSL =====
#include <metal_stdlib>

inline void AccumXyzToPixel(device metal::atomic_float* xyz_buf, uint32_t pix_flat, float cmf_x, float cmf_y,
                            float cmf_z, float weight) {
  uint32_t base = pix_flat * 3u;
  metal::atomic_fetch_add_explicit(xyz_buf + base + 0u, cmf_x * weight, metal::memory_order_relaxed);
  metal::atomic_fetch_add_explicit(xyz_buf + base + 1u, cmf_y * weight, metal::memory_order_relaxed);
  metal::atomic_fetch_add_explicit(xyz_buf + base + 2u, cmf_z * weight, metal::memory_order_relaxed);
}

#elif defined(__CUDACC__)
// ===== CUDA =====
__device__ inline void AccumXyzToPixel(float* xyz_buf, uint32_t pix_flat, float cmf_x, float cmf_y, float cmf_z,
                                       float weight) {
  uint32_t base = pix_flat * 3u;
  atomicAdd(xyz_buf + base + 0u, cmf_x * weight);
  atomicAdd(xyz_buf + base + 1u, cmf_y * weight);
  atomicAdd(xyz_buf + base + 2u, cmf_z * weight);
}

#else
// ===== Host C++ =====
#include <cmath>
#include <cstdint>

inline void AccumXyzToPixel(float* xyz_buf, std::uint32_t pix_flat, float cmf_x, float cmf_y, float cmf_z,
                            float weight) {
  std::uint32_t base = pix_flat * 3u;
  xyz_buf[base + 0u] += cmf_x * weight;
  xyz_buf[base + 1u] += cmf_y * weight;
  xyz_buf[base + 2u] += cmf_z * weight;
}

// Neumaier compensated summation. On each call adds `delta` into `sum`
// while accumulating the lost low-order bits into `comp`. Caller is
// responsible for flushing `comp` into `sum` at a safe synchronization
// point (typically only matters for very long chains; for image
// accumulation across N batches the residual `comp` stays small enough
// that the snapshot path can ignore it).
inline void NeumaierAdd(float& sum, float& comp, float delta) {
  float new_sum = sum + delta;
  comp += (std::fabs(delta) < std::fabs(sum)) ? ((sum - new_sum) + delta) : ((delta - new_sum) + sum);
  sum = new_sum;
}

#endif

#endif  // LM_ACCUM_SHARED_H_
