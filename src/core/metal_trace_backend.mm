// Metal backend for TraceBackend (Apple platforms). Pimpl over Objective-C
// Metal types so the public header stays pure C++. The kernel source is
// embedded as the kKernelSrc C++ string below; doc/trace_layer.metal is
// the syntax-check / readable reference — both MUST stay logically equivalent
// (kernel body identical; the .metal file may carry additional comments).

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/color_util.hpp"
#include "core/crystal.hpp"
#include "core/exit_seam.hpp"
#include "core/filter_spec.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/metal_trace_backend.hpp"
#include "core/projection.hpp"
#include "core/raypath.hpp"
#include "core/scatter_accum.hpp"
#include "core/simulator.hpp"  // PartitionCrystalRayNum
#include "core/trace_ops.hpp"
#include "util/color_data.hpp"
#include "util/logger.hpp"

namespace lumice {

namespace {

// Mirror of doc/trace_layer.metal. Edit both in lock-step.
constexpr const char* kKernelSrc = R"METAL(
#include <metal_stdlib>
using namespace metal;

constant float  kFloatEps  = 1e-5f;
constant ushort kInvalidId = 0xffffu;
constant uint   kRecCap    = 64u;

struct KernelParams {
  float n_idx;
  uint  max_hits;
  uint  poly_cnt;
  uint  num_rays;
  uint  img_w;
  uint  img_h;
  float az0;
  float cie_x;
  float cie_y;
  float cie_z;
  uint  ms_mode;
  uint  out_cap;
  // Projection routing (host fills per BeginSession lens type):
  //   proj_type == 0: rectangular (legacy az0-projection, uses az0)
  //   proj_type == 1: dual_fisheye_equal_area (uses r_scale / max_abs_dz)
  uint  proj_type;
  float r_scale;     // equal-area r_scale = 1/sqrt(1+overlap); 1.0 if no overlap
  float max_abs_dz;  // overlap zone |sky_z| threshold; 0 = no overlap
  // Buffer-egress (exit seam, scrum-258.1): kernel's final-exit branch writes
  // {world_dir(3), weight} to exit_d/exit_w at a session-level atomic slot
  // (exit_slot). Capacity bound is host-supplied (ComputeOutCap).
  uint  exit_cap;
  // Exit metadata (scrum-258.2): rich {dir, weight, path, crystal_id,
  // ms_layer_idx} record. crystal_id = ci for this dispatch; ms_layer_idx =
  // host-side ms_idx (only the final layer writes exit slots in ms_mode==0).
  // face_seq_cap = min(max_hits, ExitFaceSeq::kCap=15) — per-slot stride of
  // exit_face_seq_data_buf; <15 saves device memory + readback bandwidth.
  uint  crystal_id;
  uint  face_seq_cap;
  uint  ms_layer_idx;
};

inline float GetReflectRatio(float delta, float rr) {
  float d_sqrt = sqrt(delta);
  float Rs = (rr - d_sqrt) / (rr + d_sqrt); Rs *= Rs;
  float Rp = (1.0f - rr * d_sqrt) / (1.0f + rr * d_sqrt); Rp *= Rp;
  return (Rs + Rp) * 0.5f;
}

kernel void trace_layer_kernel(
    device const float*    root_d   [[buffer(0)]],
    device const float*    root_p   [[buffer(1)]],
    device const float*    root_w   [[buffer(2)]],
    device const ushort*   root_tf  [[buffer(3)]],
    device const float*    poly_n   [[buffer(4)]],
    device const float*    poly_d   [[buffer(5)]],
    device const float*    centroid [[buffer(6)]],
    constant KernelParams& prm      [[buffer(7)]],
    device atomic_float*   image    [[buffer(8)]],
    device float*          out_d    [[buffer(9)]],
    device float*          out_p    [[buffer(10)]],
    device float*          out_w    [[buffer(11)]],
    device ushort*         out_tf   [[buffer(12)]],
    device atomic_uint*    counter  [[buffer(13)]],
    device float*          rec_sink [[buffer(14)]],
    device atomic_uint*    exit_cnt [[buffer(15)]],
    device atomic_float*   exit_wsum [[buffer(16)]],
    device const float*    root_rot [[buffer(17)]],
    device float*          exit_d   [[buffer(18)]],
    device float*          exit_w   [[buffer(19)]],
    device atomic_uint*    exit_slot [[buffer(20)]],
    device ushort*         exit_crystal_id    [[buffer(21)]],
    device uchar*          exit_face_seq_len  [[buffer(22)]],
    device uchar*          exit_face_seq_data [[buffer(23)]],
    // Cont metadata (scrum-258.3 Step 2): per-cont-ray crystal_id + face
    // sequence, written by ms_mode==1 exit branch in parallel with out_d/w/tf.
    // Read host-side by CopyContSliceToRootBuf to apply per-ray filter + prob.
    // ms_mode==0 dispatches bind these to non-nil dummy buffers; kernel never
    // writes through them on that path.
    device ushort*         cont_crystal_id    [[buffer(24)]],
    device uchar*          cont_face_seq_len  [[buffer(25)]],
    device uchar*          cont_face_seq_data [[buffer(26)]],
    uint tid [[thread_position_in_grid]]) {
  if (tid >= prm.num_rays) { return; }

  float dx = root_d[tid * 3u + 0u];
  float dy = root_d[tid * 3u + 1u];
  float dz = root_d[tid * 3u + 2u];
  float ox = root_p[tid * 3u + 0u];
  float oy = root_p[tid * 3u + 1u];
  float oz = root_p[tid * 3u + 2u];
  float w  = root_w[tid];
  ushort to_face = root_tf[tid];

  // Per-ray crystal->world rotation (row-major; world = m*v, mirroring
  // Rotation::Apply on the CPU). Applied to the exit direction before
  // projection so the seam returns world-space rays (invariant 6). The
  // orientation is constant for the whole ray path, so load it once.
  float m[9];
  for (uint k = 0u; k < 9u; k++) { m[k] = root_rot[tid * 9u + k]; }

  const float n_idx    = prm.n_idx;
  const uint  poly_cnt = prm.poly_cnt;

  const int   iw_i      = int(prm.img_w);
  const int   ih_i      = int(prm.img_h);
  const float img_w_f   = float(prm.img_w);
  const float img_h_f   = float(prm.img_h);
  const float short_res = float(min(prm.img_w / 2u, prm.img_h));
  const float proj_scl  = short_res / M_PI_F;

  ushort path[kRecCap];
  uint   rec_len = 0u;

  for (uint hit = 0u; hit < prm.max_hits; hit++) {
    if (to_face == kInvalidId) { break; }
    if (rec_len < kRecCap) { path[rec_len] = to_face; rec_len += 1u; }

    float nx = poly_n[to_face * 3u + 0u];
    float ny = poly_n[to_face * 3u + 1u];
    float nz = poly_n[to_face * 3u + 2u];
    float cos_theta = dx * nx + dy * ny + dz * nz;
    float rr = (cos_theta > 0.0f) ? n_idx : (1.0f / n_idx);
    float dd = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
    bool  is_tir = dd <= 0.0f;
    float w_refl = GetReflectRatio(max(dd, 0.0f), rr) * w;
    float w_refr = is_tir ? -1.0f : (w - w_refl);
    float rdx = dx - 2.0f * cos_theta * nx;
    float rdy = dy - 2.0f * cos_theta * ny;
    float rdz = dz - 2.0f * cos_theta * nz;
    float sd  = sqrt(max(dd, 0.0f));
    float fdx, fdy, fdz;
    if (is_tir) {
      fdx = rdx; fdy = rdy; fdz = rdz;
    } else {
      fdx = rr * dx - (rr - sd) * cos_theta * nx;
      fdy = rr * dy - (rr - sd) * cos_theta * ny;
      fdz = rr * dz - (rr - sd) * cos_theta * nz;
    }

    ushort cont_face = kInvalidId;
    float c_dx = 0.0f, c_dy = 0.0f, c_dz = 0.0f;
    float c_ox = 0.0f, c_oy = 0.0f, c_oz = 0.0f;
    float c_w  = 0.0f;

    for (uint ch = 0u; ch < 2u; ch++) {
      float cdx = (ch == 0u) ? rdx : fdx;
      float cdy = (ch == 0u) ? rdy : fdy;
      float cdz = (ch == 0u) ? rdz : fdz;
      float cw  = (ch == 0u) ? w_refl : w_refr;
      if (cw < 0.0f) { continue; }

      float t_far = 1e30f;
      int   far_face = -1;
      for (uint fi = 0u; fi < poly_cnt; fi++) {
        float fnx = poly_n[fi * 3u + 0u];
        float fny = poly_n[fi * 3u + 1u];
        float fnz = poly_n[fi * 3u + 2u];
        float fd  = poly_d[fi];
        float denom = cdx * fnx + cdy * fny + cdz * fnz;
        float t = -(ox * fnx + oy * fny + oz * fnz + fd) / denom;
        if (denom > kFloatEps && t < t_far) {
          t_far = t; far_face = int(fi);
        }
      }
      float eps_thr = (to_face != kInvalidId && far_face != int(to_face)) ? -kFloatEps : kFloatEps;
      if (far_face >= 0 && t_far > eps_thr) {
        cont_face = ushort(far_face);
        c_dx = cdx; c_dy = cdy; c_dz = cdz;
        c_ox = ox + t_far * cdx;
        c_oy = oy + t_far * cdy;
        c_oz = oz + t_far * cdz;
        c_w  = cw;
      } else {
        if (prm.ms_mode == 1u) {
          float ilen = rsqrt(cdx * cdx + cdy * cdy + cdz * cdz);
          float ux = cdx * ilen, uy = cdy * ilen, uz = cdz * ilen;
          int   ef = 0;
          float bestf = -1e30f;
          for (uint fi = 0u; fi < poly_cnt; fi++) {
            float facing = -(ux * poly_n[fi * 3u + 0u] +
                             uy * poly_n[fi * 3u + 1u] +
                             uz * poly_n[fi * 3u + 2u]);
            if (facing > bestf) { bestf = facing; ef = int(fi); }
          }
          // Return the continuation direction from crystal-local to world
          // space before writing it out (invariant 6 / DESIGN D3: any ray
          // leaving the kernel is world-space). The host re-samples a new
          // per-ray crystal_rot_ for the next layer and rotates back into the
          // new local frame (mirroring CPU CollectData + InitRayOtherMs). out_p
          // / out_tf are placeholders here — the host resamples them on the
          // next-layer crystal via InitRay_p_fid, so they are not relied upon.
          float wcx = m[0] * cdx + m[1] * cdy + m[2] * cdz;
          float wcy = m[3] * cdx + m[4] * cdy + m[5] * cdz;
          float wcz = m[6] * cdx + m[7] * cdy + m[8] * cdz;
          uint slot = atomic_fetch_add_explicit(counter, 1u, memory_order_relaxed);
          if (slot < prm.out_cap) {
            out_d[slot * 3u + 0u] = wcx;
            out_d[slot * 3u + 1u] = wcy;
            out_d[slot * 3u + 2u] = wcz;
            out_p[slot * 3u + 0u] = centroid[ef * 3u + 0u];
            out_p[slot * 3u + 1u] = centroid[ef * 3u + 1u];
            out_p[slot * 3u + 2u] = centroid[ef * 3u + 2u];
            out_w[slot] = cw;
            out_tf[slot] = ushort(ef);
            // Cont metadata (scrum-258.3 Step 2): record this cont ray's
            // {crystal_id, face_seq} in parallel buffers so the host hop
            // (CopyContSliceToRootBuf) can drive filter + prob without a
            // device→host round-trip. crystal_id == prm.crystal_id (the ci
            // for THIS dispatch); seq_len bounded by prm.face_seq_cap.
            cont_crystal_id[slot] = ushort(prm.crystal_id);
            uint cseq_len = min(rec_len, prm.face_seq_cap);
            cont_face_seq_len[slot] = uchar(cseq_len);
            for (uint k = 0u; k < cseq_len; k++) {
              cont_face_seq_data[slot * prm.face_seq_cap + k] = uchar(path[k]);
            }
          }
          atomic_fetch_add_explicit(exit_cnt, 1u, memory_order_relaxed);
          atomic_fetch_add_explicit(exit_wsum, cw, memory_order_relaxed);
        } else {
          // Return the exit direction from crystal-local to world space
          // (invariant 6 / DESIGN D2): world = m * cd, row-major, matching
          // CPU CollectData's crystal_rot_.Apply(r.d_). Without this the
          // per-ray-random local frame scatters the halo into a band.
          float wx = m[0] * cdx + m[1] * cdy + m[2] * cdz;
          float wy = m[3] * cdx + m[4] * cdy + m[5] * cdz;
          float wz = m[6] * cdx + m[7] * cdy + m[8] * cdz;
          // Buffer-egress (exit seam, scrum-258.1): export this exit ray
          // {world_dir, weight} BEFORE projection — projection clipping is a
          // consumer concern, and legacy outgoing_d_ likewise captures
          // pre-clip. Written once per exit ray (the overlap dual-write below
          // never re-exports). Backend path always captures (no env gate);
          // simulator drives consumer projection via ReadbackExitRays.
          {
            uint es = atomic_fetch_add_explicit(exit_slot, 1u, memory_order_relaxed);
            if (es < prm.exit_cap) {
              exit_d[es * 3u + 0u] = wx;
              exit_d[es * 3u + 1u] = wy;
              exit_d[es * 3u + 2u] = wz;
              exit_w[es] = cw;
              // Rich metadata (scrum-258.2): crystal_id from KernelParams,
              // face sequence truncated to face_seq_cap (= min(max_hits,
              // ExitFaceSeq::kCap=15) on the host). Stride = face_seq_cap so
              // <15-byte-per-slot saves device memory + readback bandwidth.
              exit_crystal_id[es] = ushort(prm.crystal_id);
              uint seq_len = min(rec_len, prm.face_seq_cap);
              exit_face_seq_len[es] = uchar(seq_len);
              for (uint k = 0u; k < seq_len; k++) {
                exit_face_seq_data[es * prm.face_seq_cap + k] = uchar(path[k]);
              }
            }
          }
          float sx = -wx;
          float sy = -wy;
          float sz = -wz;
          if (prm.proj_type == 0u) {
            float lon = atan2(sy, sx) - prm.az0;
            lon = lon - 2.0f * M_PI_F * floor((lon + M_PI_F) / (2.0f * M_PI_F));
            float lat = asin(clamp(sz, -1.0f, 1.0f));

            int raw_x = int(floor(lon * proj_scl + img_w_f * 0.5f + 0.5f));
            int ix = ((raw_x % iw_i) + iw_i) % iw_i;
            int iy = int(floor(-lat * proj_scl + img_h_f * 0.5f + 0.5f));
            if (iy >= 0 && iy < ih_i) {
              uint pix = (uint(iy) * prm.img_w + uint(ix)) * 3u;
              atomic_fetch_add_explicit(&image[pix + 0u], prm.cie_x * cw, memory_order_relaxed);
              atomic_fetch_add_explicit(&image[pix + 1u], prm.cie_y * cw, memory_order_relaxed);
              atomic_fetch_add_explicit(&image[pix + 2u], prm.cie_z * cw, memory_order_relaxed);
            }
          } else {
            // proj_type == 1: dual fisheye equal-area, primary + opposite-hemisphere overlap.
            // DRIFT(gpu-metal-shared-kernel): intentional duplicate of CPU render.cpp:192-214 +
            //   projection::{FisheyeEqualAreaForward, DualFisheyeToPixel} (re-home into a single-
            //   source shared kernel at CUDA step; backlog: 核心模拟 GPU 迁移路线).
            int   dual_short = min(int(prm.img_w) / 2, int(prm.img_h));
            float r          = float(dual_short) * 0.5f;
            float cy_pix     = img_h_f * 0.5f;
            float cx_left    = img_w_f * 0.5f - r;
            float cx_right   = img_w_f * 0.5f + r;

            bool  is_upper = (sz >= 0.0f);
            float z_hemi   = is_upper ? sz : -sz;  // own hemisphere: +|sz|
            float k = prm.r_scale / sqrt(1.0f + clamp(z_hemi, -1.0f + 1e-6f, 1.0f));
            float xn = k * sx;
            float yn = k * sy;
            float cx_primary = is_upper ? cx_left : cx_right;
            float sx_primary = is_upper ? -1.0f : 1.0f;
            float fx = sx_primary * yn * r + cx_primary;
            float fy = xn * r + cy_pix;
            int ix = int(floor(fx + 0.5f));
            int iy = int(floor(fy + 0.5f));
            if (ix >= 0 && ix < iw_i && iy >= 0 && iy < ih_i) {
              uint pix = (uint(iy) * prm.img_w + uint(ix)) * 3u;
              atomic_fetch_add_explicit(&image[pix + 0u], prm.cie_x * cw, memory_order_relaxed);
              atomic_fetch_add_explicit(&image[pix + 1u], prm.cie_y * cw, memory_order_relaxed);
              atomic_fetch_add_explicit(&image[pix + 2u], prm.cie_z * cw, memory_order_relaxed);
            }
            // Overlap dual-write: mirror CPU render.cpp:192-214 (opposite hemisphere, dz<0).
            // Does NOT increment exit_wsum — preserves normalization parity with CPU.
            if (prm.max_abs_dz > 0.0f && z_hemi < prm.max_abs_dz) {
              float z_opp = -z_hemi;  // = -|sz| < 0 (matches z_hemi_opp)
              float k2    = prm.r_scale / sqrt(1.0f + clamp(z_opp, -1.0f + 1e-6f, 1.0f));
              float xn2   = k2 * sx;
              float yn2   = k2 * sy;
              bool  is_upper_opp = !is_upper;
              float cx_opp       = is_upper_opp ? cx_left : cx_right;
              float sx_opp       = is_upper_opp ? -1.0f : 1.0f;
              float fx2 = sx_opp * yn2 * r + cx_opp;
              float fy2 = xn2 * r + cy_pix;
              int   ix2 = int(floor(fx2 + 0.5f));
              int   iy2 = int(floor(fy2 + 0.5f));
              if (ix2 >= 0 && ix2 < iw_i && iy2 >= 0 && iy2 < ih_i) {
                uint pix2 = (uint(iy2) * prm.img_w + uint(ix2)) * 3u;
                atomic_fetch_add_explicit(&image[pix2 + 0u], prm.cie_x * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix2 + 1u], prm.cie_y * cw, memory_order_relaxed);
                atomic_fetch_add_explicit(&image[pix2 + 2u], prm.cie_z * cw, memory_order_relaxed);
              }
            }
          }
          atomic_fetch_add_explicit(exit_cnt, 1u, memory_order_relaxed);
          atomic_fetch_add_explicit(exit_wsum, cw, memory_order_relaxed);
        }
      }
    }

    if (cont_face == kInvalidId) {
      to_face = kInvalidId;
    } else {
      dx = c_dx; dy = c_dy; dz = c_dz;
      ox = c_ox; oy = c_oy; oz = c_oz;
      w = c_w;
      to_face = cont_face;
    }
  }

  float rec_csum = 0.0f;
  for (uint k = 0u; k < rec_len; k++) {
    rec_csum += float(path[k]);
  }
  rec_sink[tid] = rec_csum;
}
)METAL";

// Mirror of the Metal-side KernelParams (host layout MUST match the .metal
// struct field-for-field — all 4-byte scalars, natural alignment).
// NOTE: field order MUST match MSL KernelParams in kKernelSrc — static_assert
// guards size only; reviewer-facing field-by-field check is the maintainer's
// responsibility when adding/reordering members.
struct KernelParams {
  float    n_idx;
  uint32_t max_hits;
  uint32_t poly_cnt;
  uint32_t num_rays;
  uint32_t img_w;
  uint32_t img_h;
  float    az0;
  float    cie_x;
  float    cie_y;
  float    cie_z;
  uint32_t ms_mode;
  uint32_t out_cap;
  uint32_t proj_type;
  float    r_scale;
  float    max_abs_dz;
  uint32_t exit_cap;  // exit seam (scrum-258.1) — buffer-egress capacity (rays)
  // Exit metadata (scrum-258.2). Field order MUST match the MSL struct above
  // (crystal_id, face_seq_cap, ms_layer_idx) — silent reorder would only
  // change semantics, not sizeof, so the static_assert below is necessary
  // but insufficient. Reviewer-side cross-check between host + MSL is
  // mandatory until layout_test (plan §7 Risk 2) lands.
  uint32_t crystal_id;
  uint32_t face_seq_cap;
  uint32_t ms_layer_idx;
};
static_assert(sizeof(KernelParams) == 76u,
              "KernelParams size mismatch — update host struct to match Metal-side layout");

float ComputeAz0(const Rotation& rot) {
  float ax_z[3]{ 0.0f, 0.0f, 1.0f };
  rot.Apply(ax_z);
  return std::atan2(ax_z[1], ax_z[0]);
}

void ComputeCmf(float wl, float& cie_x, float& cie_y, float& cie_z) {
  int wl_key = static_cast<int>(wl + 0.5f);
  if (wl_key < kCmfMinWavelength || wl_key > kCmfMaxWavelength) {
    cie_x = cie_y = cie_z = 0.0f;
    return;
  }
  int idx = wl_key - kCmfMinWavelength;
  cie_x = kCmfX[idx];
  cie_y = kCmfY[idx];
  cie_z = kCmfZ[idx];
}

size_t ComputeOutCap(size_t n, size_t max_hits) {
  // explore mh16 ~9.8x fan-out; (max_hits*2+4) covers the observed upper
  // bound with margin; cap at 64M to bound device-memory usage.
  size_t cap = n * (max_hits * 2 + 4);
  return std::min<size_t>(cap, 64u * 1024u * 1024u);
}

Logger& EffectiveLogger(Logger* logger) {
  return logger ? *logger : GetGlobalLogger();
}

}  // namespace

struct MetalTraceBackend::Impl {
  Logger* logger_ = nullptr;

  id<MTLDevice>               device = nil;
  id<MTLCommandQueue>         queue  = nil;
  id<MTLComputePipelineState> pso    = nil;

  SessionSpec spec{};
  bool   in_session = false;
  size_t ms_idx = 0;
  size_t root_ray_count = 0;
  int    width = 0;
  int    height = 0;
  float  az0 = 0.0f;
  float  cie_x = 0.0f, cie_y = 0.0f, cie_z = 0.0f;
  // Projection routing — populated by BeginSession per render.lens_.type_.
  // Mirrors KernelParams fields with the same name; DispatchLayer copies them.
  uint32_t proj_type = 0u;
  float    r_scale = 1.0f;
  float    max_abs_dz = 0.0f;

  RandomNumberGenerator rng{ 0 };

  // Persistent crystal cache for the *current* TraceLayer (rebuilt per layer).
  Crystal current_crystal{};
  float   current_n_idx = 0.0f;
  bool    have_crystal = false;

  // XYZ accumulator (W*H*3 floats).
  id<MTLBuffer> xyz_image = nil;
  size_t        xyz_pix_capacity = 0;

  // Polygon geometry (uploaded per-layer; capacity-resized lazily).
  id<MTLBuffer> poly_n_buf  = nil;
  id<MTLBuffer> poly_d_buf  = nil;
  id<MTLBuffer> centroid_buf = nil;
  size_t        poly_capacity = 0;

  // First-layer root rays (uploaded from host).
  id<MTLBuffer> root_d_buf  = nil;
  id<MTLBuffer> root_p_buf  = nil;
  id<MTLBuffer> root_w_buf  = nil;
  id<MTLBuffer> root_tf_buf = nil;
  // Per-ray crystal->world rotation (9 floats/ray, row-major, == Rotation::mat_).
  // The kernel applies mat*exit_dir before projection to return exit rays from
  // crystal-local to world space (invariant 6 / DESIGN D2). Sized in lock-step
  // with root_d_buf by EnsureRootBuffers.
  id<MTLBuffer> root_rot_buf = nil;
  size_t        root_capacity = 0;

  // Continuation ping-pong (indexed by ms_idx & 1).
  id<MTLBuffer> cont_d[2]  = { nil, nil };
  id<MTLBuffer> cont_p[2]  = { nil, nil };
  id<MTLBuffer> cont_w[2]  = { nil, nil };
  id<MTLBuffer> cont_tf[2] = { nil, nil };
  // Cont metadata (scrum-258.3 Step 1): per-cont-ray {crystal_id, face_seq}
  // written by kernel ms_mode==1 branch in parallel with cont_d/p/w/tf, so the
  // host hop (CopyContSliceToRootBuf) can apply per-ray filter + prob without a
  // new device→host round-trip. Stride of cont_face_seq_data_buf = face_seq_cap_
  // (set in BeginSession from scene.max_hits_); EnsureContBuffer sizes all three
  // in lock-step with cont_d/p/w/tf.
  id<MTLBuffer> cont_crystal_id_buf[2]    = { nil, nil };
  id<MTLBuffer> cont_face_seq_len_buf[2]  = { nil, nil };
  id<MTLBuffer> cont_face_seq_data_buf[2] = { nil, nil };
  size_t        cont_capacity[2] = { 0, 0 };
  size_t        cont_counts[2]   = { 0, 0 };
  size_t        out_cap = 0;

  id<MTLBuffer> counter_buf  = nil;
  id<MTLBuffer> rec_sink_buf = nil;
  size_t        rec_sink_capacity = 0;
  size_t        max_produced = 0;

  // Exit-ray statistics buffers (parity harness). Both 4-byte atomic scalars
  // populated by the kernel in both ms_mode branches and read back after each
  // DispatchLayer; cached in `last_stats` for the producing TraceLayer to
  // hand off to MetalLayerHandle.
  id<MTLBuffer> exit_count_buf = nil;
  id<MTLBuffer> exit_w_sum_buf = nil;
  LayerStats    last_stats{};

  // Buffer-egress (exit seam, scrum-258.1): kernel exports world-space
  // {world_dir(3), weight} for every exit ray to exit_ray_d/w at a
  // session-level atomic slot (exit_slot_buf). ReadbackExitRays copies them
  // out for the simulator to drive the legacy consumer projection path,
  // replacing the per-batch O(W*H) ReadbackImage. Reset once per session
  // (first TraceLayer). Capture is unconditional for the backend path.
  id<MTLBuffer> exit_ray_d_buf = nil;
  id<MTLBuffer> exit_ray_w_buf = nil;
  id<MTLBuffer> exit_slot_buf  = nil;
  size_t        exit_ray_capacity = 0;
  // Exit metadata (scrum-258.2). Sized in lock-step with exit_ray_d/w by
  // EnsureExitBuffers; face_seq_data stride = face_seq_cap_ (set in
  // BeginSession from scene.max_hits_).
  id<MTLBuffer> exit_crystal_id_buf    = nil;
  id<MTLBuffer> exit_face_seq_len_buf  = nil;
  id<MTLBuffer> exit_face_seq_data_buf = nil;
  uint32_t      face_seq_cap_ = 0;

  // Per-layer-exit-processing (scrum-258.3): host-side mirrors of legacy
  // simulator.cpp:425 CollectData. Each non-final layer writes the ci →
  // Crystal/AxisDistribution/FilterConfig mapping into hop_*_[out_slot] (ping-
  // pong on ms_idx & 1), so CopyContSliceToRootBuf reads them via
  // cont_crystal_id_buf[in_slot] in the layer-boundary host hop. The final
  // layer writes its own ci → ... mapping into last_layer_* for
  // ReadbackExitRays.
  std::vector<Crystal>            hop_crystals_[2];
  std::vector<AxisDistribution>   hop_axis_dists_[2];
  std::vector<FilterConfig>       hop_filter_configs_[2];
  float                           hop_ms_prob_[2]      = { 0.0f, 0.0f };
  uint8_t                         hop_ms_layer_idx_[2] = { 0u, 0u };
  std::vector<Crystal>            last_layer_crystals_;
  std::vector<AxisDistribution>   last_layer_axis_dists_;
  std::vector<FilterConfig>       last_layer_filter_configs_;
  float                           last_ms_prob_      = 0.0f;
  uint8_t                         last_ms_layer_idx_ = 0u;
  // Mid-layer exits accumulated across non-final layers (legacy CollectData
  // mirror: filter-pass + rng >= ms_info.prob_ → emit outgoing in *this* layer).
  // Drained + merged with the final-layer exits in ReadbackExitRays.
  std::vector<ExitRayRecord> accumulated_mid_exits_;

  // Layer dispatch helpers.
  void EnsureDevice();
  void EnsurePso();
  void EnsureImage(int w, int h);
  void EnsurePolyBuffers(size_t poly_cnt);
  void EnsureRootBuffers(size_t n);
  void EnsureContBuffer(int slot);
  void EnsureRecSink(size_t n);
  void EnsureExitBuffers(size_t cap);  // exit seam (scrum-258.1)
  void UploadCrystal(const Crystal& crystal);
  void ResolveLayerCrystalForCi(const ScatteringSetting& setting, bool use_host,
                                const HostRayBatch& host_batch);
  size_t GenerateFirstLayerRootsForCi(const ScatteringSetting& setting,
                                      size_t ci, size_t crystal_ray_num);
  size_t CopyContSliceToRootBuf(const ScatteringSetting& setting, size_t ci_start, size_t ci_n, int in_slot);
  void DispatchLayer(size_t num_rays,
                     id<MTLBuffer> r_d, id<MTLBuffer> r_p,
                     id<MTLBuffer> r_w, id<MTLBuffer> r_tf,
                     uint32_t ms_mode, int out_slot,
                     uint32_t counter_init,
                     uint32_t crystal_id, uint32_t ms_layer_idx);
  void Reset();
};

void MetalTraceBackend::Impl::EnsureDevice() {
  if (device != nil) {
    return;
  }
  device = MTLCreateSystemDefaultDevice();
  if (device == nil) {
    NSArray<id<MTLDevice>>* all = MTLCopyAllDevices();
    if (all.count > 0) {
      device = all[0];
    }
  }
  assert(device != nil && "MetalTraceBackend: no Metal device available");
  queue = [device newCommandQueue];
  assert(queue != nil);
}

void MetalTraceBackend::Impl::EnsurePso() {
  if (pso != nil) {
    return;
  }
  NSError* err = nil;
  NSString* src = [NSString stringWithUTF8String:kKernelSrc];
  MTLCompileOptions* opts = [MTLCompileOptions new];
  // Disable fast-math / contract-FMA so the kernel's mul-add sequences round
  // identically to the CPU backend's separate operations. Without this the
  // Metal compiler fuses (a*b + c) into fma(a, b, c), drifting from CPU by
  // ~ULP per bounce and ~1e-4 over an N=4096 sum (observed during Test E
  // bring-up before this option was set).
  if (@available(macOS 15.0, *)) {
    opts.mathMode = MTLMathModeSafe;
  } else {
    opts.fastMathEnabled = NO;
  }
  id<MTLLibrary> lib = [device newLibraryWithSource:src options:opts error:&err];
  if (lib == nil) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: kernel compile failed: {}",
               err.localizedDescription.UTF8String);
    assert(false && "MetalTraceBackend: kernel compile failed");
  }
  id<MTLFunction> fn = [lib newFunctionWithName:@"trace_layer_kernel"];
  assert(fn != nil && "MetalTraceBackend: kernel entry point missing");
  pso = [device newComputePipelineStateWithFunction:fn error:&err];
  if (pso == nil) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: pipeline state creation failed: {}",
               err.localizedDescription.UTF8String);
    assert(false && "MetalTraceBackend: pipeline state creation failed");
  }
}

void MetalTraceBackend::Impl::EnsureImage(int w, int h) {
  size_t pix = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (pix != xyz_pix_capacity) {
    xyz_image = [device newBufferWithLength:pix * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
    assert(xyz_image != nil);
    xyz_pix_capacity = pix;
  }
  std::memset([xyz_image contents], 0, pix * 3 * sizeof(float));
}

void MetalTraceBackend::Impl::EnsurePolyBuffers(size_t poly_cnt) {
  if (poly_cnt <= poly_capacity) {
    return;
  }
  poly_capacity = poly_cnt;
  poly_n_buf  = [device newBufferWithLength:poly_cnt * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(poly_n_buf != nil);
  poly_d_buf  = [device newBufferWithLength:poly_cnt * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(poly_d_buf != nil);
  centroid_buf = [device newBufferWithLength:poly_cnt * 3 * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(centroid_buf != nil);
}

void MetalTraceBackend::Impl::EnsureRootBuffers(size_t n) {
  if (n <= root_capacity) {
    return;
  }
  root_capacity = n;
  root_d_buf  = [device newBufferWithLength:n * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_d_buf != nil);
  root_p_buf  = [device newBufferWithLength:n * 3 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_p_buf != nil);
  root_w_buf  = [device newBufferWithLength:n * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_w_buf != nil);
  root_tf_buf = [device newBufferWithLength:n * sizeof(uint16_t)
                                    options:MTLResourceStorageModeShared];
  assert(root_tf_buf != nil);
  root_rot_buf = [device newBufferWithLength:n * 9 * sizeof(float)
                                    options:MTLResourceStorageModeShared];
  assert(root_rot_buf != nil);
}

void MetalTraceBackend::Impl::EnsureContBuffer(int slot) {
  if (out_cap <= cont_capacity[slot]) {
    return;
  }
  cont_capacity[slot] = out_cap;
  cont_d[slot]  = [device newBufferWithLength:out_cap * 3 * sizeof(float)
                                      options:MTLResourceStorageModeShared];
  assert(cont_d[slot] != nil);
  cont_p[slot]  = [device newBufferWithLength:out_cap * 3 * sizeof(float)
                                      options:MTLResourceStorageModeShared];
  assert(cont_p[slot] != nil);
  cont_w[slot]  = [device newBufferWithLength:out_cap * sizeof(float)
                                      options:MTLResourceStorageModeShared];
  assert(cont_w[slot] != nil);
  cont_tf[slot] = [device newBufferWithLength:out_cap * sizeof(uint16_t)
                                      options:MTLResourceStorageModeShared];
  assert(cont_tf[slot] != nil);
  // Cont metadata (scrum-258.3 Step 1): sized in lock-step with cont_d/p/w/tf.
  // face_seq_cap_ MUST be set by BeginSession before TraceLayer reaches here.
  assert(face_seq_cap_ > 0u && "BeginSession must set face_seq_cap_ before EnsureContBuffer");
  cont_crystal_id_buf[slot] = [device newBufferWithLength:out_cap * sizeof(uint16_t)
                                                  options:MTLResourceStorageModeShared];
  assert(cont_crystal_id_buf[slot] != nil);
  cont_face_seq_len_buf[slot] = [device newBufferWithLength:out_cap * sizeof(uint8_t)
                                                    options:MTLResourceStorageModeShared];
  assert(cont_face_seq_len_buf[slot] != nil);
  cont_face_seq_data_buf[slot] =
      [device newBufferWithLength:out_cap * static_cast<size_t>(face_seq_cap_) * sizeof(uint8_t)
                          options:MTLResourceStorageModeShared];
  assert(cont_face_seq_data_buf[slot] != nil);
}

void MetalTraceBackend::Impl::EnsureRecSink(size_t n) {
  if (n <= rec_sink_capacity) {
    return;
  }
  rec_sink_capacity = n;
  rec_sink_buf = [device newBufferWithLength:n * sizeof(float)
                                     options:MTLResourceStorageModeShared];
  assert(rec_sink_buf != nil);
}

// Exit seam (scrum-258.1): grow-only buffer-egress storage + atomic slot.
// Sized to ComputeOutCap(total_ray_num, max_hits) — same fan-out bound the
// kernel already uses for continuation buffers, so single-MS exits cannot
// outnumber it. Multi-MS per-layer sizing is owned by 258.3.
void MetalTraceBackend::Impl::EnsureExitBuffers(size_t cap) {
  if (exit_slot_buf == nil) {
    exit_slot_buf = [device newBufferWithLength:sizeof(uint32_t)
                                        options:MTLResourceStorageModeShared];
    assert(exit_slot_buf != nil);
  }
  if (cap <= exit_ray_capacity) {
    return;
  }
  exit_ray_capacity = cap;
  exit_ray_d_buf = [device newBufferWithLength:cap * 3 * sizeof(float)
                                       options:MTLResourceStorageModeShared];
  assert(exit_ray_d_buf != nil);
  exit_ray_w_buf = [device newBufferWithLength:cap * sizeof(float)
                                       options:MTLResourceStorageModeShared];
  assert(exit_ray_w_buf != nil);
  // Exit metadata (scrum-258.2). face_seq_cap_ MUST be set by BeginSession
  // before TraceLayer reaches this point; assert defends against future
  // reorderings that would size buffer(23) to zero.
  assert(face_seq_cap_ > 0u && "BeginSession must set face_seq_cap_ before EnsureExitBuffers");
  exit_crystal_id_buf = [device newBufferWithLength:cap * sizeof(uint16_t)
                                            options:MTLResourceStorageModeShared];
  assert(exit_crystal_id_buf != nil);
  exit_face_seq_len_buf = [device newBufferWithLength:cap * sizeof(uint8_t)
                                              options:MTLResourceStorageModeShared];
  assert(exit_face_seq_len_buf != nil);
  exit_face_seq_data_buf =
      [device newBufferWithLength:cap * static_cast<size_t>(face_seq_cap_) * sizeof(uint8_t)
                          options:MTLResourceStorageModeShared];
  assert(exit_face_seq_data_buf != nil);
}

void MetalTraceBackend::Impl::UploadCrystal(const Crystal& crystal) {
  size_t poly_cnt = crystal.PolygonFaceCount();
  EnsurePolyBuffers(poly_cnt);

  std::memcpy([poly_n_buf contents], crystal.GetPolygonFaceNormal(),
              poly_cnt * 3 * sizeof(float));
  std::memcpy([poly_d_buf contents], crystal.GetPolygonFaceDist(),
              poly_cnt * sizeof(float));

  // Centroid per polygon face (mean of the polygon-triangle vertices) — same
  // formula as the explore spike (metal_full.mm / metal_ms.mm).
  const int* poly_tri = crystal.GetPolygonFaceTriId();
  const float* tvtx   = crystal.GetTriangleVtx();
  auto* centroid_ptr  = static_cast<float*>([centroid_buf contents]);
  for (size_t f = 0; f < poly_cnt; f++) {
    const float* v = tvtx + static_cast<size_t>(poly_tri[f]) * 9;
    for (int k = 0; k < 3; k++) {
      centroid_ptr[f * 3 + k] = (v[0 * 3 + k] + v[1 * 3 + k] + v[2 * 3 + k]) / 3.0f;
    }
  }
}

void MetalTraceBackend::Impl::ResolveLayerCrystalForCi(const ScatteringSetting& setting,
                                                        bool use_host,
                                                        const HostRayBatch& host_batch) {
  if (use_host && host_batch.crystal != nullptr) {
    current_crystal = *host_batch.crystal;
    current_n_idx = host_batch.refractive_index;
  } else {
    current_crystal = MakeCrystal(rng, setting.crystal_.param_);
    current_n_idx = current_crystal.GetRefractiveIndex(spec.wl.wl_);
  }
  have_crystal = true;
  UploadCrystal(current_crystal);
}

size_t MetalTraceBackend::Impl::GenerateFirstLayerRootsForCi(const ScatteringSetting& setting,
                                                              size_t ci, size_t crystal_ray_num) {
  const AxisDistribution& crystal_axis = setting.crystal_.axis_;

  RayBuffer workspace[2]{};
  workspace[0].Reset(crystal_ray_num);
  workspace[1].Reset(crystal_ray_num);
  RayBuffer all_data = AllocateAllData(*spec.scene, crystal_ray_num);

  InitRayFirstMs(rng, spec.scene->light_source_.param_, spec.wl, crystal_ray_num,
                 current_crystal, /*crystal_id=*/ci, crystal_axis,
                 workspace, all_data);

  // EnsureRootBuffers is called by TraceLayer at the top of each layer with
  // total_ray_num so per-ci root buffers are guaranteed >= crystal_ray_num.
  size_t n = workspace[0].size_;
  auto* d_ptr   = static_cast<float*>([root_d_buf contents]);
  auto* p_ptr   = static_cast<float*>([root_p_buf contents]);
  auto* w_ptr   = static_cast<float*>([root_w_buf contents]);
  auto* tf_ptr  = static_cast<uint16_t*>([root_tf_buf contents]);
  auto* rot_ptr = static_cast<float*>([root_rot_buf contents]);

  for (size_t i = 0; i < n; i++) {
    const RaySeg& r = workspace[0][i];
    d_ptr[i * 3 + 0] = r.d_[0];
    d_ptr[i * 3 + 1] = r.d_[1];
    d_ptr[i * 3 + 2] = r.d_[2];
    p_ptr[i * 3 + 0] = r.p_[0];
    p_ptr[i * 3 + 1] = r.p_[1];
    p_ptr[i * 3 + 2] = r.p_[2];
    w_ptr[i] = r.w_;
    tf_ptr[i] = static_cast<uint16_t>(r.to_face_);
    // Per-ray crystal->world rotation (row-major mat_, same layout the kernel
    // applies as mat*v). InitRayFirstMs sampled this orientation and applied
    // its inverse to bring d_ into crystal-local space for tracing; the kernel
    // re-applies the forward rotation before projection (invariant 6).
    std::memcpy(rot_ptr + i * 9, r.crystal_rot_.GetMat(), 9 * sizeof(float));
  }
  return n;
}

size_t MetalTraceBackend::Impl::CopyContSliceToRootBuf(const ScatteringSetting& setting,
                                                       size_t ci_start, size_t ci_n, int in_slot) {
  // Inter-layer host hop. Two responsibilities:
  //
  //  (A) Frame transit (DESIGN D3 / mirrors CPU InitRayOtherMs).
  //      Step 1 made the kernel export continuation DIRECTIONS in WORLD space.
  //      For the next layer — possibly a different crystal — we (per
  //      InitRayOtherMs, simulator.cpp:199-210):
  //        1) resample a fresh per-ray crystal orientation (InitRay_rot),
  //        2) rotate the world-space direction into the NEW crystal-local frame
  //           (ApplyInverse), and
  //        3) resample the entry point + hit face on the NEW crystal
  //           (InitRay_p_fid),
  //      then upload d/p/tf + the new per-ray rotation matrix.
  //
  //  (B) Per-layer filter + prob (scrum-258.3 Step 4). Mirrors legacy
  //      simulator.cpp:425 CollectData:
  //        filter-fail              → drop
  //        filter-pass && rng<prob  → continue (write to root_*_buf)
  //        filter-pass && rng>=prob → mid-layer exit (push into
  //                                   accumulated_mid_exits_, drained by
  //                                   ReadbackExitRays)
  //      Filter input comes from hop_crystals_[in_slot] / hop_axis_dists_ /
  //      hop_filter_configs_ (populated by the PREVIOUS layer's ci loop);
  //      face sequence + crystal_id come from cont_face_seq_*_buf[in_slot] /
  //      cont_crystal_id_buf[in_slot] (populated by the previous layer's
  //      kernel ms_mode==1 branch).
  //
  // The function returns the actual continuation count after filter+prob; the
  // caller substitutes this for the previous unconditional `ci_n`. Mid-layer
  // exits and discards do NOT advance the root-buffer write index.
  //
  // `current_crystal` is the THIS-layer crystal, already resolved by
  // ResolveLayerCrystalForCi before this call. `rng` is the session RNG member
  // (seeded in BeginSession), reused exactly as GenerateFirstLayerRootsForCi /
  // ResolveLayerCrystalForCi do.
  const float*    src_d  = static_cast<const float*>([cont_d[in_slot] contents]) + ci_start * 3;
  const float*    src_w  = static_cast<const float*>([cont_w[in_slot] contents]) + ci_start;
  const uint16_t* src_cid =
      static_cast<const uint16_t*>([cont_crystal_id_buf[in_slot] contents]) + ci_start;
  const uint8_t*  src_seq_len =
      static_cast<const uint8_t*>([cont_face_seq_len_buf[in_slot] contents]) + ci_start;
  const uint8_t*  src_seq_data =
      static_cast<const uint8_t*>([cont_face_seq_data_buf[in_slot] contents])
      + ci_start * static_cast<size_t>(face_seq_cap_);
  const size_t    seq_stride = static_cast<size_t>(face_seq_cap_);

  // Per-ci filter cache. Same crystal_id (src_cid[i]) repeats across the
  // ci slice, so build FilterSpec once per crystal and reuse — avoids
  // reconstructing the spec for every ray (review Suggestion 1).
  const size_t crystal_cnt = hop_crystals_[in_slot].size();
  std::vector<std::unique_ptr<FilterSpec>> spec_per_ci(crystal_cnt);
  for (size_t k = 0; k < crystal_cnt; k++) {
    spec_per_ci[k] = FilterSpec::Create(hop_filter_configs_[in_slot][k],
                                         hop_crystals_[in_slot][k],
                                         hop_axis_dists_[in_slot][k]);
  }
  const float src_ms_prob = hop_ms_prob_[in_slot];
  const uint8_t src_ms_layer_idx = hop_ms_layer_idx_[in_slot];

  // First pass: filter + prob → partition into (continue) / (mid-exit) /
  // (drop). Continuation rays are staged into a transient workspace and then
  // run through the frame-transit InitRay_rot / InitRay_p_fid chain in bulk.
  RayBuffer workspace[2]{};
  workspace[0].Reset(ci_n);
  workspace[0].size_ = 0;
  size_t n_continue = 0;
  for (size_t i = 0; i < ci_n; i++) {
    uint16_t crystal_id = src_cid[i];
    assert(crystal_id < crystal_cnt && "cont crystal_id out of range — hop_crystals_ undersized");
    if (crystal_id >= crystal_cnt) {
      continue;  // safety: skip stray records
    }

    // Build {RaySeg, RaypathRecorder} for filter Check (world-space d/p, like
    // legacy CollectData post-Apply). to_face_ = kInvalidId because this is an
    // outgoing candidate from the previous layer (the kernel signaled "no
    // continuation polygon hit" by writing to the cont buffer in ms_mode==1).
    RaySeg r{};
    r.d_[0] = src_d[i * 3 + 0];
    r.d_[1] = src_d[i * 3 + 1];
    r.d_[2] = src_d[i * 3 + 2];
    r.w_ = src_w[i];
    r.to_face_ = kInvalidId;
    r.is_continue_ = false;
    // Mirror legacy InitRay_other_info: crystal_config_id_ comes from the
    // producing crystal's config_id_. MakeCrystal leaves config_id_ at
    // kInvalidId (crystal.hpp:287) unless explicitly assigned; this matches
    // legacy semantics so CrystalSpec filter behaves the same on both paths.
    const Crystal& src_crystal = hop_crystals_[in_slot][crystal_id];
    r.crystal_config_id_ = src_crystal.config_id_;

    RaypathRecorder rec{};
    uint8_t seq_len = src_seq_len[i];
    if (seq_len > RaypathRecorder::kInlineCap) {
      seq_len = RaypathRecorder::kInlineCap;
    }
    rec.size_ = seq_len;
    rec.overflow_idx_ = RaypathRecorder::kNoOverflow;  // 0xFFFFu, inline-only
    std::memcpy(rec.data_, src_seq_data + i * seq_stride, seq_len);

    const FilterSpec* spec = spec_per_ci[crystal_id].get();
    bool filter_pass = (spec == nullptr) || spec->Check(r, rec, nullptr);
    if (!filter_pass) {
      continue;  // discard
    }
    if (rng.GetUniform() < src_ms_prob) {
      // Continue: stage into workspace[0] for the frame-transit chain below.
      // workspace[0].size_ is the packed write index — discards/mid-exits do
      // NOT advance it, so the continuation slice stays contiguous.
      RaySeg& wr = workspace[0][workspace[0].size_];
      wr = r;  // copies d/w/to_face/is_continue/crystal_config_id
      workspace[0].size_++;
      n_continue++;
    } else {
      // Mid-layer exit: emit as an ExitRayRecord tagged with the PRODUCING
      // layer's ms_idx (hop_ms_layer_idx_[in_slot]). ReadbackExitRays drains
      // and merges with last-layer exits at session end.
      ExitRayRecord mid_rec{};
      mid_rec.dir[0] = r.d_[0];
      mid_rec.dir[1] = r.d_[1];
      mid_rec.dir[2] = r.d_[2];
      mid_rec.weight = r.w_;
      mid_rec.crystal_id = crystal_id;
      mid_rec.ms_layer_idx = src_ms_layer_idx;
      mid_rec.path.size_ = seq_len;
      std::memcpy(mid_rec.path.data_, rec.data_, seq_len);
      accumulated_mid_exits_.push_back(mid_rec);
    }
  }

  if (n_continue == 0) {
    return 0;
  }

  // Frame-transit chain on the packed continuation slice (n_continue rays).
  InitRay_rot(rng, setting.crystal_.axis_, workspace);
  for (size_t i = 0; i < n_continue; i++) {
    workspace[0][i].crystal_rot_.ApplyInverse(workspace[0][i].d_);
  }
  InitRay_p_fid(current_crystal, &workspace[0]);

  auto* d_ptr   = static_cast<float*>([root_d_buf contents]);
  auto* p_ptr   = static_cast<float*>([root_p_buf contents]);
  auto* w_ptr   = static_cast<float*>([root_w_buf contents]);
  auto* tf_ptr  = static_cast<uint16_t*>([root_tf_buf contents]);
  auto* rot_ptr = static_cast<float*>([root_rot_buf contents]);
  for (size_t i = 0; i < n_continue; i++) {
    const RaySeg& r = workspace[0][i];
    d_ptr[i * 3 + 0] = r.d_[0];
    d_ptr[i * 3 + 1] = r.d_[1];
    d_ptr[i * 3 + 2] = r.d_[2];
    p_ptr[i * 3 + 0] = r.p_[0];
    p_ptr[i * 3 + 1] = r.p_[1];
    p_ptr[i * 3 + 2] = r.p_[2];
    w_ptr[i] = r.w_;
    tf_ptr[i] = static_cast<uint16_t>(r.to_face_);
    std::memcpy(rot_ptr + i * 9, r.crystal_rot_.GetMat(), 9 * sizeof(float));
  }
  return n_continue;
}

void MetalTraceBackend::Impl::DispatchLayer(size_t num_rays,
                                            id<MTLBuffer> r_d, id<MTLBuffer> r_p,
                                            id<MTLBuffer> r_w, id<MTLBuffer> r_tf,
                                            uint32_t ms_mode, int out_slot,
                                            uint32_t counter_init,
                                            uint32_t crystal_id, uint32_t ms_layer_idx) {
  KernelParams params{};
  params.n_idx    = current_n_idx;
  params.max_hits = static_cast<uint32_t>(spec.scene->max_hits_);
  params.poly_cnt = static_cast<uint32_t>(current_crystal.PolygonFaceCount());
  params.num_rays = static_cast<uint32_t>(num_rays);
  params.img_w    = static_cast<uint32_t>(width);
  params.img_h    = static_cast<uint32_t>(height);
  params.az0      = az0;
  params.cie_x    = cie_x;
  params.cie_y    = cie_y;
  params.cie_z    = cie_z;
  params.ms_mode  = ms_mode;
  params.out_cap  = static_cast<uint32_t>(out_cap);
  params.proj_type  = proj_type;
  params.r_scale    = r_scale;
  params.max_abs_dz = max_abs_dz;
  params.exit_cap   = static_cast<uint32_t>(exit_ray_capacity);
  params.crystal_id   = crystal_id;
  params.face_seq_cap = face_seq_cap_;
  params.ms_layer_idx = ms_layer_idx;
  // Defensive sanity bounds — silent host/MSL field reorder would set
  // crystal_id to a different field's value (max_hits ~8, face_seq_cap ~8,
  // ms_layer_idx ~0). The bounds below are narrow enough to fire for any
  // reorder that puts a non-crystal field into the crystal_id slot.
  assert(crystal_id < kMaxCrystalNum && "crystal_id out of range — check KernelParams layout");
  assert(face_seq_cap_ > 0u && face_seq_cap_ <= ExitFaceSeq::kCap &&
         "face_seq_cap_ out of range — check BeginSession + KernelParams layout");

  EnsureRecSink(num_rays);
  EnsureContBuffer(out_slot);
  // Multi-ci append semantics: counter_init carries the running offset of
  // already-written continuation rays from previous ci dispatches; the
  // kernel's atomic_fetch_add resumes from there. ci=0 always passes 0
  // (equivalent to the previous unconditional reset).
  // last_stats is NOT reset here — TraceLayer zeroes it once before the
  // ci loop, and each DispatchLayer accumulates via += below.
  if (counter_buf == nil) {
    counter_buf = [device newBufferWithLength:sizeof(uint32_t)
                                      options:MTLResourceStorageModeShared];
  }
  *static_cast<uint32_t*>([counter_buf contents]) = counter_init;
  // exit_count / exit_w_sum are atomic accumulators populated by the kernel.
  // Reset before each dispatch and add the readback into last_stats so the
  // per-ci contributions sum correctly.
  if (exit_count_buf == nil) {
    exit_count_buf = [device newBufferWithLength:sizeof(uint32_t)
                                         options:MTLResourceStorageModeShared];
    assert(exit_count_buf != nil);
  }
  if (exit_w_sum_buf == nil) {
    exit_w_sum_buf = [device newBufferWithLength:sizeof(float)
                                         options:MTLResourceStorageModeShared];
    assert(exit_w_sum_buf != nil);
  }
  *static_cast<uint32_t*>([exit_count_buf contents]) = 0u;
  *static_cast<float*>([exit_w_sum_buf contents]) = 0.0f;

  id<MTLCommandBuffer> cb = [queue commandBuffer];
  id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
  [enc setComputePipelineState:pso];
  [enc setBuffer:r_d            offset:0 atIndex:0];
  [enc setBuffer:r_p            offset:0 atIndex:1];
  [enc setBuffer:r_w            offset:0 atIndex:2];
  [enc setBuffer:r_tf           offset:0 atIndex:3];
  [enc setBuffer:poly_n_buf     offset:0 atIndex:4];
  [enc setBuffer:poly_d_buf     offset:0 atIndex:5];
  [enc setBuffer:centroid_buf   offset:0 atIndex:6];
  [enc setBytes:&params length:sizeof(KernelParams) atIndex:7];
  [enc setBuffer:xyz_image      offset:0 atIndex:8];
  [enc setBuffer:cont_d[out_slot]  offset:0 atIndex:9];
  [enc setBuffer:cont_p[out_slot]  offset:0 atIndex:10];
  [enc setBuffer:cont_w[out_slot]  offset:0 atIndex:11];
  [enc setBuffer:cont_tf[out_slot] offset:0 atIndex:12];
  [enc setBuffer:counter_buf    offset:0 atIndex:13];
  [enc setBuffer:rec_sink_buf   offset:0 atIndex:14];
  [enc setBuffer:exit_count_buf offset:0 atIndex:15];
  [enc setBuffer:exit_w_sum_buf offset:0 atIndex:16];
  [enc setBuffer:root_rot_buf   offset:0 atIndex:17];
  // Exit seam (scrum-258.1) — buffer-egress {dir, weight} + atomic slot.
  // Bound unconditionally; the kernel always writes (no env gate).
  [enc setBuffer:exit_ray_d_buf offset:0 atIndex:18];
  [enc setBuffer:exit_ray_w_buf offset:0 atIndex:19];
  [enc setBuffer:exit_slot_buf  offset:0 atIndex:20];
  // Exit metadata (scrum-258.2) — bind in lock-step with exit_ray_d/w; the
  // kernel writes to all three when exit_slot is acquired.
  [enc setBuffer:exit_crystal_id_buf     offset:0 atIndex:21];
  [enc setBuffer:exit_face_seq_len_buf   offset:0 atIndex:22];
  [enc setBuffer:exit_face_seq_data_buf  offset:0 atIndex:23];
  // Cont metadata (scrum-258.3 Step 2): bound for both ms_mode==0 and
  // ms_mode==1 dispatches. ms_mode==0 doesn't take the out_d/p/w/tf write
  // branch, so cont_crystal_id_buf[out_slot] etc. are never written through —
  // but Metal still requires non-nil buffer bindings. The grow-only
  // EnsureContBuffer(out_slot) above guarantees these are sized (>= 1 elem)
  // before this dispatch.
  [enc setBuffer:cont_crystal_id_buf[out_slot]    offset:0 atIndex:24];
  [enc setBuffer:cont_face_seq_len_buf[out_slot]  offset:0 atIndex:25];
  [enc setBuffer:cont_face_seq_data_buf[out_slot] offset:0 atIndex:26];

  NSUInteger tg = std::min<NSUInteger>(256, pso.maxTotalThreadsPerThreadgroup);
  [enc dispatchThreads:MTLSizeMake(num_rays, 1, 1)
   threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
  [enc endEncoding];
  [cb commit];
  [cb waitUntilCompleted];

  if (cb.status != MTLCommandBufferStatusCompleted) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: GPU dispatch failed: {}",
               cb.error.localizedDescription.UTF8String);
    assert(false && "MetalTraceBackend: GPU dispatch failed");
  }

  uint32_t produced = *static_cast<uint32_t*>([counter_buf contents]);
  if (produced > out_cap) {
    ILOG_ERROR(EffectiveLogger(logger_),
               "MetalTraceBackend: continuation overflow produced={} out_cap={}",
               produced, out_cap);
    assert(false && "MetalTraceBackend: continuation overflow");
    // Release-build clamp: assert is compiled out, so clamp produced to out_cap
    // to prevent the next layer reading cont_* buffers out of bounds on the GPU.
    produced = static_cast<uint32_t>(out_cap);
  }
  if (produced > max_produced) {
    max_produced = produced;
  }
  cont_counts[out_slot] = produced;

  // Readback exit-ray stats for the parity harness. Both atomics live in
  // Shared storage, so a plain host load after waitUntilCompleted is
  // sufficient (no blit encoder required on unified memory). Accumulate
  // into last_stats so the multi-ci loop's contributions sum correctly;
  // TraceLayer is responsible for zeroing last_stats before the ci loop.
  last_stats.exit_count += *static_cast<uint32_t*>([exit_count_buf contents]);
  last_stats.exit_w_sum += *static_cast<float*>([exit_w_sum_buf contents]);
}

void MetalTraceBackend::Impl::Reset() {
  if (max_produced > 0) {
    ILOG_DEBUG(EffectiveLogger(logger_),
               "MetalTraceBackend: session ended — max continuation produced={} (out_cap={})",
               max_produced, out_cap);
  } else {
    ILOG_DEBUG(EffectiveLogger(logger_),
               "MetalTraceBackend: session ended — single-layer MS, no continuation");
  }
  in_session = false;
  ms_idx = 0;
  root_ray_count = 0;
  width = 0;
  height = 0;
  az0 = 0.0f;
  cie_x = cie_y = cie_z = 0.0f;
  proj_type = 0u;
  r_scale = 1.0f;
  max_abs_dz = 0.0f;
  have_crystal = false;
  current_n_idx = 0.0f;
  out_cap = 0;
  max_produced = 0;
  cont_counts[0] = cont_counts[1] = 0;
  last_stats = LayerStats{};
  // Exit seam (scrum-258.1): buffers + capacity persist across sessions
  // (grow-only); only the atomic slot needs reset for the next session,
  // which happens in TraceLayer on the first MS layer.
  // scrum-258.2: face_seq_cap_ is re-derived per BeginSession from
  // scene.max_hits_; clear to make a session-mid leak obvious.
  face_seq_cap_ = 0;
  // scrum-258.3: per-layer filter+prob state. hop_*_/last_layer_* are
  // populated by TraceLayer's ci loop in the next session; clear here so a
  // stale layer's settings cannot bleed across sessions if BeginSession
  // skips re-assigning a particular slot.
  for (int s = 0; s < 2; s++) {
    hop_crystals_[s].clear();
    hop_axis_dists_[s].clear();
    hop_filter_configs_[s].clear();
    hop_ms_prob_[s] = 0.0f;
    hop_ms_layer_idx_[s] = 0u;
  }
  last_layer_crystals_.clear();
  last_layer_axis_dists_.clear();
  last_layer_filter_configs_.clear();
  last_ms_prob_ = 0.0f;
  last_ms_layer_idx_ = 0u;
  accumulated_mid_exits_.clear();
  spec = SessionSpec{};
}


// ============================== MetalTraceBackend ============================

MetalTraceBackend::MetalTraceBackend(Logger* logger)
    : impl_(std::make_unique<Impl>()) {
  impl_->logger_ = logger;
}

MetalTraceBackend::~MetalTraceBackend() = default;

void MetalTraceBackend::BeginSession(const SessionSpec& spec) {
  assert(!impl_->in_session && "BeginSession called on an already-open session");
  assert(spec.scene != nullptr);
  assert(spec.render != nullptr);
  assert((spec.render->lens_.type_ == LensParam::kRectangular ||
          spec.render->lens_.type_ == LensParam::kDualFisheyeEqualArea) &&
         "MetalTraceBackend supports kRectangular or kDualFisheyeEqualArea lens");
  // az0-projection is only equivalent to RectangularProject at zenith view
  // (el=90°, ro=0°). Dual-fisheye-EA covers the full globe, view fields are
  // unused, so we only assert for the rectangular branch.
  if (spec.render->lens_.type_ == LensParam::kRectangular) {
    assert(std::abs(spec.render->view_.el_ - 90.0f) < 0.01f &&
           "MetalTraceBackend kRectangular requires zenith view (el≈90°)");
  }

  impl_->spec = spec;
  impl_->in_session = true;
  impl_->ms_idx = 0;
  impl_->root_ray_count = 0;
  impl_->width  = spec.render->resolution_[0];
  impl_->height = spec.render->resolution_[1];

  Rotation camera_rot = MakeCameraRotation(*spec.render);
  impl_->az0 = ComputeAz0(camera_rot);
  ComputeCmf(spec.wl.wl_, impl_->cie_x, impl_->cie_y, impl_->cie_z);

  // Projection routing — kernel reads proj_type/r_scale/max_abs_dz from
  // KernelParams (filled by DispatchLayer from these Impl fields).
  if (spec.render->lens_.type_ == LensParam::kDualFisheyeEqualArea) {
    impl_->proj_type  = 1u;
    impl_->max_abs_dz = spec.render->overlap_;
    impl_->r_scale    = projection::ComputeEARScale(spec.render->overlap_);
  } else {
    impl_->proj_type  = 0u;
    impl_->r_scale    = 1.0f;
    impl_->max_abs_dz = 0.0f;
  }

  if (spec.seed != 0) {
    impl_->rng.SetSeed(spec.seed);
    RandomNumberGenerator::GetInstance().SetSeed(spec.seed);
  }

  impl_->EnsureDevice();
  impl_->EnsurePso();
  impl_->EnsureImage(impl_->width, impl_->height);

  impl_->cont_counts[0] = 0;
  impl_->cont_counts[1] = 0;
  impl_->out_cap = 0;
  // scrum-258.3 Step 4: drain mid-layer exits accumulated by the previous
  // session (paranoia; Reset already clears it, but BeginSession is the
  // session entry and the canonical place to guarantee empty state).
  impl_->accumulated_mid_exits_.clear();
  // Exit metadata (scrum-258.2): per-slot stride of buffer(23). Real configs
  // have max_hits ≤ ExitFaceSeq::kCap (7-8 in practice), so the min() clamp
  // is defensive against a future scene bumping max_hits beyond 15. Set BEFORE
  // EnsureExitBuffers (called from TraceLayer's first-MS branch) so the device
  // allocation reflects the actual stride.
  impl_->face_seq_cap_ = std::min<uint32_t>(
      static_cast<uint32_t>(spec.scene->max_hits_), static_cast<uint32_t>(ExitFaceSeq::kCap));
}

LayerHandlePtr MetalTraceBackend::TraceLayer(const RootRaySource& roots) {
  assert(impl_->in_session && "TraceLayer outside session");
  assert(impl_->ms_idx < impl_->spec.scene->ms_.size() &&
         "TraceLayer beyond configured MS layers");
  @autoreleasepool {
    const auto& ms_info = impl_->spec.scene->ms_[impl_->ms_idx];
    assert(!ms_info.setting_.empty() && "MS layer has no scattering settings");
    bool first_ms = !roots.is_device;

    size_t total_ray_num = first_ms ? roots.host.count : roots.device.count;
    if (first_ms) {
      impl_->root_ray_count = total_ray_num;
      // Exit seam (scrum-258.1): size the session-level exit buffer once and
      // reset its atomic slot. Capacity = ComputeOutCap (same fan-out upper
      // bound the continuation buffers use). Single-MS guarantees this is
      // SINGLE-MS ONLY: exit_slot accumulates across all layers; capacity is
      // sized for the first (and only) MS layer here. Multi-MS per-session
      // capacity re-sizing is owned by scrum-258.3.
      size_t exit_cap = ComputeOutCap(total_ray_num, impl_->spec.scene->max_hits_);
      impl_->EnsureExitBuffers(exit_cap);  // SINGLE-MS ONLY (see above)
      *static_cast<uint32_t*>([impl_->exit_slot_buf contents]) = 0u;
    }
    // Recalculate out_cap per layer so each layer's continuation buffer is
    // sized to the actual fan-out from that layer's input count. This matters
    // for ≥3 MS layers where the fan-out from layer N can exceed the root-count
    // bound used by layer 0.
    impl_->out_cap = ComputeOutCap(total_ray_num, impl_->spec.scene->max_hits_);
    if (total_ray_num == 0) {
      return std::make_unique<MetalLayerHandle>(0u, LayerStats{});
    }

    // Partition the layer's rays across crystal populations. Matches
    // simulator.cpp:572-586 and CpuTraceBackend::TraceLayer.
    size_t crystal_cnt = ms_info.setting_.size();
    std::vector<float> proportions;
    proportions.reserve(crystal_cnt);
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      proportions.push_back(ms_info.setting_[ci].crystal_proportion_);
    }
    std::vector<double> carry(crystal_cnt, 0.0);
    auto crystal_ray_num = PartitionCrystalRayNum(proportions, total_ray_num, carry);

    // Pre-allocate root_*_buf to the full per-layer total. Each per-ci
    // dispatch only uses crystal_ray_num[ci] of it; sizing for the upper
    // bound once avoids repeated grow-only allocations inside the ci loop
    // and costs nothing on M2 unified memory.
    impl_->EnsureRootBuffers(total_ray_num);

    bool last_layer = (impl_->ms_idx + 1u == impl_->spec.scene->ms_.size());
    uint32_t ms_mode = last_layer ? 0u : 1u;
    int out_slot = static_cast<int>(impl_->ms_idx & 1u);
    int in_slot = static_cast<int>((impl_->ms_idx - 1) & 1u);  // only used when !first_ms

    // Reset per-layer counters BEFORE the ci loop: counter_buf is written
    // by each DispatchLayer (counter_init = previous cont_counts), and
    // last_stats accumulates via += inside DispatchLayer. Both must start
    // at zero for the layer.
    impl_->last_stats = LayerStats{};
    impl_->cont_counts[out_slot] = 0;

    // scrum-258.3 Step 3: per-layer filter+prob hop state. Non-final layers
    // populate hop_*_[out_slot] (read back by the NEXT layer's
    // CopyContSliceToRootBuf via the ping-pong slot indirection); the final
    // layer populates last_layer_* (read by ReadbackExitRays).
    if (last_layer) {
      impl_->last_layer_crystals_.resize(crystal_cnt);
      impl_->last_layer_axis_dists_.resize(crystal_cnt);
      impl_->last_layer_filter_configs_.resize(crystal_cnt);
      impl_->last_ms_prob_ = ms_info.prob_;
      impl_->last_ms_layer_idx_ = static_cast<uint8_t>(impl_->ms_idx);
    } else {
      impl_->hop_crystals_[out_slot].resize(crystal_cnt);
      impl_->hop_axis_dists_[out_slot].resize(crystal_cnt);
      impl_->hop_filter_configs_[out_slot].resize(crystal_cnt);
      impl_->hop_ms_prob_[out_slot] = ms_info.prob_;
      // ms_layer_idx records the layer that PRODUCED this cont, i.e. impl_->ms_idx
      // (the current layer). The next layer reads it via in_slot and tags any
      // mid-exit emitted there with the producing layer's index. ReadbackExitRays
      // then merges those mid-exits with last-layer exits without a -1 adjustment.
      impl_->hop_ms_layer_idx_[out_slot] = static_cast<uint8_t>(impl_->ms_idx);
    }

    size_t ci_start = 0;  // running offset into the previous-layer
                          // continuation buffer (non-first_ms only)
    for (size_t ci = 0; ci < crystal_cnt; ci++) {
      size_t ci_n = crystal_ray_num[ci];
      if (ci_n == 0) {
        continue;
      }
      const auto& setting = ms_info.setting_[ci];
      bool use_host = first_ms && ci == 0 && roots.host.crystal != nullptr;
      impl_->ResolveLayerCrystalForCi(setting, use_host,
                                       first_ms ? roots.host : HostRayBatch{});

      // scrum-258.3 Step 3: store per-ci crystal + axis + filter for use by
      // the host hop (CopyContSliceToRootBuf on the next layer, or
      // ReadbackExitRays for the final layer). current_crystal was just
      // resolved by ResolveLayerCrystalForCi.
      if (last_layer) {
        impl_->last_layer_crystals_[ci] = impl_->current_crystal;
        impl_->last_layer_axis_dists_[ci] = setting.crystal_.axis_;
        impl_->last_layer_filter_configs_[ci] = setting.filter_;
      } else {
        impl_->hop_crystals_[out_slot][ci] = impl_->current_crystal;
        impl_->hop_axis_dists_[out_slot][ci] = setting.crystal_.axis_;
        impl_->hop_filter_configs_[out_slot][ci] = setting.filter_;
      }

      size_t in_count = 0;
      if (first_ms) {
        in_count = impl_->GenerateFirstLayerRootsForCi(setting, ci, ci_n);
      } else {
        // Stage this ci's slice of the prior continuation buffer into the
        // root buffers so the kernel reads a contiguous, aligned input.
        // scrum-258.3 Step 4: CopyContSliceToRootBuf now applies per-ray
        // filter + prob and returns the actual continuation count. Skip the
        // DispatchLayer if filter+prob filtered everything (mirrors the
        // existing `if (ci_n == 0) continue` guard above).
        in_count = impl_->CopyContSliceToRootBuf(setting, ci_start, ci_n, in_slot);
        ci_start += ci_n;
        if (in_count == 0) {
          continue;
        }
      }

      // counter_init = current cumulative write offset; ci=0 starts at 0,
      // each subsequent ci resumes where the previous one's atomic
      // counter left off (already read back into cont_counts[out_slot]
      // by the previous DispatchLayer via waitUntilCompleted).
      uint32_t counter_init = static_cast<uint32_t>(impl_->cont_counts[out_slot]);
      impl_->DispatchLayer(in_count,
                           impl_->root_d_buf, impl_->root_p_buf,
                           impl_->root_w_buf, impl_->root_tf_buf,
                           ms_mode, out_slot, counter_init,
                           static_cast<uint32_t>(ci),
                           static_cast<uint32_t>(impl_->ms_idx));
      // ci_start was incremented above inside the !first_ms branch (before
      // the in_count==0 continue), so the next ci reads the correct slice
      // even if this ci's filter+prob dropped everything.
    }

    size_t produced = last_layer ? 0u : impl_->cont_counts[out_slot];
    return std::make_unique<MetalLayerHandle>(produced, impl_->last_stats);
  }
}

RootRaySource MetalTraceBackend::Recombine(LayerHandlePtr handle, const RecombineSpec& spec) {
  assert(impl_->in_session);
  assert(handle != nullptr);
  // v1 does not implement device-side shuffle; integration callers must opt
  // out explicitly (RecombineSpec.shuffle = false). The default-true value is
  // safe for the CPU backend only.
  assert(!spec.shuffle &&
         "MetalTraceBackend v1 does not implement device-side shuffle");
  size_t count = handle->ContinuationCount();
  (void)spec;

  impl_->ms_idx++;

  DeviceRayBatch dev;
  // backend_ptr is an opaque token (we use `this` so callers can sanity-check
  // it against the producing backend; the actual buffers live in Impl::cont_*).
  dev.backend_ptr = this;
  dev.count = count;
  return RootRaySource::FromDevice(dev);
}

void MetalTraceBackend::ReadbackImage(XyzImageData& out) {
  assert(impl_->in_session);
  assert(out.data != nullptr);
  assert(out.width == impl_->width && out.height == impl_->height &&
         "XyzImageData dimensions must match BeginSession resolution");
  size_t pix = static_cast<size_t>(impl_->width) * static_cast<size_t>(impl_->height);
  std::memcpy(out.data, [impl_->xyz_image contents], pix * 3 * sizeof(float));
}

// Exit seam (scrum-258.1/258.2/258.3): assemble rich ExitRayRecords.
//
// 258.3 makes per-layer filter+prob the seam's responsibility, mirroring legacy
// simulator.cpp:425 CollectData per-layer:
//   final-layer exits   : filter + prob applied here in ReadbackExitRays (the
//                         GPU kernel exports every geometric exit, this gate
//                         drops filter-fails and the rng<prob "would continue"
//                         fraction).
//   mid-layer exits     : already filtered + prob-gated by
//                         CopyContSliceToRootBuf; accumulated into
//                         impl_->accumulated_mid_exits_ across non-final layers
//                         and drained + appended here.
// Overflow (produced > capacity) on the final-layer exit ring is logged and
// clamped; ComputeOutCap is the same fan-out bound the continuation buffers
// use, so structurally rare.
size_t MetalTraceBackend::ReadbackExitRays(std::vector<ExitRayRecord>& out) {
  assert(impl_->in_session);
  out.clear();
  if (impl_->exit_slot_buf == nil) {
    return 0;
  }
  uint32_t produced = *static_cast<uint32_t*>([impl_->exit_slot_buf contents]);
  size_t count = std::min<size_t>(produced, impl_->exit_ray_capacity);
  if (produced > impl_->exit_ray_capacity) {
    ILOG_ERROR(EffectiveLogger(impl_->logger_),
               "MetalTraceBackend: exit-ray overflow produced={} exit_cap={} (clamped)",
               produced, impl_->exit_ray_capacity);
  }
  const auto* d_ptr        = static_cast<const float*>([impl_->exit_ray_d_buf contents]);
  const auto* w_ptr        = static_cast<const float*>([impl_->exit_ray_w_buf contents]);
  const auto* cid_ptr      = static_cast<const uint16_t*>([impl_->exit_crystal_id_buf contents]);
  const auto* seq_len_ptr  = static_cast<const uint8_t*>([impl_->exit_face_seq_len_buf contents]);
  const auto* seq_data_ptr = static_cast<const uint8_t*>([impl_->exit_face_seq_data_buf contents]);
  const size_t stride      = impl_->face_seq_cap_;

  // Per-ci FilterSpec cache for the final layer. Same crystal_id repeats
  // across exits; build each spec once.
  const size_t last_cnt = impl_->last_layer_crystals_.size();
  std::vector<std::unique_ptr<FilterSpec>> spec_per_ci(last_cnt);
  for (size_t k = 0; k < last_cnt; k++) {
    spec_per_ci[k] = FilterSpec::Create(impl_->last_layer_filter_configs_[k],
                                         impl_->last_layer_crystals_[k],
                                         impl_->last_layer_axis_dists_[k]);
  }
  const float    final_prob      = impl_->last_ms_prob_;
  const uint8_t  final_ms_layer  = impl_->last_ms_layer_idx_;

  out.reserve(count + impl_->accumulated_mid_exits_.size());
  for (size_t i = 0; i < count; i++) {
    uint16_t crystal_id = cid_ptr[i];
    if (crystal_id >= last_cnt) {
      continue;  // safety: skip stray records from a malformed kernel run
    }

    // Reconstruct {RaySeg, RaypathRecorder} for filter Check, mirroring
    // CollectData post-Apply (d/p in world space, to_face_ = kInvalidId).
    RaySeg r{};
    r.d_[0] = d_ptr[i * 3 + 0];
    r.d_[1] = d_ptr[i * 3 + 1];
    r.d_[2] = d_ptr[i * 3 + 2];
    r.w_ = w_ptr[i];
    r.to_face_ = kInvalidId;
    r.is_continue_ = false;
    const Crystal& src_crystal = impl_->last_layer_crystals_[crystal_id];
    r.crystal_config_id_ = src_crystal.config_id_;

    RaypathRecorder rec{};
    uint8_t seq_len = std::min<uint8_t>(seq_len_ptr[i], ExitFaceSeq::kCap);
    rec.size_ = seq_len;
    rec.overflow_idx_ = RaypathRecorder::kNoOverflow;
    std::memcpy(rec.data_, seq_data_ptr + i * stride, seq_len);

    const FilterSpec* spec = spec_per_ci[crystal_id].get();
    bool filter_pass = (spec == nullptr) || spec->Check(r, rec, nullptr);
    if (!filter_pass) {
      continue;
    }
    // Legacy mirror: rng<prob → "would continue", but there is no next layer,
    // so the ray is dropped (no outgoing emission). prob=1.0 therefore produces
    // zero final-layer exits, which matches simulator.cpp:444 behaviour.
    if (impl_->rng.GetUniform() < final_prob) {
      continue;
    }

    ExitRayRecord erec{};
    erec.dir[0] = r.d_[0];
    erec.dir[1] = r.d_[1];
    erec.dir[2] = r.d_[2];
    erec.weight = r.w_;
    erec.crystal_id = crystal_id;
    erec.ms_layer_idx = final_ms_layer;
    erec.path.size_ = seq_len;
    std::memcpy(erec.path.data_, rec.data_, seq_len);
    out.push_back(erec);
  }

  // Drain mid-layer exits accumulated by CopyContSliceToRootBuf across all
  // non-final layers of this session.
  for (const auto& mid_rec : impl_->accumulated_mid_exits_) {
    out.push_back(mid_rec);
  }
  impl_->accumulated_mid_exits_.clear();
  return out.size();
}

void MetalTraceBackend::EndSession() {
  impl_->Reset();
}

bool MetalTraceBackend::IsCompatible(const RenderConfig& render) const {
  if (render.lens_.type_ == LensParam::kRectangular) {
    return std::abs(render.view_.el_ - 90.0f) <= 0.01f;
  }
  if (render.lens_.type_ == LensParam::kDualFisheyeEqualArea) {
    // kernel implements fov180 full-globe only; reject custom fov to avoid
    // silent wrong-projection from non-GUI callers (LUMICE_TRACE_BACKEND=metal
    // + dual-fisheye config). view.el is unused for full-globe — no constraint.
    return std::abs(render.lens_.fov_ - 180.0f) <= 0.5f;
  }
  return false;
}

}  // namespace lumice
