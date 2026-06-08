// Lumice Metal trace kernel — fused trace + recorder + (equirect XYZ
// accumulate | device-side compaction append) for one MS layer.
//
// IMPORTANT: this file (doc/trace_layer.metal) is the readable reference and
// standalone syntax-check target (`xcrun -sdk macosx metal -c trace_layer.metal`).
// It is NOT compiled by the build system. The runtime source is the kKernelSrc
// string literal in src/core/metal_trace_backend.mm — the two MUST stay
// logically equivalent (kernel body identical; this file may carry extra comments).
// NOTE (since 253.1): equivalence currently holds for the FRAME-TRANSFORM logic
// (per-ray root_rot, mat·v before projection). This reference omits the
// exit-stats accumulators (buffers 15/16, exit_cnt/exit_wsum) present in
// kKernelSrc — pre-existing drift, see DECISION 2026-06-08 in the 253.1 task.
// Do not treat this file as a full drop-in replacement for kKernelSrc.
//
// Kernel form follows trace_full.metal from the explore spike
// (scratchpad/explore-gpu-metal-trace-spike/spike/trace_full.metal): one
// thread per root ray, registers-resident state, `to_face`-select fused
// reflect/refract. The DO_RECORD/ACCUM_MODE/MS_MODE compile-time switches of
// the spike collapse into:
//   - recorder is always on (path[] in thread-local, checksum sink writes
//     back to prevent compiler elision). v1 has no filter-predicate consumer
//     so the path itself is unread — this exists to validate the register-
//     pressure form that the explore measured at ≤16% cost.
//   - XYZ is always 3-channel, atomics on `image[pix*3 + 0..2]` weighted by
//     host-supplied CIE CMF values (cie_x, cie_y, cie_z at the session wl).
//   - ms_mode is a runtime branch: 0 → equirect-project + XYZ accumulate
//     (final-layer terminal), 1 → atomic-append compaction into continuation
//     buffers (non-final layer feeding the next TraceLayer).

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
    // NOTE: buffers 15 (exit_cnt) and 16 (exit_wsum) — the parity-harness
    // exit-ray stats accumulators — exist in kKernelSrc but are omitted from
    // this readable reference (pre-existing drift, not 253.1). root_rot keeps
    // its real index 17 to stay logically equivalent on the frame transform.
    device const float*    root_rot [[buffer(17)]],
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
  // projection so the seam returns world-space rays (invariant 6). Constant
  // over the whole ray path — load once.
  float m[9];
  for (uint k = 0u; k < 9u; k++) { m[k] = root_rot[tid * 9u + k]; }

  const float n_idx    = prm.n_idx;
  const uint  poly_cnt = prm.poly_cnt;

  // Equirect projection scale = min(W/2, H) / pi  (matches CPU
  // RectangularProject in src/core/lens_proj.hpp).
  const int   iw_i      = int(prm.img_w);
  const int   ih_i      = int(prm.img_h);
  const float img_w_f   = float(prm.img_w);
  const float img_h_f   = float(prm.img_h);
  const float short_res = float(min(prm.img_w / 2u, prm.img_h));
  const float proj_scl  = short_res / M_PI_F;

  // Backend-local recorder. Fixed-cap thread-local; explore validated ≤16%
  // cost up through mh64 with no register spill on M2.
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
        // Outgoing ray.
        if (prm.ms_mode == 1u) {
          // Append: pick the argmax-facing face and seed a continuation root.
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
          uint slot = atomic_fetch_add_explicit(counter, 1u, memory_order_relaxed);
          if (slot < prm.out_cap) {
            out_d[slot * 3u + 0u] = cdx;
            out_d[slot * 3u + 1u] = cdy;
            out_d[slot * 3u + 2u] = cdz;
            out_p[slot * 3u + 0u] = centroid[ef * 3u + 0u];
            out_p[slot * 3u + 1u] = centroid[ef * 3u + 1u];
            out_p[slot * 3u + 2u] = centroid[ef * 3u + 2u];
            out_w[slot] = cw;
            out_tf[slot] = ushort(ef);
          }
        } else {
          // Accumulate: equirect projection + 3-channel XYZ atomic add. Mirror
          // CPU RectangularProject + SpectrumToXyz from src/core/. CPU uses
          // raw (un-renormalized) direction — Metal must skip the rsqrt to
          // stay bit-close (the trace formulas already preserve |d|=1 modulo
          // float drift; atan2 is scale-invariant, asin is clamped).
          // Return the exit direction from crystal-local to world space
          // (invariant 6 / DESIGN D2): world = m * cd, row-major, matching
          // CPU CollectData's crystal_rot_.Apply(r.d_).
          float wx = m[0] * cdx + m[1] * cdy + m[2] * cdz;
          float wy = m[3] * cdx + m[4] * cdy + m[5] * cdz;
          float wz = m[6] * cdx + m[7] * cdy + m[8] * cdz;
          float sx = -wx;
          float sy = -wy;
          float sz = -wz;
          float lon = atan2(sy, sx) - prm.az0;
          // Wrap lon to [-pi, pi]: equivalent to the while-loops in
          // RectangularProject. At exact lon == +pi the formula yields -pi;
          // after the (raw_x mod W) wrap this collapses to the same pixel.
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

  // Recorder sink: write a per-thread checksum so the compiler cannot elide
  // path[] stores. v1 has no consumer; once filter-predicate evaluation lands
  // this will be replaced by an actual filter pass over `path`.
  float rec_csum = 0.0f;
  for (uint k = 0u; k < rec_len; k++) {
    rec_csum += float(path[k]);
  }
  rec_sink[tid] = rec_csum;
}
