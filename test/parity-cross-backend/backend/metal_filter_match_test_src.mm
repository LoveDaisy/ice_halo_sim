// Parity-only Metal kernel source string (test scaffold).
//
// Moved out of `src/core/metal_filter_match_src.mm` by task-270.8 boundary-
// hardening so the test kernel no longer ships inside liblumice. The MSL
// kernel below references identifiers from `kFilterMatchHelperSrc`; callers
// must concatenate `kFilterMatchHelperSrc` (production helpers) before this
// string when building the parity Metal library.

#include "parity-cross-backend/backend/metal_filter_match_test_src.hpp"

#if defined(__APPLE__)

namespace lumice {

// clang-format off
const char* const kFilterMatchTestKernelSrc = R"METAL(
struct FilterMatchTestParams {
  uint n;
  uint face_seq_cap;
  uint check_mode;  // 0 = Match (ignore action), 1 = Check (apply action)
};

// One thread per ray. Reads (path, path_len, crystal_slot, crystal_config_id,
// ray_dir, filter_idx) and writes a single uchar 0/1 to `out_match`.
//
// Buffer layout (mirrors plan D6 binding contract — both this kernel and the
// host harness must agree on indices):
//   0  filter_desc_buf       : DeviceFilterDesc[n_filters]
//   1  (retired: the getfn_offsets prefix-sum table was dropped; the
//       GetFn remap is now per-instance, indexed by the ray's poly_off)
//   2  poly_fn_buf           : uchar[] per-instance GetFn table (single crystal
//                              in this fixture → ray_cslot doubles as poly_off=0)
//   3  ray_path_buf          : uchar[n * face_seq_cap]
//   4  ray_path_len_buf      : uchar[n]
//   5  ray_crystal_slot      : uint16[n]
//   6  ray_crystal_cid       : uint16[n]
//   7  ray_dir_buf           : float[n * 3]
//   8  ray_filter_idx        : uint[n]
//   9  out_match_buf         : uchar[n]
//   10 params                : FilterMatchTestParams
//   11 complex_sub_desc_buf  : DeviceFilterDesc[] (Complex sub-specs, flat)
//   12 and_term_counts_buf   : uchar[] (task-device-flat-and-terms — parallel
//                              flat AND-term counts, one uint8 per OR-clause
//                              across every Complex parent; indexed via
//                              `and_terms_start`). Independent MTLBuffer here
//                              (test kernel has plenty of free bindings —
//                              production Metal packs it into buffer 27 via
//                              KernelParams.and_term_counts_base_offset).
kernel void filter_match_test_kernel(
    device const DeviceFilterDesc* filter_desc        [[buffer(0)]],
    device const uchar*            poly_fn            [[buffer(2)]],
    device const uchar*            ray_path           [[buffer(3)]],
    device const uchar*            ray_path_len       [[buffer(4)]],
    device const ushort*           ray_cslot          [[buffer(5)]],
    device const ushort*           ray_cid            [[buffer(6)]],
    device const float*            ray_dir            [[buffer(7)]],
    device const uint*             ray_filter         [[buffer(8)]],
    device uchar*                  out_match          [[buffer(9)]],
    constant FilterMatchTestParams& prm               [[buffer(10)]],
    device const DeviceFilterDesc* complex_sub_desc   [[buffer(11)]],
    device const uchar*            and_term_counts    [[buffer(12)]],
    uint tid [[thread_position_in_grid]]) {
  if (tid >= prm.n) { return; }
  uint len = (uint)ray_path_len[tid];
  if (len > kDevRecCap) { len = kDevRecCap; }
  uchar path_local[64];  // kDevRecCap
  uint base = tid * prm.face_seq_cap;
  for (uint i = 0; i < len; i++) {
    path_local[i] = ray_path[base + i];
  }
  float dir_local[3];
  dir_local[0] = ray_dir[tid * 3 + 0];
  dir_local[1] = ray_dir[tid * 3 + 1];
  dir_local[2] = ray_dir[tid * 3 + 2];
  uint fi = ray_filter[tid];
  device const DeviceFilterDesc& f = filter_desc[fi];
  bool result;
  // ray_cslot doubles as poly_off (0 for this single-crystal
  // fixture, whose poly_fn stripe starts at absolute offset 0).
  if (prm.check_mode == 0u) {
    result = DeviceFilterMatch(f, complex_sub_desc, and_term_counts, path_local, len, poly_fn,
                               (uint)ray_cslot[tid], dir_local, (uint)ray_cid[tid]);
  } else {
    result = DeviceFilterCheck(f, complex_sub_desc, and_term_counts, path_local, len, poly_fn,
                               (uint)ray_cslot[tid], dir_local, (uint)ray_cid[tid]);
  }
  out_match[tid] = result ? 1u : 0u;
}
)METAL";
// clang-format on

}  // namespace lumice

#endif  // defined(__APPLE__)
