// Device-side filter MATCH functions shared between CUDA kernels and host
// (CPU-side parity verification). Mirrors the MSL implementation in
// `src/core/metal/lumice_trace.metal:61-313` byte-for-byte; the host and CUDA
// build use this header, while Metal continues to use the MSL copy embedded in
// the metallib. Keep this file's logic in lock-step with the MSL source — the
// CPU-side unit test (test_device_filter_check_host.cpp) catches drift on the
// host/CUDA side, and the Metal parity battery catches drift between host and
// MSL.
//
// Function modifiers go through `LM_FN` (see lm_shims.h): host C++ → `inline`,
// CUDA → `__host__ __device__ inline`. All pointers are plain (no MSL address-
// space qualifiers); CUDA / host compile this header into both __device__ and
// host code paths so the same translation unit can be linked into the CUDA
// kernel + the unit test.
//
// Locality: not part of the public C API. Pulled in by the CUDA backend
// (`src/core/backend/cuda_trace_backend.cu`) and the CPU-side unit test.

#ifndef SRC_CORE_SHARED_FILTER_SHARED_H_
#define SRC_CORE_SHARED_FILTER_SHARED_H_

#include <cstdint>

#include "core/device_filter_desc.hpp"
#include "core/shared/lm_shims.h"

// Naming note: function bodies and identifiers (kDev*, *_dev) preserve the
// snake-case / suffix shape used in the MSL source so byte-for-byte mirroring
// is verifiable by inspection. Project-default CamelCase would obscure that.
// Cognitive-complexity / function-size warnings on ReduceBuffer_dev + dispatch
// functions are intrinsic to mirroring the MSL control flow exactly; splitting
// them would invite divergence. Silenced section-wide rather than per function.
// NOLINTBEGIN(readability-identifier-naming,readability-function-cognitive-complexity,readability-function-size)
namespace lumice {
namespace lm_filter {

// --- Constants (mirror MSL kDev* in lumice_trace.metal) ---------------------
//
// Tags duplicate the host-side kDeviceFilterType* constants intentionally:
// both sides import `device_filter_desc.hpp`, so we can just reference those
// rather than redeclare. ReduceBuffer's fn_period constant is hex-only (the
// only supported crystal family); see `detail::ReduceBuffer` in
// filter_spec.cpp:23 for the host counterpart.
inline constexpr int kDevFnPeriodHex = 6;
inline constexpr uint32_t kDevRecCap = 64u;  // == ExitFaceSeq::kCap == kMaxHits

// --- ReduceBuffer (canonical re-ordering of a face-number sequence) ---------
//
// Byte-identical with `lumice::detail::ReduceBuffer` in filter_spec.cpp:60-95
// and `ReduceBuffer_dev` in lumice_trace.metal:83-120. Operates in place on
// `data[0..size-1]` (face-number space, NOT poly-index space — callers must
// pre-remap via ApplyGetFn_dev).
LM_FN void PCanonicalShiftInPlace_dev(uint8_t* data, uint32_t size) {
  int first_pri = -1;
  for (uint32_t i = 0; i < size; ++i) {
    uint8_t x = data[i];
    if (x < 3u) {
      continue;
    }
    uint8_t pyr = static_cast<uint8_t>(x / 10u);
    int pri = static_cast<int>(x % 10u);
    if (first_pri < 0) {
      first_pri = pri;
    }
    pri = (pri + kDevFnPeriodHex - first_pri) % kDevFnPeriodHex + 3;
    data[i] = static_cast<uint8_t>(pyr * 10u + static_cast<uint32_t>(pri));
  }
}

LM_FN bool LexLess_dev(const uint8_t* a, const uint8_t* b, uint32_t size) {
  for (uint32_t i = 0; i < size; ++i) {
    if (a[i] != b[i]) {
      return a[i] < b[i];
    }
  }
  return false;
}

LM_FN void ReduceBuffer_dev(uint8_t* data, uint32_t size, uint8_t symmetry, int32_t sigma_a, bool d_applicable) {
  if (symmetry == 0u /* kSymNone */) {
    return;
  }
  if (symmetry & 1u /* kSymP */) {
    PCanonicalShiftInPlace_dev(data, size);
  }
  if ((symmetry & 4u /* kSymD */) && d_applicable) {
    uint8_t scratch[kDevRecCap];
    for (uint32_t i = 0; i < size; ++i) {
      uint8_t x = data[i];
      if (x < 3u) {
        scratch[i] = x;
        continue;
      }
      uint8_t pyr = static_cast<uint8_t>(x / 10u);
      int pri0 = static_cast<int>(x % 10u) - 3;
      int new_pri0 = ((sigma_a - pri0) % kDevFnPeriodHex + kDevFnPeriodHex) % kDevFnPeriodHex;
      scratch[i] = static_cast<uint8_t>(pyr * 10u + static_cast<uint32_t>(new_pri0 + 3));
    }
    if (symmetry & 1u /* kSymP */) {
      PCanonicalShiftInPlace_dev(scratch, size);
    }
    if (LexLess_dev(scratch, data, size)) {
      for (uint32_t i = 0; i < size; ++i) {
        data[i] = scratch[i];
      }
    }
  }
  if (symmetry & 2u /* kSymB */) {
    uint8_t scratch[kDevRecCap];
    bool changed = false;
    for (uint32_t i = 0; i < size; ++i) {
      uint8_t x = data[i];
      if (x <= 2u) {
        scratch[i] = static_cast<uint8_t>(3u - x);
        changed = true;
      } else if (x >= 13u && x <= 18u) {
        scratch[i] = static_cast<uint8_t>(x + 10u);
        changed = true;
      } else if (x >= 23u && x <= 28u) {
        scratch[i] = static_cast<uint8_t>(x - 10u);
        changed = true;
      } else {
        scratch[i] = x;
      }
    }
    if (changed && LexLess_dev(scratch, data, size)) {
      for (uint32_t i = 0; i < size; ++i) {
        data[i] = scratch[i];
      }
    }
  }
}

// --- ApplyGetFn (remap poly-index path → face-number space) -----------------
//
// `getfn_offsets[crystal_slot]` is the byte-start of this crystal's stripe in
// the flat `getfn_bytes` array; for poly-index `p`, the face-number is
// `getfn_bytes[base + p]`. The caller-owned `out` buffer must hold at least
// `len` bytes. Trusts `path[k] < stripe_len` (kernel/test inputs control this).
LM_FN void ApplyGetFn_dev(const uint8_t* path, uint32_t len, const uint8_t* getfn_bytes, const uint32_t* getfn_offsets,
                          uint32_t crystal_slot, uint8_t* out) {
  uint32_t base = getfn_offsets[crystal_slot];
  for (uint32_t i = 0; i < len; ++i) {
    out[i] = getfn_bytes[base + path[i]];
  }
}

// --- DeviceFilterMatch* per type --------------------------------------------
//
// Returns the PREDICATE result (no action_ XOR). DeviceFilterCheck applies
// action_ at the end. Mirrors filter_spec.hpp:42-45.

LM_FN bool DeviceFilterMatchRaypath(const DeviceFilterDesc& f, const uint8_t* path, uint32_t path_len,
                                    const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot) {
  if (path_len != f.canonical_len) {
    return false;
  }
  uint8_t buf[kDevRecCap];
  ApplyGetFn_dev(path, path_len, getfn_bytes, getfn_offsets, crystal_slot, buf);
  if (f.fn_period < 0 || f.symmetry == 0u /* kSymNone */) {
    for (uint32_t i = 0; i < path_len; ++i) {
      if (buf[i] != f.canonical_bytes[i]) {
        return false;
      }
    }
    return true;
  }
  ReduceBuffer_dev(buf, path_len, f.symmetry, f.sigma_a, f.d_applicable != 0u);
  for (uint32_t i = 0; i < path_len; ++i) {
    if (buf[i] != f.canonical_bytes[i]) {
      return false;
    }
  }
  return true;
}

LM_FN bool DeviceFilterMatchEntryExit(const DeviceFilterDesc& f, const uint8_t* path, uint32_t path_len,
                                      const uint8_t* getfn_bytes, const uint32_t* getfn_offsets,
                                      uint32_t crystal_slot) {
  if (path_len == 0u) {
    return false;
  }
  if (path_len < f.min_len) {
    return false;
  }
  if (f.max_len != 0u && path_len > f.max_len) {
    return false;
  }
  if (!f.has_entry && !f.has_exit) {
    return true;
  }
  uint8_t ee[2];
  uint32_t base = getfn_offsets[crystal_slot];
  uint32_t ee_len = 0u;
  if (f.has_entry) {
    ee[ee_len++] = getfn_bytes[base + path[0]];
  }
  if (f.has_exit) {
    ee[ee_len++] = getfn_bytes[base + path[path_len - 1u]];
  }
  if (f.fn_period < 0 || f.symmetry == 0u /* kSymNone */) {
    for (uint32_t i = 0; i < ee_len; ++i) {
      if (ee[i] != f.canonical_bytes[i]) {
        return false;
      }
    }
    return true;
  }
  uint8_t buf[kDevRecCap];
  for (uint32_t i = 0; i < ee_len; ++i) {
    buf[i] = ee[i];
  }
  ReduceBuffer_dev(buf, ee_len, f.symmetry, f.sigma_a, f.d_applicable != 0u);
  if (ee_len != f.canonical_len) {
    return false;
  }
  for (uint32_t i = 0; i < ee_len; ++i) {
    if (buf[i] != f.canonical_bytes[i]) {
      return false;
    }
  }
  return true;
}

LM_FN bool DeviceFilterMatchDirection(const DeviceFilterDesc& f, const float* ray_dir) {
  float d = f.dir[0] * ray_dir[0] + f.dir[1] * ray_dir[1] + f.dir[2] * ray_dir[2];
  return d > f.radii_c;
}

LM_FN bool DeviceFilterMatchCrystal(const DeviceFilterDesc& f, uint32_t ray_crystal_config_id) {
  return ray_crystal_config_id == f.crystal_id;
}

// Simple-only dispatch (no Complex). DeviceFilterMatchComplex calls this for
// each sub-spec; including Complex here would form a static cycle (mirrors
// MSL discipline at lumice_trace.metal:213-218).
LM_FN bool DeviceFilterMatchSimple(const DeviceFilterDesc& f, const uint8_t* path, uint32_t path_len,
                                   const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot,
                                   const float* ray_dir, uint32_t ray_crystal_config_id) {
  if (f.type == kDeviceFilterTypeNone) {
    return true;
  }
  if (f.type == kDeviceFilterTypeRaypath) {
    return DeviceFilterMatchRaypath(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot);
  }
  if (f.type == kDeviceFilterTypeEntryExit) {
    return DeviceFilterMatchEntryExit(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot);
  }
  if (f.type == kDeviceFilterTypeDirection) {
    return DeviceFilterMatchDirection(f, ray_dir);
  }
  if (f.type == kDeviceFilterTypeCrystal) {
    return DeviceFilterMatchCrystal(f, ray_crystal_config_id);
  }
  return false;
}

// Complex = OR over AND-clauses of Simple sub-specs. Empty Complex returns
// false (matches host `ComplexSpec::Match` falling-through at
// filter_spec.cpp:274-288; DO NOT change to true — would silently disable
// deferred-construction Complex filters).
LM_FN bool DeviceFilterMatchComplex(const DeviceFilterDesc& f, const DeviceFilterDesc* complex_sub_desc_buf,
                                    const uint8_t* and_term_counts_buf, const uint8_t* path, uint32_t path_len,
                                    const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot,
                                    const float* ray_dir, uint32_t ray_crystal_config_id) {
  if (f.or_clause_count == 0u) {
    return false;
  }
  uint32_t sub_idx = f.sub_desc_start;
  for (uint32_t or_i = 0; or_i < static_cast<uint32_t>(f.or_clause_count); ++or_i) {
    uint32_t and_n = static_cast<uint32_t>(and_term_counts_buf[f.and_terms_start + or_i]);
    bool and_ok = true;
    for (uint32_t and_j = 0; and_j < and_n; ++and_j) {
      if (!DeviceFilterMatchSimple(complex_sub_desc_buf[sub_idx], path, path_len, getfn_bytes, getfn_offsets,
                                   crystal_slot, ray_dir, ray_crystal_config_id)) {
        and_ok = false;
        // Skip remaining sub-descs in this AND-clause so sub_idx lands at the
        // next OR-clause start (matches MSL short-circuit at
        // lumice_trace.metal:268-270).
        sub_idx += (and_n - and_j);
        break;
      }
      ++sub_idx;
    }
    if (and_ok) {
      return true;
    }
  }
  return false;
}

// Top-level dispatch — Complex → DeviceFilterMatchComplex; everything else →
// DeviceFilterMatchSimple. Static call graph stays acyclic.
LM_FN bool DeviceFilterMatch(const DeviceFilterDesc& f, const DeviceFilterDesc* complex_sub_desc_buf,
                             const uint8_t* and_term_counts_buf, const uint8_t* path, uint32_t path_len,
                             const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot,
                             const float* ray_dir, uint32_t ray_crystal_config_id) {
  if (f.type == kDeviceFilterTypeComplex) {
    return DeviceFilterMatchComplex(f, complex_sub_desc_buf, and_term_counts_buf, path, path_len, getfn_bytes,
                                    getfn_offsets, crystal_slot, ray_dir, ray_crystal_config_id);
  }
  return DeviceFilterMatchSimple(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot, ray_dir,
                                 ray_crystal_config_id);
}

// Check = Match XOR (action == filter_out). Same algebra as
// filter_spec.hpp:42-45.
LM_FN bool DeviceFilterCheck(const DeviceFilterDesc& f, const DeviceFilterDesc* complex_sub_desc_buf,
                             const uint8_t* and_term_counts_buf, const uint8_t* path, uint32_t path_len,
                             const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot,
                             const float* ray_dir, uint32_t ray_crystal_config_id) {
  bool m = DeviceFilterMatch(f, complex_sub_desc_buf, and_term_counts_buf, path, path_len, getfn_bytes, getfn_offsets,
                             crystal_slot, ray_dir, ray_crystal_config_id);
  return (f.action == 0u) ? m : !m;
}

// task-331.6 (raypath-color foundation): per-summand match mask, the device
// mirror of host `ComplexSpec::MatchSummandMask` / `FilterSpec::MatchSummandMask`
// (filter_spec.cpp:307 / filter_spec.hpp:58) and MSL `DeviceFilterSummandMask`
// (lumice_trace.metal). Returns a mask whose bit k is set iff OR-summand k
// matched — EVERY clause is evaluated (no cross-clause short-circuit) so the
// collapse-to-boolean gate does not discard the per-summand info the component
// mask needs. `out_matched` receives the pre-action collapse boolean
// (== DeviceFilterMatch). Semantics:
//   - Complex       : bit or_i set iff its AND-clause fully matched.
//   - None          : device emits mask 0, matched=true. NOTE (task-339.1):
//                     host NoneSpec now yields a whole-crystal bit (mask 0b1,
//                     1 summand) so BuildComponentTable allocates a None bit and
//                     shifts later bits by +1. The device intentionally still
//                     emits 0 here — None whole-crystal bits are host-only until
//                     scrum-3c wires device-side per-class lanes. Do NOT treat
//                     this as "mirrors NoneSpec" anymore.
//   - other simple  : 1 summand → bit 0 iff matched (mirrors non-None simple).
// The result is capped at kDeviceFilterMaxOrClauses summands (== the Complex
// or_clause_count upper bound); the component-bit table uses the same stride.
LM_FN uint32_t DeviceFilterSummandMask(const DeviceFilterDesc& f, const DeviceFilterDesc* complex_sub_desc_buf,
                                       const uint8_t* and_term_counts_buf, const uint8_t* path, uint32_t path_len,
                                       const uint8_t* getfn_bytes, const uint32_t* getfn_offsets, uint32_t crystal_slot,
                                       const float* ray_dir, uint32_t ray_crystal_config_id, bool* out_matched) {
  if (f.type == kDeviceFilterTypeComplex) {
    uint32_t mask = 0u;
    uint32_t sub_idx = f.sub_desc_start;
    // task-device-flat-and-terms: the `or_i < kDeviceFilterMaxOrClauses` cap
    // is INTENTIONAL here — this function is a color-path helper (see the
    // comment block above) and the color bit map / uint32 mask are still
    // stride-limited to `kDeviceFilterMaxOrClauses` summands. Physical-filter
    // Complex descriptors that carry more OR-clauses use `DeviceFilterCheck`,
    // whose loop is bounded only by `f.or_clause_count` (no color-side cap).
    for (uint32_t or_i = 0u; or_i < static_cast<uint32_t>(f.or_clause_count) && or_i < kDeviceFilterMaxOrClauses;
         ++or_i) {
      uint32_t and_n = static_cast<uint32_t>(and_term_counts_buf[f.and_terms_start + or_i]);
      bool and_ok = true;
      for (uint32_t and_j = 0u; and_j < and_n; ++and_j) {
        if (!DeviceFilterMatchSimple(complex_sub_desc_buf[sub_idx], path, path_len, getfn_bytes, getfn_offsets,
                                     crystal_slot, ray_dir, ray_crystal_config_id)) {
          and_ok = false;
          sub_idx += (and_n - and_j);  // skip rest of this AND-clause
          break;
        }
        ++sub_idx;
      }
      if (and_ok) {
        mask |= (1u << or_i);
      }
    }
    *out_matched = (mask != 0u);
    return mask;
  }
  if (f.type == kDeviceFilterTypeNone) {
    *out_matched = true;  // None passes but contributes 0 summands
    return 0u;
  }
  bool m = DeviceFilterMatchSimple(f, path, path_len, getfn_bytes, getfn_offsets, crystal_slot, ray_dir,
                                   ray_crystal_config_id);
  *out_matched = m;
  return m ? 1u : 0u;
}

}  // namespace lm_filter
}  // namespace lumice
// NOLINTEND(readability-identifier-naming,readability-function-cognitive-complexity,readability-function-size)

#endif  // SRC_CORE_SHARED_FILTER_SHARED_H_
