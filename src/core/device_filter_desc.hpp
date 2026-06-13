// Device-side filter MATCH descriptor + per-crystal GetFn table builders.
//
// Scope: scrum-267 task-msl-filter-match-port. Produces a flat, GPU-friendly
// description of a `FilterConfig` for the device filter MATCH path (E4 spike
// extended to full filter type coverage). The MSL kernel reads these structs
// + a per-crystal GetFn byte table to mirror `FilterSpec::Check` bit-for-bit
// (validated by the parity harness at scrum-267.1 acceptance).
//
// Layout contract: this header is the single source of truth for the C++ side
// of the host/device `DeviceFilterDesc` layout (D2 in plan). The MSL source
// string in `metal_filter_match_src.mm` must mirror field order/types exactly.
//
// Locality: not part of the public C API. Only compiled into lumice_obj on
// Apple builds (paired with the Metal backend).

#ifndef SRC_CORE_DEVICE_FILTER_DESC_H_
#define SRC_CORE_DEVICE_FILTER_DESC_H_

#if defined(__APPLE__)

#include <cstdint>
#include <vector>

#include "config/filter_config.hpp"
#include "core/crystal.hpp"
#include "core/math.hpp"

namespace lumice {

// Device-side filter type tags. Must stay aligned with the MSL `kDevFilterType*`
// constants in `kFilterMatchHelperSrc` (mirror enforced by the parity harness:
// any tag drift produces mismatch on a non-empty filter).
constexpr uint8_t kDeviceFilterTypeNone = 0;
constexpr uint8_t kDeviceFilterTypeRaypath = 1;
constexpr uint8_t kDeviceFilterTypeEntryExit = 2;
constexpr uint8_t kDeviceFilterTypeDirection = 3;
constexpr uint8_t kDeviceFilterTypeCrystal = 4;
// Complex is out of scope this task (plan §2). Device dispatch returns
// pass-through `true` for type=5 so generation paths that synthesize Complex
// descs do not silently corrupt other types.
constexpr uint8_t kDeviceFilterTypeComplex = 5;

// Plain-data descriptor uploaded to the Metal `filter_desc_buf_` (per filter).
//
// Field order/type MUST match the MSL `struct DeviceFilterDesc` declared in
// kFilterMatchHelperSrc. Fields not relevant to the active `type` are zero-
// initialized but still occupy their slots (fixed layout simplifies the device
// reader).
//
// `d_applicable` semantics: mirrors `RaypathOrbit::d_applicable_` — when false
// the device MATCH path skips the σ-mirror branch of ReduceBuffer even if
// `symmetry & kSymD` is set, exactly as `detail::ReduceBuffer` does
// (filter_spec.cpp:67). Both host and device read this single flag; no other
// short-circuit logic is keyed off it.
struct DeviceFilterDesc {
  uint8_t type;                 // see kDeviceFilterType* above
  uint8_t action;               // 0=kFilterIn (match→true), 1=kFilterOut (match→false)
  uint8_t symmetry;             // FilterConfig::kSymP/B/D bitmask
  uint8_t d_applicable;         // 0/1; gates σ-mirror branch in ReduceBuffer
  int32_t sigma_a;              // σ-mirror parameter; meaningful only when d_applicable
  int32_t fn_period;            // Crystal::FnPeriod(); <0 ⇒ skip ReduceBuffer (memcmp only)
  uint8_t canonical_bytes[64];  // = kMaxHits; Raypath/EntryExit canonical hits
  uint8_t canonical_len;        // # significant bytes in canonical_bytes
  uint8_t has_entry;            // EntryExit only
  uint8_t has_exit;             // EntryExit only
  uint8_t _pad0;                // align next 4-byte block
  uint32_t min_len;             // EntryExit lower bound (≥1); Raypath uses canonical_len
  uint32_t max_len;             // EntryExit upper bound; 0 = no upper bound
  float dir[3];                 // Direction only (unit cartesian vector)
  float radii_c;                // Direction only: cos(radii_deg)
  uint32_t crystal_id;          // Crystal only
};

static_assert(sizeof(DeviceFilterDesc) <= 256, "DeviceFilterDesc must stay under 256 bytes — see plan §3.D2");

namespace detail {

// Build a flat `DeviceFilterDesc` from a host-side `FilterConfig`.
//
// `crystal` provides `GetFn` (for Raypath/EntryExit canonical computation) and
// `FnPeriod` (for the fn_period field). `axis_dist` resolves
// `d_applicable` + `sigma_a` exactly as `FilterSpec::Create` does. The current
// project supports hexagonal crystals only (`fn_period == 6`); for the custom-
// crystal fallback the function still produces a valid desc — the device path
// memcmps verbatim when `fn_period < 0`.
//
// Complex filters (`ComplexFilterParam`) produce a desc with
// `type = kDeviceFilterTypeComplex` and otherwise-zero fields; the device-side
// dispatch returns pass-through `true` for that branch (plan §2 — Complex is
// out of scope this task).
DeviceFilterDesc BuildDeviceFilterDesc(const FilterConfig& config, const Crystal& crystal,
                                       const AxisDistribution& axis_dist);

// Build the per-crystal poly-index → face-number byte table. Mirrors the GetFn
// remap that `CopyContSliceToRootBuf` does on host (metal_trace_backend.mm
// :1670-1675). Output length == `crystal.PolygonFaceCount()`; each byte is
// `crystal.GetFn(poly_idx) & 0xFF` (face numbers fit in 7 bits — basal 1/2,
// prism 3..8, pyramidal 13..18/23..28).
std::vector<uint8_t> BuildDeviceGetFnBytes(const Crystal& crystal);

}  // namespace detail
}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // SRC_CORE_DEVICE_FILTER_DESC_H_
