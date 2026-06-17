// MSL helper source strings for device filter MATCH.
//
// scrum-267 task-msl-filter-match-port (plan D3). `kFilterMatchHelperSrc` is
// the "device counterpart" of FilterSpec::Check / RaypathOrbit::Contains:
// `static inline` helpers spliced in front of `kKernelSrc` to wire the gate
// into the production trace kernel without name collisions. The companion
// parity harness lives under `test/parity-cross-backend/backend/` and
// supplies its own test kernel string
// (`kFilterMatchTestKernelSrc`, declared in
// `metal_filter_match_test_src.hpp`); the test kernel is no longer linked
// into liblumice (task-270.8 boundary-hardening).

#ifndef SRC_CORE_METAL_FILTER_MATCH_SRC_H_
#define SRC_CORE_METAL_FILTER_MATCH_SRC_H_

#if defined(__APPLE__)

namespace lumice {

// Trace-kernel splice contract (scrum-267 task-fused-emit-gate Step 1):
// kFilterMatchHelperSrc declares `constant uint kDevRecCap = 64` and the
// production trace kernel kKernelSrc declares `constant uint kRecCap = 64`.
// Both names refer to the same hop-count budget — the host-visible mirror
// below is checked against the trace kernel's `kTraceKernelRecCap` (in
// metal_trace_backend.mm) via static_assert so a future bump cannot drift
// the two MSL constants apart silently.
constexpr int kDevFilterMatchRecCap = 64;

// `static inline` helpers shared by the parity test kernel AND the
// production trace kernel splice. No I/O / kernel entry points here.
extern const char* const kFilterMatchHelperSrc;

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // SRC_CORE_METAL_FILTER_MATCH_SRC_H_
