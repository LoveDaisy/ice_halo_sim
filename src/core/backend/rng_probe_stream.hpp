// Host-side RngProbeStream enum + wire values (scrum-328.2 Step 1).
//
// Single source of truth for which device PCG stream a test wants the raw
// pcg_uniform draw of, when calling `EnableRngProbeForTest`. Extracts the
// implicit "which kernel is executed next" routing that the pre-scrum-328.2
// probe wiring relied on (task-325 GPU RNG probe: gate on trace kernel when
// `ms_mode==1`, transit on transit kernel between layers) into an explicit
// selector — the corresponding kernel checks `probe_stream_wire ==
// <own_kind>` before writing, so a mis-timed Enable now surfaces as an all-
// zero readback instead of silently capturing whichever stream ran.
//
// Host-only: this header is included by CUDA/Metal backend headers + backend
// `.cu`/`.mm` implementations, but NOT by device shader / pcg_shared.h — the
// device side only sees the raw uint32_t wire value carried in a kernel arg.
#ifndef LM_RNG_PROBE_STREAM_H_
#define LM_RNG_PROBE_STREAM_H_

#include <cstdint>

namespace lumice {

// Wire values MUST stay contiguous and start at 0 — the CUDA/Metal kernel
// checks `probe_stream_wire == <constant>` so any renumbering must be applied
// pairwise (host cast site + kernel constant).
enum class RngProbeStream : uint32_t {
  kGen = 0u,        // gen_root_kernel  — first-layer root sampler
  kGateMs1 = 1u,    // trace_single_ms_kernel (ms_mode==1) — per-bounce emit gate
  kGateFinal = 2u,  // trace_single_ms_kernel (ms_mode==0) — final-layer emit gate
  kTransit = 3u,    // transit_multi_ms_kernel / transit_root_kernel — MS-layer transit
};

constexpr uint32_t ToWireValue(RngProbeStream s) {
  return static_cast<uint32_t>(s);
}

// Sentinel used when no probe is armed — kernels compare against a live wire
// value, so an out-of-range value guarantees no branch matches.
constexpr uint32_t kRngProbeStreamNone = 0xFFFFFFFFu;

}  // namespace lumice

#endif  // LM_RNG_PROBE_STREAM_H_
