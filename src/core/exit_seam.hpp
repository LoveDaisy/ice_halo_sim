#ifndef CORE_EXIT_SEAM_H_
#define CORE_EXIT_SEAM_H_

#include <cstdint>
#include <type_traits>

#include "core/raypath.hpp"

namespace lumice {

// Per-ray rich exit record returned by TraceBackend::ReadbackExitRays
// (scrum-258.2+). Carries metadata for downstream filter + symmetry-fold
// (258.3); 258.2 only produces records, it does NOT consume crystal_id or
// path. Defined-layout, trivially copyable, no dynamic allocation — backends
// can memcpy directly out of device buffers.
//
// Byte layout (verified against static_assert below; do not hand-recompute
// offsets — the compiler is the source of truth):
//   float dir[3]              (12B, world-space exit direction)
//   float weight              (4B,  ray weight)
//   ExitFaceSeq path          (65B, inline face sequence; task-284 bumped
//                                   kCap 15→64 so paths up to kMaxHits=64 are
//                                   no longer silently truncated)
//   uint16_t crystal_id       (2B, session-level crystal index; 1B padding
//                                   before it due to path ending at odd offset)
//   uint8_t ms_layer_idx      (1B,  0-based MS layer index)
//   uint8_t wl_idx            (1B,  per-ray wavelength pool index; 0 on the
//                                   CPU backend — pool layout is Metal-specific,
//                                   scrum-268.8)
//   uint64_t component_mask   (8B,  task-331.1 raypath-color foundation per-ray
//                                   mask; 0 on Metal/CUDA in phase-1 because
//                                   the device path does not populate it. CPU
//                                   backend copies it out of workspace[1].
//                                   ComponentAt(j) at ExitRayRecord assembly.)
// The uint64_t at the end forces 8-byte alignment on the whole struct, so
// sizeof jumps from the pre-331.1 88B → 96B (path-anchor trailing pad grows
// to align component_mask to 8B).
// ExitRayRecord has no on-disk serialization (used in-process between trace
// backend and consumer), so the sizeof bump only adjusts the static_assert.
struct ExitRayRecord {
  float dir[3];
  float weight;
  ExitFaceSeq path;
  uint16_t crystal_id;
  uint8_t ms_layer_idx;
  uint8_t wl_idx = 0;
  uint64_t component_mask = 0;  // task-331.1 (raypath-color foundation).
};
static_assert(sizeof(ExitRayRecord) == 96u,
              "ExitRayRecord layout changed — re-check exit-seam wire format and bandwidth budget");
static_assert(std::is_trivially_copyable_v<ExitRayRecord>,
              "ExitRayRecord must stay trivially copyable for memcpy out of device buffers");

}  // namespace lumice

#endif  // CORE_EXIT_SEAM_H_
