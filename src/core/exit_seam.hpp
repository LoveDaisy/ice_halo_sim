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
//   offset 0:  float    dir[3]        (12B, world-space exit direction)
//   offset 12: float    weight        (4B,  ray weight)
//   offset 16: ExitFaceSeq path       (16B, inline face sequence)
//   offset 32: uint16_t crystal_id    (2B,  session-level crystal index)
//   offset 34: uint8_t  ms_layer_idx  (1B,  0-based MS layer index)
//   offset 35: uint8_t  pad_          (1B,  alignment padding)
//   sizeof == 36, align == 4.
struct ExitRayRecord {
  float dir[3];
  float weight;
  ExitFaceSeq path;
  uint16_t crystal_id;
  uint8_t ms_layer_idx;
  uint8_t pad_ = 0;
};
static_assert(sizeof(ExitRayRecord) == 36u,
              "ExitRayRecord layout changed — re-check exit-seam wire format and bandwidth budget");
static_assert(std::is_trivially_copyable_v<ExitRayRecord>,
              "ExitRayRecord must stay trivially copyable for memcpy out of device buffers");

}  // namespace lumice

#endif  // CORE_EXIT_SEAM_H_
