#ifndef SRC_CORE_BEAM_TRACER_H_
#define SRC_CORE_BEAM_TRACER_H_

#include <cstddef>
#include <vector>

#include "core/geo3d.hpp"

namespace lumice {

class Crystal;
class Rotation;


// A beam segment propagating inside a crystal.
struct BeamSegment {
  float direction[3]{};           // Propagation direction (crystal frame)
  ConvexPolygon2D cross_section;  // 2D cross-section perpendicular to direction
  float weight = 0.0f;            // Accumulated Fresnel weight
  int face_id = -1;               // Current polygon face ID (-1 = before entry)
  size_t depth = 0;               // Bounce depth (number of internal reflections so far)
  std::vector<int> raypath;       // Face number sequence for Filter/stats
};


// Output of beam tracing for one crystal orientation.
struct BeamTraceResult {
  std::vector<float> outgoing_d;                   // Outgoing directions [dx,dy,dz, ...], 3*M floats
  std::vector<float> outgoing_w;                   // Outgoing weights, M floats
  std::vector<std::vector<int>> outgoing_raypath;  // Face number sequence per outgoing beam
  float total_entry_area = 0.0f;                   // Total entry projected area (for normalization)
};


// Perform beam tracing through a convex crystal.
// rot: rotation that transforms crystal frame → world frame
//      (i.e., rot.ApplyInverse transforms world → crystal).
// light_dir: incoming light direction in world frame (unit vector, pointing toward crystal).
// refractive_index: ice refractive index at the wavelength of interest.
// max_hits: maximum number of internal bounces.
// Precondition: crystal must be centered at the origin (rotation center = centroid = origin).
// Precondition: crystal.PolygonFaceCount() > 0 (convex crystal with slab data).
BeamTraceResult BeamTrace(const Crystal& crystal, const Rotation& rot, const float* light_dir, float refractive_index,
                          size_t max_hits);


}  // namespace lumice

#endif  // SRC_CORE_BEAM_TRACER_H_
