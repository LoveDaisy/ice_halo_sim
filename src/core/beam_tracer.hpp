#ifndef SRC_CORE_BEAM_TRACER_H_
#define SRC_CORE_BEAM_TRACER_H_

#include <cstddef>
#include <vector>

namespace lumice {

class Crystal;
class Rotation;

// Maximum vertices for a convex polygon in beam tracing.
// Ice crystal faces have at most ~6-8 vertices; S-H clipping adds at most 1 per clip edge.
// 24 provides ample headroom for pyramidal crystals with many faces.
constexpr size_t kMaxPolyVertices = 24;

// Minimum beam cross-section area. Beams smaller than this are discarded.
constexpr float kMinBeamArea = 1e-8f;


// 2D convex polygon with fixed-capacity vertex storage.
// Vertices are stored as [x0,y0, x1,y1, ...] in counter-clockwise order.
struct ConvexPolygon2D {
  float vertices[2 * kMaxPolyVertices]{};
  size_t count = 0;

  size_t VertexCount() const { return count; }
  float Area() const;  // Shoelace formula
};


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


// Build an orthonormal basis (u, v) perpendicular to direction d.
// d must be a unit vector. u and v are output as 3-float arrays.
void BuildOrthonormalBasis(const float* d, float* u, float* v);

// Project 3D points onto 2D plane defined by orthonormal basis (u, v).
// pts_3d: n points as [x0,y0,z0, x1,y1,z1, ...], pts_2d: output [u0,v0, u1,v1, ...]
void ProjectTo2D(const float* basis_u, const float* basis_v, const float* pts_3d, size_t n, float* pts_2d);

// Extract ordered polygon face vertices from Crystal's triangle mesh.
// Returns vertex count. out_vertices_3d must have space for 3*kMaxPolyVertices floats.
// NOTE: This reconstructs polygon vertices from triangulation — a tech-debt workaround.
// Ideally Crystal creation should preserve polygon vertex lists directly.
size_t ExtractPolygonFaceVertices(const Crystal& crystal, int poly_face_id, float* out_vertices_3d);

// Clip a 2D convex polygon by a half-plane: nx*x + ny*y + d >= 0.
// Returns the clipped polygon (may have 0 vertices if fully clipped).
ConvexPolygon2D ClipByHalfPlane(const ConvexPolygon2D& poly, float nx, float ny, float d);

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
