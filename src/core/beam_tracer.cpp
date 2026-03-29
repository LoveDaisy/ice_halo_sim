#include "core/beam_tracer.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>

#include "core/crystal.hpp"
#include "core/geo3d.hpp"
#include "core/math.hpp"
#include "core/optics.hpp"


namespace lumice {

// ============================================================================
// ConvexPolygon2D
// ============================================================================

float ConvexPolygon2D::Area() const {
  if (count < 3) {
    return 0.0f;
  }
  float area = 0.0f;
  for (size_t i = 0; i < count; i++) {
    size_t j = (i + 1) % count;
    float xi = vertices[2 * i];
    float yi = vertices[2 * i + 1];
    float xj = vertices[2 * j];
    float yj = vertices[2 * j + 1];
    area += xi * yj - xj * yi;
  }
  return std::abs(area) * 0.5f;
}


// ============================================================================
// Orthonormal basis & projection
// ============================================================================

void BuildOrthonormalBasis(const float* d, float* u, float* v) {
  // Pick the coordinate axis least parallel to d as the helper vector.
  float ax = std::abs(d[0]);
  float ay = std::abs(d[1]);
  float az = std::abs(d[2]);

  float helper[3]{};
  if (ax <= ay && ax <= az) {
    helper[0] = 1.0f;
  } else if (ay <= ax && ay <= az) {
    helper[1] = 1.0f;
  } else {
    helper[2] = 1.0f;
  }

  // u = normalize(helper × d)
  Cross3(helper, d, u);
  Normalize3(u);

  // v = d × u (already unit length since d and u are orthonormal)
  Cross3(d, u, v);
}


void ProjectTo2D(const float* basis_u, const float* basis_v, const float* pts_3d, size_t n, float* pts_2d) {
  for (size_t i = 0; i < n; i++) {
    const float* p = pts_3d + i * 3;
    pts_2d[2 * i] = Dot3(p, basis_u);
    pts_2d[2 * i + 1] = Dot3(p, basis_v);
  }
}


// ============================================================================
// Polygon face vertex extraction
// ============================================================================

size_t ExtractPolygonFaceVertices(const Crystal& crystal, int poly_face_id, float* out_vertices_3d) {
  const float* poly_n = crystal.GetPolygonFaceNormal();
  const float* tri_n = crystal.GetTriangleNormal();
  const float* tri_vtx = crystal.GetTriangleVtx();
  size_t tri_cnt = crystal.TotalTriangles();

  const float* target_n = poly_n + poly_face_id * 3;

  // Collect unique vertices from triangles sharing this face normal.
  float collected[3 * kMaxPolyVertices]{};
  size_t vtx_count = 0;

  for (size_t t = 0; t < tri_cnt; t++) {
    float dot = Dot3(target_n, tri_n + t * 3);
    if (dot < 1.0f - 1e-3f) {
      continue;
    }
    // This triangle belongs to the polygon face.
    for (int vi = 0; vi < 3; vi++) {
      const float* v = tri_vtx + t * 9 + vi * 3;
      // Check for duplicate.
      bool dup = false;
      for (size_t k = 0; k < vtx_count; k++) {
        float dx = collected[k * 3] - v[0];
        float dy = collected[k * 3 + 1] - v[1];
        float dz = collected[k * 3 + 2] - v[2];
        if (dx * dx + dy * dy + dz * dz < math::kFloatEps * math::kFloatEps) {
          dup = true;
          break;
        }
      }
      if (!dup && vtx_count < kMaxPolyVertices) {
        collected[vtx_count * 3] = v[0];
        collected[vtx_count * 3 + 1] = v[1];
        collected[vtx_count * 3 + 2] = v[2];
        vtx_count++;
      }
    }
  }

  if (vtx_count < 3) {
    return 0;
  }

  // Project vertices onto 2D using face normal as the projection axis.
  float basis_u[3]{};
  float basis_v[3]{};
  BuildOrthonormalBasis(target_n, basis_u, basis_v);

  float pts_2d[2 * kMaxPolyVertices]{};
  ProjectTo2D(basis_u, basis_v, collected, vtx_count, pts_2d);

  // Compute centroid in 2D.
  float cx = 0.0f;
  float cy = 0.0f;
  for (size_t i = 0; i < vtx_count; i++) {
    cx += pts_2d[2 * i];
    cy += pts_2d[2 * i + 1];
  }
  cx /= static_cast<float>(vtx_count);
  cy /= static_cast<float>(vtx_count);

  // Sort by angle around centroid (counter-clockwise).
  std::vector<size_t> idx(vtx_count);
  for (size_t i = 0; i < vtx_count; i++) {
    idx[i] = i;
  }
  std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
    float angle_a = std::atan2(pts_2d[2 * a + 1] - cy, pts_2d[2 * a] - cx);
    float angle_b = std::atan2(pts_2d[2 * b + 1] - cy, pts_2d[2 * b] - cx);
    return angle_a < angle_b;
  });

  // Output sorted 3D vertices.
  for (size_t i = 0; i < vtx_count; i++) {
    out_vertices_3d[i * 3] = collected[idx[i] * 3];
    out_vertices_3d[i * 3 + 1] = collected[idx[i] * 3 + 1];
    out_vertices_3d[i * 3 + 2] = collected[idx[i] * 3 + 2];
  }

  return vtx_count;
}


// ============================================================================
// Sutherland-Hodgman clipping
// ============================================================================

ConvexPolygon2D ClipByHalfPlane(const ConvexPolygon2D& poly, float nx, float ny, float d) {
  if (poly.count == 0) {
    return poly;
  }

  // S-H clipping adds at most 1 vertex per edge. For a convex polygon with N vertices,
  // one half-plane clip produces at most N+1 vertices. With kMaxPolyVertices=24, this is safe
  // for ice crystals (max ~10 polygon faces → max ~10 sequential clips from 6 vertices → ~16 vertices).
  assert(poly.count <= kMaxPolyVertices && "Input polygon exceeds kMaxPolyVertices");

  ConvexPolygon2D result{};

  for (size_t i = 0; i < poly.count; i++) {
    size_t j = (i + 1) % poly.count;
    float xi = poly.vertices[2 * i];
    float yi = poly.vertices[2 * i + 1];
    float xj = poly.vertices[2 * j];
    float yj = poly.vertices[2 * j + 1];

    float di = nx * xi + ny * yi + d;
    float dj = nx * xj + ny * yj + d;

    bool i_inside = di >= -math::kFloatEps;
    bool j_inside = dj >= -math::kFloatEps;

    if (i_inside && j_inside) {
      // Both inside: add j.
      if (result.count < kMaxPolyVertices) {
        result.vertices[2 * result.count] = xj;
        result.vertices[2 * result.count + 1] = yj;
        result.count++;
      }
    } else if (i_inside && !j_inside) {
      // i inside, j outside: add intersection.
      float t = di / (di - dj);
      if (result.count < kMaxPolyVertices) {
        result.vertices[2 * result.count] = xi + t * (xj - xi);
        result.vertices[2 * result.count + 1] = yi + t * (yj - yi);
        result.count++;
      }
    } else if (!i_inside && j_inside) {
      // i outside, j inside: add intersection + j.
      float t = di / (di - dj);
      if (result.count < kMaxPolyVertices) {
        result.vertices[2 * result.count] = xi + t * (xj - xi);
        result.vertices[2 * result.count + 1] = yi + t * (yj - yi);
        result.count++;
      }
      if (result.count < kMaxPolyVertices) {
        result.vertices[2 * result.count] = xj;
        result.vertices[2 * result.count + 1] = yj;
        result.count++;
      }
    }
    // Both outside: add nothing.
  }

  return result;
}


// ============================================================================
// Beam partitioning: determine which crystal faces a beam cross-section hits
// ============================================================================

namespace {

// Compute the intersection of two convex polygons using S-H clipping.
// Clips polygon `a` by each edge of polygon `b`.
// Both polygons must have CCW vertex order.
// Temporary location: will be moved to geo3d in task-refactor-sim.
ConvexPolygon2D IntersectConvexPolygons(const ConvexPolygon2D& a, const ConvexPolygon2D& b) {
  if (a.count < 3 || b.count < 3) {
    return {};
  }

  ConvexPolygon2D result = a;
  for (size_t i = 0; i < b.count; i++) {
    size_t j = (i + 1) % b.count;
    float ex = b.vertices[2 * j] - b.vertices[2 * i];
    float ey = b.vertices[2 * j + 1] - b.vertices[2 * i + 1];

    // Inward-pointing normal of edge (left side for CCW polygon): (-ey, ex)
    float nx = -ey;
    float ny = ex;
    // Half-plane: nx*(x - bx) + ny*(y - by) >= 0  =>  nx*x + ny*y + d >= 0
    float d = -(nx * b.vertices[2 * i] + ny * b.vertices[2 * i + 1]);

    result = ClipByHalfPlane(result, nx, ny, d);
    if (result.count < 3) {
      return {};
    }
  }
  return result;
}

struct PartitionResult {
  int target_face_id;
  ConvexPolygon2D polygon;
};


// For the beam traveling in direction `beam_dir`, find which polygon faces it can hit
// and partition the 2D cross-section accordingly.
// Uses face-plane projection: project each candidate face onto the beam's 2D cross-section,
// then intersect with the beam polygon. Convexity of the crystal guarantees non-overlapping partitions.
std::vector<PartitionResult> PartitionBeam(const BeamSegment& beam, const float* basis_u, const float* basis_v,
                                           const Crystal& crystal) {
  size_t poly_cnt = crystal.PolygonFaceCount();
  const float* poly_n = crystal.GetPolygonFaceNormal();

  // Find candidate faces: those where the beam is heading toward (dot(beam_dir, face_normal) > eps).
  // Exclude the current face.
  std::vector<int> candidates;
  for (size_t fi = 0; fi < poly_cnt; fi++) {
    if (static_cast<int>(fi) == beam.face_id) {
      continue;
    }
    float dot_dn = Dot3(beam.direction, poly_n + fi * 3);
    if (dot_dn > math::kFloatEps) {
      candidates.push_back(static_cast<int>(fi));
    }
  }

  if (candidates.empty()) {
    return {};
  }

  // If only one candidate, the entire beam hits that face.
  if (candidates.size() == 1) {
    return { { candidates[0], beam.cross_section } };
  }

  // Multiple candidates: project each face onto the beam's 2D cross-section and intersect.
  std::vector<PartitionResult> results;

  for (int fi : candidates) {
    // Extract 3D vertices of candidate face.
    float face_vtx_3d[3 * kMaxPolyVertices]{};
    size_t vtx_count = ExtractPolygonFaceVertices(crystal, fi, face_vtx_3d);
    if (vtx_count < 3) {
      continue;
    }

    // Project 3D vertices to beam's 2D space: u2d = v·bu, v2d = v·bv.
    // This is a parallel projection along the beam direction — the d-component is discarded.
    ConvexPolygon2D projected_poly{};
    projected_poly.count = vtx_count;
    for (size_t vi = 0; vi < vtx_count; vi++) {
      const float* v = face_vtx_3d + vi * 3;
      projected_poly.vertices[2 * vi] = Dot3(v, basis_u);
      projected_poly.vertices[2 * vi + 1] = Dot3(v, basis_v);
    }

    // Check winding order: ExtractPolygonFaceVertices produces CCW in face-normal view,
    // but projection to bu/bv may flip it. Compute signed area (shoelace without abs)
    // to detect CW winding (negative = CW) and reverse if needed.
    // Note: cannot use Area() here — it returns std::abs, always >= 0.
    float signed_area = 0.0f;
    for (size_t si = 0; si < vtx_count; si++) {
      size_t sj = (si + 1) % vtx_count;
      signed_area += projected_poly.vertices[2 * si] * projected_poly.vertices[2 * sj + 1] -
                     projected_poly.vertices[2 * sj] * projected_poly.vertices[2 * si + 1];
    }
    if (signed_area < 0) {
      for (size_t lo = 0, hi = vtx_count - 1; lo < hi; lo++, hi--) {
        std::swap(projected_poly.vertices[2 * lo], projected_poly.vertices[2 * hi]);
        std::swap(projected_poly.vertices[2 * lo + 1], projected_poly.vertices[2 * hi + 1]);
      }
    }

    // Intersect beam cross-section with projected face polygon.
    ConvexPolygon2D intersection = IntersectConvexPolygons(beam.cross_section, projected_poly);
    if (intersection.Area() > kMinBeamArea) {
      results.push_back({ fi, intersection });
    }
  }

  return results;
}

}  // namespace


// ============================================================================
// BeamTrace main function
// ============================================================================

BeamTraceResult BeamTrace(const Crystal& crystal, const Rotation& rot, const float* light_dir, float refractive_index,
                          size_t max_hits) {
  BeamTraceResult result{};

  size_t poly_cnt = crystal.PolygonFaceCount();
  if (poly_cnt == 0) {
    return result;
  }

  const float* poly_n = crystal.GetPolygonFaceNormal();

  // Transform light direction to crystal frame.
  float d_local[3] = { light_dir[0], light_dir[1], light_dir[2] };
  rot.ApplyInverse(d_local);

  // Build basis for the incoming light direction (shared across all entry faces).
  float basis_u[3]{};
  float basis_v[3]{};
  BuildOrthonormalBasis(d_local, basis_u, basis_v);

  // Find entry faces: faces where dot(d_local, face_normal) < 0 (facing the light).
  for (size_t fi = 0; fi < poly_cnt; fi++) {
    float cos_entry = Dot3(d_local, poly_n + fi * 3);
    if (cos_entry >= 0) {
      continue;  // Back-facing, not an entry face.
    }

    // Get polygon face vertices in 3D.
    float face_vtx_3d[3 * kMaxPolyVertices]{};
    size_t vtx_count = ExtractPolygonFaceVertices(crystal, static_cast<int>(fi), face_vtx_3d);
    if (vtx_count < 3) {
      continue;
    }

    // Project face vertices onto 2D (perpendicular to d_local).
    ConvexPolygon2D entry_poly{};
    ProjectTo2D(basis_u, basis_v, face_vtx_3d, vtx_count, entry_poly.vertices);
    entry_poly.count = vtx_count;

    float entry_area = entry_poly.Area();
    if (entry_area < kMinBeamArea) {
      continue;
    }

    // Projected entry area = geometric area * |cos(angle between face normal and light dir)|
    float proj_area = entry_area;  // Already in the projection plane perpendicular to d_local.
    result.total_entry_area += proj_area;

    // Compute Fresnel at entry face.
    const float* face_normal = poly_n + fi * 3;
    float cos_theta = Dot3(d_local, face_normal);  // Negative (light enters)
    float rr = 1.0f / refractive_index;            // Air → ice

    // Entry: reflected beam goes outward (outgoing), refracted beam enters crystal.
    float reflect_ratio_param = (1.0f - rr * rr) / (cos_theta * cos_theta) + rr * rr;
    float reflect_ratio = GetReflectRatio(std::max(reflect_ratio_param, 0.0f), rr);

    // Output reflected beam (bounces off entry face).
    {
      float refl_dir[3]{};
      ComputeReflectedDir(d_local, face_normal, refl_dir);

      // Transform back to world frame.
      rot.Apply(refl_dir);
      result.outgoing_d.push_back(refl_dir[0]);
      result.outgoing_d.push_back(refl_dir[1]);
      result.outgoing_d.push_back(refl_dir[2]);
      result.outgoing_w.push_back(reflect_ratio * proj_area);
      result.outgoing_raypath.push_back({ static_cast<int>(crystal.GetFn(crystal.GetPolygonFaceTriId()[fi])) });
    }

    // Refracted beam enters crystal.
    float refr_dir[3]{};
    bool refracted = ComputeRefractedDir(d_local, face_normal, rr, cos_theta, refr_dir);
    if (!refracted) {
      continue;  // Total reflection at entry (unusual but handle it).
    }

    float transmit_weight = (1.0f - reflect_ratio) * proj_area;

    // Build basis for the refracted direction.
    float inner_basis_u[3]{};
    float inner_basis_v[3]{};
    // Normalize refracted direction.
    Normalize3(refr_dir);
    BuildOrthonormalBasis(refr_dir, inner_basis_u, inner_basis_v);

    // Project entry polygon into the refracted beam's cross-section plane.
    ConvexPolygon2D inner_poly{};
    ProjectTo2D(inner_basis_u, inner_basis_v, face_vtx_3d, vtx_count, inner_poly.vertices);
    inner_poly.count = vtx_count;

    BeamSegment initial_beam{};
    std::memcpy(initial_beam.direction, refr_dir, sizeof(float) * 3);
    initial_beam.cross_section = inner_poly;
    initial_beam.weight = transmit_weight;
    initial_beam.face_id = static_cast<int>(fi);
    initial_beam.raypath.push_back(static_cast<int>(crystal.GetFn(crystal.GetPolygonFaceTriId()[fi])));

    // BFS through crystal interior. Termination by bounce depth (not dequeue count).
    std::deque<BeamSegment> queue;
    queue.push_back(std::move(initial_beam));

    while (!queue.empty()) {
      BeamSegment beam = std::move(queue.front());
      queue.pop_front();

      if (beam.depth >= max_hits) {
        continue;  // Exceeded maximum bounce depth.
      }

      // Build basis for this beam's direction.
      float bu[3]{};
      float bv[3]{};
      BuildOrthonormalBasis(beam.direction, bu, bv);

      // Partition beam cross-section among candidate next-faces.
      auto partitions = PartitionBeam(beam, bu, bv, crystal);

      for (auto& part : partitions) {
        const float* hit_normal = poly_n + part.target_face_id * 3;
        float hit_cos = Dot3(beam.direction, hit_normal);
        // Internal propagation: beam exits through face (hit_cos > 0 guaranteed by PartitionBeam).
        float hit_rr = refractive_index;  // Ice → air

        float delta = (1.0f - hit_rr * hit_rr) / (hit_cos * hit_cos) + hit_rr * hit_rr;
        bool total_reflection = delta <= 0.0f;
        float hit_reflect_ratio = GetReflectRatio(std::max(delta, 0.0f), hit_rr);

        float part_area = part.polygon.Area();
        int face_num = static_cast<int>(crystal.GetFn(crystal.GetPolygonFaceTriId()[part.target_face_id]));

        auto part_raypath = beam.raypath;
        part_raypath.push_back(face_num);

        // Weight invariant: beam.weight carries absolute area-weighted Fresnel transmission.
        // Each partition's share = beam.weight * (part_area / total_cross_section_area).
        if (!total_reflection) {
          // Refracted beam exits the crystal → outgoing.
          float exit_dir[3]{};
          ComputeRefractedDir(beam.direction, hit_normal, hit_rr, hit_cos, exit_dir);
          Normalize3(exit_dir);

          float exit_dir_world[3] = { exit_dir[0], exit_dir[1], exit_dir[2] };
          rot.Apply(exit_dir_world);

          float exit_weight = (1.0f - hit_reflect_ratio) * beam.weight * part_area / beam.cross_section.Area();
          result.outgoing_d.push_back(exit_dir_world[0]);
          result.outgoing_d.push_back(exit_dir_world[1]);
          result.outgoing_d.push_back(exit_dir_world[2]);
          result.outgoing_w.push_back(exit_weight);
          result.outgoing_raypath.push_back(part_raypath);
        }

        // Reflected beam continues inside the crystal.
        float refl_dir[3]{};
        ComputeReflectedDir(beam.direction, hit_normal, refl_dir);
        Normalize3(refl_dir);

        float cont_weight_fraction = total_reflection ? 1.0f : hit_reflect_ratio;
        float cont_weight = cont_weight_fraction * beam.weight * part_area / beam.cross_section.Area();

        if (cont_weight * part_area > kMinBeamArea * kMinBeamArea && beam.depth + 1 < max_hits) {
          // Re-project partition polygon into the reflected direction's cross-section plane.
          float rbu[3]{};
          float rbv[3]{};
          BuildOrthonormalBasis(refl_dir, rbu, rbv);

          // The partition polygon is in the old beam's 2D space. Convert to 3D then re-project.
          // 3D point = u2d * bu + v2d * bv (in crystal frame, relative to some origin on the beam axis).
          ConvexPolygon2D cont_poly{};
          for (size_t vi = 0; vi < part.polygon.count; vi++) {
            float u2d = part.polygon.vertices[2 * vi];
            float v2d = part.polygon.vertices[2 * vi + 1];
            float p3d[3]{};
            for (int j = 0; j < 3; j++) {
              p3d[j] = u2d * bu[j] + v2d * bv[j];
            }
            cont_poly.vertices[2 * vi] = Dot3(p3d, rbu);
            cont_poly.vertices[2 * vi + 1] = Dot3(p3d, rbv);
          }
          cont_poly.count = part.polygon.count;

          BeamSegment cont_beam{};
          std::memcpy(cont_beam.direction, refl_dir, sizeof(float) * 3);
          cont_beam.cross_section = cont_poly;
          cont_beam.weight = cont_weight;
          cont_beam.face_id = part.target_face_id;
          cont_beam.depth = beam.depth + 1;
          cont_beam.raypath = part_raypath;

          queue.push_back(std::move(cont_beam));
        }
      }
    }
  }

  return result;
}


}  // namespace lumice
