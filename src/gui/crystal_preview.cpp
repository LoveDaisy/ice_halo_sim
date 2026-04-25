#include "gui/crystal_preview.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "lumice.h"

namespace lumice::gui {

// ----- Modal Singleton State Inventory (multi-instance refactor entry point) -----
// All file-scope state below is scoped to the single active edit modal. If the
// project grows to support multi-instance preview, every variable here needs
// to move into a per-instance context struct.
//   g_crystal_rotation / g_crystal_zoom / g_crystal_style  — trackball controls
//   g_crystal_mesh_hash                                    — upload-skip hash
//   g_last_mesh / g_last_mesh_valid                        — cached mesh for overlay

// Crystal preview trackball state
float g_crystal_rotation[16] = {
  1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
};
float g_crystal_zoom = kDefaultCrystalZoom;
int g_crystal_style = 1;      // Default: Hidden Line (index into kCrystalStyleNames)
int g_crystal_mesh_hash = 0;  // Hash of crystal params for change detection

// Cached mesh copy consumed by face-number overlay.
// vertices/edge_face_normals are already Y-Z swapped + AABB normalized
// (BuildCrystalMeshData modifies them in-place before returning).
static LUMICE_CrystalMesh g_last_mesh{};
static bool g_last_mesh_valid = false;

int CrystalParamHash(const CrystalConfig& c) {
  // Simple hash to detect parameter changes
  int h = static_cast<int>(c.type);
  auto hash_float = [](float f) {
    int i;
    std::memcpy(&i, &f, sizeof(i));
    return i;
  };
  h ^= hash_float(c.height) * 31;
  h ^= hash_float(c.prism_h) * 37;
  h ^= hash_float(c.upper_h) * 41;
  h ^= hash_float(c.lower_h) * 43;
  h ^= hash_float(c.upper_alpha) * 47;
  h ^= hash_float(c.lower_alpha) * 53;
  for (int i = 0; i < 6; i++) {
    h ^= hash_float(c.face_distance[i]) * (59 + i);
  }
  return h;
}

void ResetCrystalView() {
  // Legacy default view (+20 deg around X). Kept for callers that have no axis
  // preset in scope (main GUI init and test harness setup); modal Reset View
  // uses the AxisPreset overload below.
  constexpr float kAngle = 0.35f;  // ~20 degrees
  float c = std::cos(kAngle);
  float s = std::sin(kAngle);
  // Rotation around X axis
  g_crystal_rotation[0] = 1;
  g_crystal_rotation[1] = 0;
  g_crystal_rotation[2] = 0;
  g_crystal_rotation[3] = 0;
  g_crystal_rotation[4] = 0;
  g_crystal_rotation[5] = c;
  g_crystal_rotation[6] = s;
  g_crystal_rotation[7] = 0;
  g_crystal_rotation[8] = 0;
  g_crystal_rotation[9] = -s;
  g_crystal_rotation[10] = c;
  g_crystal_rotation[11] = 0;
  g_crystal_rotation[12] = 0;
  g_crystal_rotation[13] = 0;
  g_crystal_rotation[14] = 0;
  g_crystal_rotation[15] = 1;
  g_crystal_zoom = kDefaultCrystalZoom;
}

void ResetCrystalView(AxisPreset preset, const AxisDist params[3]) {
  DefaultPreviewRotation(preset, params, g_crystal_rotation);
  g_crystal_zoom = kDefaultCrystalZoom;
}

// Apply incremental rotation from mouse drag. Rotation is composed in world
// coordinates (model is left-multiplied by Rodrigues(world_axis, angle)) so the
// camera position stays fixed and the crystal turns relative to the user's
// mental model of world space.
//
// Coordinate convention: the GUI mesh frame has a Y-Z swap relative to core
// (see crystal_preview.cpp::BuildCrystalMeshData), so the user's "world +z up"
// corresponds to mesh +y, and "world -y forward (camera direction)" to mesh +z.
// The trackball axis is therefore expressed in mesh coordinates as:
//   drag right (dx>0) → axis = mesh +y (= world +z) → spin around world up
//   drag down  (dy>0) → axis = mesh +x (= world +x, camera right) → top of crystal toward user
void ApplyTrackballRotation(float dx, float dy) {
  float mag = std::sqrt(dx * dx + dy * dy);
  if (mag < 1e-4f) {
    return;  // ~ angle < 1e-6f under the 0.01 sensitivity scale
  }
  float ax = dy / mag;  // mesh +x component (= world +x, camera-right)
  float ay = dx / mag;  // mesh +y component (= world +z, vertical spin axis)
  float az = 0.0f;      // mesh +z (= world -y, camera-forward) — no roll
  float angle = mag * 0.01f;

  float ca = std::cos(angle);
  float sa = std::sin(angle);

  // Rodrigues rotation matrix (column-major)
  float r[16] = {};
  r[0] = ca + ax * ax * (1 - ca);
  r[1] = ay * ax * (1 - ca) + az * sa;
  r[2] = az * ax * (1 - ca) - ay * sa;
  r[4] = ax * ay * (1 - ca) - az * sa;
  r[5] = ca + ay * ay * (1 - ca);
  r[6] = az * ay * (1 - ca) + ax * sa;
  r[8] = ax * az * (1 - ca) + ay * sa;
  r[9] = ay * az * (1 - ca) - ax * sa;
  r[10] = ca + az * az * (1 - ca);
  r[15] = 1.0f;

  // g_crystal_rotation = r · g_crystal_rotation. Left-multiply means the new
  // rotation is composed in world coordinates (independent of current model
  // orientation). With CrystalRenderer's view = T·V_rot·model split, this
  // achieves "drag rotates the crystal around world axes; camera stays put".
  float tmp[16];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += r[i + k * 4] * g_crystal_rotation[k + j * 4];
      }
      tmp[i + j * 4] = sum;
    }
  }
  std::memcpy(g_crystal_rotation, tmp, sizeof(g_crystal_rotation));
}

bool BuildCrystalMeshData(const CrystalConfig& cr, LUMICE_CrystalMesh* out) {
  char json_buf[512];
  auto* fd = cr.face_distance;
  if (cr.type == CrystalType::kPrism) {
    snprintf(json_buf, sizeof(json_buf),
             R"({"type":"prism","shape":{"height":%.4f,)"
             R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
             cr.height, fd[0], fd[1], fd[2], fd[3], fd[4], fd[5]);
  } else {
    snprintf(json_buf, sizeof(json_buf),
             R"({"type":"pyramid","shape":{"prism_h":%.4f,"upper_h":%.4f,"lower_h":%.4f,)"
             R"("upper_wedge_angle":%.4f,"lower_wedge_angle":%.4f,)"
             R"("face_distance":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}})",
             cr.prism_h, cr.upper_h, cr.lower_h, cr.upper_alpha, cr.lower_alpha, fd[0], fd[1], fd[2], fd[3], fd[4],
             fd[5]);
  }

  *out = {};
  if (LUMICE_GetCrystalMesh(nullptr, json_buf, out) != LUMICE_OK) {
    return false;
  }

  // Y-Z swap: convert from core coordinate system to OpenGL view
  for (int vi = 0; vi < out->vertex_count; vi++) {
    float y = out->vertices[vi * 3 + 1];
    float z = out->vertices[vi * 3 + 2];
    out->vertices[vi * 3 + 1] = z;
    out->vertices[vi * 3 + 2] = -y;
  }
  for (int ei = 0; ei < out->edge_count; ei++) {
    for (int side = 0; side < 2; side++) {
      float* n = &out->edge_face_normals[ei * 6 + side * 3];
      float ny = n[1];
      float nz = n[2];
      n[1] = nz;
      n[2] = -ny;
    }
  }

  // AABB normalization: scale to fit unit cube
  if (out->vertex_count > 0) {
    float min_x = out->vertices[0], max_x = out->vertices[0];
    float min_y = out->vertices[1], max_y = out->vertices[1];
    float min_z = out->vertices[2], max_z = out->vertices[2];
    for (int vi = 1; vi < out->vertex_count; vi++) {
      float x = out->vertices[vi * 3];
      float y = out->vertices[vi * 3 + 1];
      float z = out->vertices[vi * 3 + 2];
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
      min_z = std::min(min_z, z);
      max_z = std::max(max_z, z);
    }
    float extent = std::max({ max_x - min_x, max_y - min_y, max_z - min_z });
    if (extent > 1e-6f) {
      float scale = 1.0f / extent;
      for (int vi = 0; vi < out->vertex_count; vi++) {
        out->vertices[vi * 3] *= scale;
        out->vertices[vi * 3 + 1] *= scale;
        out->vertices[vi * 3 + 2] *= scale;
      }
    }
  }

  return true;
}

int BuildAndUploadCrystalMesh(const CrystalConfig& cr) {
  LUMICE_CrystalMesh mesh{};
  if (!BuildCrystalMeshData(cr, &mesh)) {
    return 0;
  }

  g_crystal_renderer.UpdateMesh(mesh.vertices, mesh.vertex_count, mesh.edges, mesh.edge_count, mesh.triangles,
                                mesh.triangle_count, mesh.edge_face_normals);
  g_last_mesh = mesh;
  g_last_mesh_valid = true;
  return CrystalParamHash(cr);
}

const LUMICE_CrystalMesh* GetLastCrystalMesh() {
  return g_last_mesh_valid ? &g_last_mesh : nullptr;
}

void ResetLastCrystalMesh() {
  g_last_mesh_valid = false;
  g_last_mesh = {};
}

}  // namespace lumice::gui
