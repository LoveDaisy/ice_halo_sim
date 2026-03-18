#include "gui/crystal_preview.hpp"

#include <cmath>
#include <cstring>

#include "gui/gui_state.hpp"

namespace lumice::gui {

// Crystal preview trackball state
float g_crystal_rotation[16] = {
  1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
};
float g_crystal_zoom = 2.5f;
int g_crystal_style = 1;      // Default: Hidden Line (index into kCrystalStyleNames)
int g_crystal_mesh_id = -1;   // Crystal ID of cached mesh
int g_crystal_mesh_hash = 0;  // Hash of crystal params for change detection

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
  for (int i = 0; i < 3; i++) {
    h ^= c.upper_indices[i] * (47 + i);
    h ^= c.lower_indices[i] * (53 + i);
  }
  return h;
}

void ResetCrystalView() {
  // Default slightly elevated view (rotate +20 deg around X = tilt top away)
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
  g_crystal_zoom = 2.5f;
}

// Apply incremental rotation around axis (dx, dy) from mouse drag
void ApplyTrackballRotation(float dx, float dy) {
  float angle = std::sqrt(dx * dx + dy * dy) * 0.01f;
  if (angle < 1e-6f)
    return;

  // Rotation axis perpendicular to drag: "grab and move" model.
  // Drag right (dx>0) → axis +Y (up) → crystal turns right.
  // Drag down (dy>0) → axis +X (right) → crystal top comes toward viewer.
  float ax = dy / (angle / 0.01f);
  float ay = dx / (angle / 0.01f);
  float az = 0.0f;
  float len = std::sqrt(ax * ax + ay * ay + az * az);
  if (len < 1e-6f)
    return;
  ax /= len;
  ay /= len;
  az /= len;

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

  // new_rotation = r * g_crystal_rotation
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

}  // namespace lumice::gui
