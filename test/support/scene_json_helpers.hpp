#pragma once

// Scene/config serialization probes shared by the import-export tests on both sides of the
// gui_test / gui_unit_test split. The logic cases moved to gui_unit_test; the GL and
// document-reset cases stayed in gui_test, and both halves assert through these helpers.
//
// Deliberately target-agnostic: no ImGui, no test engine, no test_gui_shared.hpp. Both targets
// already carry test/ on their include path (test/CMakeLists.txt and test/gui/CMakeLists.txt),
// so `#include "support/scene_json_helpers.hpp"` is one spelling valid in both — the same
// arrangement support/user_defaults_test_env.hpp already uses.

#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "lumice.h"

namespace lumice::test {

// ===== LUMICE_Scene test scaffolding (399.5) =====
// LUMICE_Scene is opaque, so the pre-handle "fill a LUMICE_Config and assert cfg.<field>"
// pattern becomes "BuildScene, serialize, assert on the JSON document". That is not a weaker
// assertion medium: LUMICE_SceneToJson dumps the scene's own internal document verbatim (the
// Scene IS a JSON tree — see LUMICE_Scene_ in c_api.cpp), so reading it is reading the scene's
// state, not a re-encoding of it.
//
// Scene ids are 0-based and assigned by the Scene itself (lumice.h "Incremental build"), where
// the pre-handle GUI code assigned crystals `pool_id + 1` and ran its own 1-based filter
// counter. Assertions on emitted ids therefore start at 0 — the ids are internal cross-
// references (scattering entry -> crystal/filter, composition term -> filter), never a
// user-visible or persisted value.
inline nlohmann::json SceneJson(const LUMICE_Scene* scene) {
  size_t len = 0;
  if (scene == nullptr || LUMICE_SceneToJson(scene, nullptr, 0, &len) != LUMICE_OK) {
    return nlohmann::json{};
  }
  std::string buf(len + 1, '\0');
  if (LUMICE_SceneToJson(scene, buf.data(), buf.size(), nullptr) != LUMICE_OK) {
    return nlohmann::json{};
  }
  buf.resize(len);
  return nlohmann::json::parse(buf);
}

// Build the commit-path scene for `state` and return its document. Returns a null json when
// BuildScene rejects the state (ABI overflow) — callers that care assert on that separately.
inline nlohmann::json CommitSceneJson(const gui::GuiState& state) {
  return SceneJson(gui::BuildScene(state, gui::SceneIntent::kSimCommit).get());
}

// Core-config JSON for `state` through the production export path (BuildExportJsonOrWarn).
// There is no separately-maintained JSON emitter any more: export JSON is produced by
// serializing the same LUMICE_Scene the commit path builds.
// Returns "" when the state exceeds the ABI bounds (the export reject path).
inline std::string CoreJson(const gui::GuiState& state) {
  std::string json;
  gui::BuildExportJsonOrWarn(state, &json, nullptr);
  return json;
}

// ===== Sync-group geometry probes (404.3) =====
// Deliberate copies of the helpers in test/unit-correctness/server/test_c_api.cpp (namespace-local
// there): that TU is not linked here, and lifting ~30 lines into a shared header for one mirrored
// pair would cost more than it saves. Kept behaviourally identical so a failure here means the
// same thing it means there.
//
// Signed plane offset (centroid · unit normal) of every prism face (face numbers 3..8), in
// face_number order. Measured off the geometry rather than assuming a face_number <->
// face_distance[i] mapping, so the assertions stay true statements about "same-group faces sit at
// the same distance" regardless of internal ordering. BuildCrystalMeshData's Y-Z swap is a
// rotation and its AABB normalization a uniform scale, so both preserve the equalities asserted.
inline std::vector<float> PrismFacePlaneOffsets(const LUMICE_CrystalMesh& mesh) {
  std::vector<float> offsets(6, std::numeric_limits<float>::quiet_NaN());
  for (int fi = 0; fi < mesh.face_count; ++fi) {
    const int fn = mesh.face_numbers_by_face[fi];
    if (fn < 3 || fn > 8) {
      continue;  // basal / pyramidal
    }
    const int offset = mesh.face_vtx_offsets[fi];
    const int count = mesh.face_vtx_counts[fi];
    if (count <= 0) {
      continue;
    }
    float c[3] = { 0.0f, 0.0f, 0.0f };
    for (int k = 0; k < count; ++k) {
      const float* p = mesh.vertices + mesh.face_vtx_pool[offset + k] * 3;
      c[0] += p[0];
      c[1] += p[1];
      c[2] += p[2];
    }
    const float* n = mesh.face_normals + fi * 3;
    offsets[fn - 3] = (c[0] * n[0] + c[1] * n[1] + c[2] * n[2]) / static_cast<float>(count);
  }
  return offsets;
}

// Number of distinct values in `v` under `tol`. Phrases "the three C3-equivalent faces collapsed
// onto one distance" without naming which faces those are.
inline size_t CountDistinct(const std::vector<float>& v, float tol) {
  std::vector<float> uniq;
  for (float x : v) {
    if (std::none_of(uniq.begin(), uniq.end(), [&](float u) { return std::fabs(u - x) <= tol; })) {
      uniq.push_back(x);
    }
  }
  return uniq.size();
}

}  // namespace lumice::test
