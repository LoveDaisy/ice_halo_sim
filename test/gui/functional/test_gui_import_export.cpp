#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/export_fbo_renderer.hpp"  // RenderExportToRgba for GL pixel-level assertions
#include "gui/raypath_segments.hpp"     // ParseSummandText / SumOfProducts (SoP round-trip tests)
#include "support/scene_json_helpers.hpp"
#include "test_gui_shared.hpp"

// SceneJson / CommitSceneJson / CoreJson / PrismFacePlaneOffsets / CountDistinct used to live in
// this file's anonymous namespace. The logic half of the suite moved to gui_unit_test
// (test/unit-correctness/gui/test_import_export_logic.cpp) and asserts through the same probes,
// so they were hoisted to a target-agnostic header instead of being copied into both halves.
using lumice::test::CommitSceneJson;
using lumice::test::CoreJson;
using lumice::test::CountDistinct;
using lumice::test::PrismFacePlaneOffsets;
using lumice::test::SceneJson;

// task-349.4 wait-until-condition helper — bounded polling that yields to the main render
// thread until `condition()` returns true or `max_yields` frames elapse. Predicates should
// mirror the immediately-following IM_CHECK so a real production regression (condition
// never becomes true) still surfaces as a failure at the same assertion line, just after
// waiting up to the bound instead of after a fixed frame count.
namespace {
constexpr int kDoOpenSettleYieldLimit = 60;

template <typename Fn>
void YieldUntilTrue(ImGuiTestContext* ctx, int max_yields, Fn&& condition) {
  for (int i = 0; i < max_yields && !condition(); ++i) {
    ctx->Yield();
  }
}

// task-350: shared request/response scaffolding for the "gl_render_*" subcases (see the
// block at the end of RegisterImportExportTests). Kept out here so GlOpGuiFunc below is a
// plain non-capturing function — ImGuiTestGuiFunc is `void (*)(ImGuiTestContext*)` and
// rejects capturing lambdas.
struct GlOpTestState {
  bool requested = false;
  bool done = false;
  // Inputs (TestFunc → GuiFunc): sequence of GL ops to run this frame.
  bool do_upload_stale = false;
  bool do_upload_bg = false;
  bool do_clear = false;
  bool do_upload_fresh_after_clear = false;
  bool bg_enabled = false;
  int dst_w = 32;
  int dst_h = 16;
  // Outputs (GuiFunc → TestFunc).
  bool export_ok = false;
  unsigned char center_r = 0;
  unsigned char center_g = 0;
  unsigned char center_b = 0;
  void Reset() { *this = GlOpTestState{}; }
};

GlOpTestState g_gl_op;

// Stable fills. Chosen so a single center-pixel channel check discriminates
// {stale sim} vs {black post-clear} vs {bg composited} vs {fresh upload} with no tolerance.
constexpr unsigned char kStaleMagenta[3] = { 0xFF, 0x00, 0xFF };  // r=255, g=0,   b=255
constexpr unsigned char kFreshCyan[3] = { 0x00, 0xFF, 0xFF };     // r=0,   g=255, b=255
constexpr unsigned char kBgGreen[3] = { 0x00, 0xFF, 0x00 };       // r=0,   g=255, b=0

void FillSolid(std::vector<unsigned char>& buf, int w, int h, const unsigned char rgb[3]) {
  buf.resize(static_cast<size_t>(w) * h * 3);
  for (int i = 0; i < w * h; ++i) {
    buf[i * 3 + 0] = rgb[0];
    buf[i * 3 + 1] = rgb[1];
    buf[i * 3 + 2] = rgb[2];
  }
}

// GuiFunc body: dispatch requested GL operations in a single main-thread frame so the
// interleaved "clear then upload before any Render()" case is exercised without an
// intervening PreviewRenderer::Render() — RenderExportToRgba's internal Render() is the
// sole Render() call in the request-fulfillment window.
void GlOpGuiFunc(ImGuiTestContext*) {
  if (!g_gl_op.requested || g_gl_op.done)
    return;
  const int tex_w = 16;
  const int tex_h = 8;
  std::vector<unsigned char> pixels;
  if (g_gl_op.do_upload_stale) {
    FillSolid(pixels, tex_w, tex_h, kStaleMagenta);
    lumice::gui::g_preview.UploadTexture(pixels.data(), tex_w, tex_h);
  }
  if (g_gl_op.do_upload_bg) {
    FillSolid(pixels, tex_w, tex_h, kBgGreen);
    lumice::gui::g_preview.UploadBgTexture(pixels.data(), tex_w, tex_h);
  }
  if (g_gl_op.do_clear) {
    lumice::gui::g_preview.ClearTexture();
  }
  if (g_gl_op.do_upload_fresh_after_clear) {
    FillSolid(pixels, tex_w, tex_h, kFreshCyan);
    lumice::gui::g_preview.UploadTexture(pixels.data(), tex_w, tex_h);
  }
  // Rectangular/equirect lens fills the full viewport with texture samples, so a uniformly
  // colored source texture yields a uniformly colored output — center-pixel probe suffices
  // (no fisheye visibility cutout at frame center).
  lumice::gui::PreviewParams params;
  lumice::gui::ConfigureEquirectExportParams(params);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // RGB (non-XYZ) mode: texture sampled as-is
  if (g_gl_op.bg_enabled) {
    params.bg.enabled = true;
    params.bg.alpha = 0.0f;  // shader: bg * (1 - alpha) + sim * alpha — alpha=0 = pure bg
    params.bg.aspect = static_cast<float>(tex_w) / static_cast<float>(tex_h);
  }
  auto rgba =
      lumice::gui::RenderExportToRgba(lumice::gui::g_preview, params, g_gl_op.dst_w, g_gl_op.dst_h, std::nullopt);
  g_gl_op.export_ok = !rgba.empty();
  if (g_gl_op.export_ok) {
    // RenderExportToRgba returns RGBA8, row-major, top-down (matches stbi_write_png).
    const int cx = g_gl_op.dst_w / 2;
    const int cy = g_gl_op.dst_h / 2;
    const size_t off = (static_cast<size_t>(cy) * g_gl_op.dst_w + cx) * 4;
    g_gl_op.center_r = rgba[off + 0];
    g_gl_op.center_g = rgba[off + 1];
    g_gl_op.center_b = rgba[off + 2];
  }
  g_gl_op.done = true;
}
}  // namespace

// ========== Import/Export Tests ==========

void RegisterImportExportTests(ImGuiTestEngine* engine) {
  // Test 4: Full .lmc roundtrip with all EntryCard fields
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "lmc_full_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Set up a state with multiple entries, filter, and pyramid crystal
      auto& entry0 = gui::g_state.layers[0].entries[0];
      gui::CrystalOf(gui::g_state, entry0).type = gui::CrystalType::kPyramid;
      gui::CrystalOf(gui::g_state, entry0).prism_h = 2.0f;
      gui::CrystalOf(gui::g_state, entry0).upper_h = 0.3f;
      gui::CrystalOf(gui::g_state, entry0).lower_h = 0.4f;
      // Non-default face_distance to exercise the conditional-write branch in SerializeCrystal.
      gui::CrystalOf(gui::g_state, entry0).face_distance[0] = 1.2f;
      gui::CrystalOf(gui::g_state, entry0).face_distance[3] = 0.9f;
      // Non-default axis distributions to verify axis round-trip on all three axes.
      // All three use the same SerializeAxisDist/FillAxisDist path, but covering each
      // individually guards against future per-axis special-casing (e.g. roll default-omission).
      gui::CrystalOf(gui::g_state, entry0).zenith = gui::AxisDist{ gui::AxisDistType::kGauss, 25.0f, 3.0f };
      gui::CrystalOf(gui::g_state, entry0).azimuth = gui::AxisDist{ gui::AxisDistType::kUniform, 10.0f, 20.0f };
      gui::CrystalOf(gui::g_state, entry0).roll = gui::AxisDist{ gui::AxisDistType::kLaplacian, 5.0f, 2.0f };
      entry0.proportion = 75.0f;
      gui::FilterConfig f;
      f.SetRaypath(gui::RaypathParams{ "3-1-5" });
      gui::SetFilter(gui::g_state, entry0, f);

      // Add a second entry with its own crystal pool slot (ID-pool model).
      gui::CrystalConfig extra_crystal;
      extra_crystal.type = gui::CrystalType::kPrism;
      extra_crystal.height = 3.0f;
      gui::EntryCard extra;
      extra.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      gui::g_state.crystals.push_back(extra_crystal);
      extra.proportion = 25.0f;
      gui::g_state.layers[0].entries.push_back(extra);

      // Save
      const std::string tmp_path = GuiTestTempPath("lumice_full_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset and load
      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      // Verify all fields survived round-trip
      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      auto& loaded0 = gui::g_state.layers[0].entries[0];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).type, gui::CrystalType::kPyramid);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).prism_h, 2.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).upper_h, 0.3f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).lower_h, 0.4f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).face_distance[0], 1.2f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).face_distance[3], 0.9f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.type, gui::AxisDistType::kGauss);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.mean, 25.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).zenith.std, 3.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.type, gui::AxisDistType::kUniform);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.mean, 10.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).azimuth.std, 20.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.type, gui::AxisDistType::kLaplacian);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.mean, 5.0f);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded0).roll.std, 2.0f);
      IM_CHECK_EQ(loaded0.proportion, 75.0f);
      IM_CHECK(loaded0.filter_id.has_value());
      IM_CHECK_EQ(gui::g_state.filters[*loaded0.filter_id].RaypathText(), std::string("3-1-5"));

      auto& loaded1 = gui::g_state.layers[0].entries[1];
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded1).type, gui::CrystalType::kPrism);
      IM_CHECK_EQ(gui::CrystalOf(gui::g_state, loaded1).height, 3.0f);
      IM_CHECK_EQ(loaded1.proportion, 25.0f);

      std::remove(tmp_path.c_str());
    };
  }

  // Test: crystal shape RANDOMIZATION round-trip (AC1). Guards the data-loss defect this task
  // fixes — pre-fix, file_io read a {type,mean,std} shape distribution as a bare mean and wrote it
  // back as a scalar, so a CLI-configured randomization silently vanished on a GUI load→save. The
  // config below carries a randomized height AND a per-face heterogeneous face_distance (face[0]
  // Uniform, face[3] Gauss, one randomized face whose center happens to equal the default 1.0 —
  // the boundary that a center-only "is default" check would wrongly drop). Round-trips through the
  // real SaveLmcFile/LoadLmcFile production path (no hand-assembled JSON, per issue hard constraint)
  // and asserts every {type,center,spread} component survives.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "shape_load_downgrades_nonuniform_to_uniform");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // The GUI edits UNIFORM shape distributions only. A file authored by JSON/CLI may carry richer
      // families (gauss/zigzag/laplacian/gauss_legacy); on load those are downgraded to uniform,
      // keeping the center/spread values, and the downgrade is counted for the load-complete notice.
      // NO_RANDOM (off) and UNIFORM pass through unchanged. This is a deliberate, notified capability
      // downgrade (owner decision) — not a silent loss (the earlier bug collapsed a loaded distribution
      // to its bare mean, dropping type/std). This test injects a rich distribution set (simulating a
      // JSON file) and asserts the downgrade contract.
      auto& entry0 = gui::g_state.layers[0].entries[0];
      auto& cr = gui::CrystalOf(gui::g_state, entry0);
      cr.type = gui::CrystalType::kPrism;
      cr.height = gui::ShapeDist{ gui::ShapeDistType::kGauss, 2.5f, 0.4f };
      cr.face_distance[0] = gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.2f, 0.3f };
      cr.face_distance[1] = gui::ShapeDist{ gui::ShapeDistType::kNoRandom, 0.8f, 0.0f };
      cr.face_distance[2] = gui::ShapeDist{ gui::ShapeDistType::kZigzag, 1.0f, 0.15f };  // center == default 1.0
      cr.face_distance[3] = gui::ShapeDist{ gui::ShapeDistType::kGauss, 0.9f, 0.05f };
      cr.face_distance[4] = gui::ShapeDist{ gui::ShapeDistType::kLaplacian, 1.1f, 0.2f };
      cr.face_distance[5] = gui::ShapeDist{ gui::ShapeDistType::kGaussLegacy, 1.3f, 0.25f };

      const std::string tmp_path = GuiTestTempPath("lumice_shape_rand_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      gui::TakeShapeDistDowngradeCount();  // discard any stale count from prior parses
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      auto& loaded = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      // height (was Gauss) → Uniform, center/spread preserved.
      IM_CHECK(loaded.height == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 2.5f, 0.4f }));
      // face_distance: Uniform/NO_RANDOM pass through; the four non-uniform families → Uniform (values kept).
      IM_CHECK(loaded.face_distance[0] == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.2f, 0.3f }));
      IM_CHECK(loaded.face_distance[1] == (gui::ShapeDist{ gui::ShapeDistType::kNoRandom, 0.8f, 0.0f }));
      IM_CHECK(loaded.face_distance[2] == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.0f, 0.15f }));
      IM_CHECK(loaded.face_distance[3] == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 0.9f, 0.05f }));
      IM_CHECK(loaded.face_distance[4] == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.1f, 0.2f }));
      IM_CHECK(loaded.face_distance[5] == (gui::ShapeDist{ gui::ShapeDistType::kUniform, 1.3f, 0.25f }));
      // Exactly the five non-uniform fields (height + faces 2/3/4/5) were downgraded and counted.
      IM_CHECK_EQ(gui::TakeShapeDistDowngradeCount(), 5);

      std::remove(tmp_path.c_str());
    };
  }

  // ===================== shape-scalar sync groups (404.3) =====================
  // The GUI carries sync groups but grows no control for them (that is 404.4), so these tests
  // drive the data path directly: import → preview, save → load, and the preview's change
  // detection. The three of them are what makes "a hand-authored config with sync_group works in
  // the GUI" a checked statement rather than a claim.

  // AC2: .lmc round-trip, every applicable slot. Two crystals because the wire form is type-gated
  // exactly as the shape scalars themselves are: a prism writes height + face_distance, a pyramid
  // writes prism_h/upper_h/lower_h + face_distance. Between them all ten LUMICE_SHAPE_SCALAR_*
  // slots are exercised — a per-slot sweep, because a table that drops one key shows up only as
  // "that one field's grouping vanished" (the failure mode 404.2 hit as key-name drift).
  // The pyramid arm also guards the index-order trap: UPPER_H is slot 1 and PRISM_H slot 2, the
  // reverse of the struct's field order, so distinct values here catch a positional mapping.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "sync_group_lmc_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      auto& prism = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      prism.type = gui::CrystalType::kPrism;
      prism.height.sync_group = 5;
      for (int i = 0; i < 6; ++i) {
        prism.face_distance[i].sync_group = (i % 2 == 0) ? 1 : 2;
      }

      gui::CrystalConfig pyr;
      pyr.type = gui::CrystalType::kPyramid;
      pyr.prism_h.sync_group = 3;
      pyr.upper_h.sync_group = 4;  // != prism_h: a positional mapping swaps these two
      pyr.lower_h.sync_group = 3;
      for (int i = 0; i < 6; ++i) {
        pyr.face_distance[i].sync_group = 6 - i;
      }
      gui::EntryCard extra;
      extra.crystal_id = static_cast<int>(gui::g_state.crystals.size());
      extra.proportion = 50.0f;
      gui::g_state.crystals.push_back(pyr);
      gui::g_state.layers[0].entries.push_back(extra);

      // Key names are asserted against the emitted document, not just round-trip stability: a
      // round-trip is self-consistent even if the GUI writes keys core cannot read (the third
      // mirror table's failure mode — there is no compile-time link between the three copies).
      const auto doc = nlohmann::json::parse(gui::SerializeGuiStateJson(gui::g_state));
      // Presence first: without it a serializer that emits nothing fails by throwing out of
      // operator[] instead of reporting which expectation broke.
      IM_CHECK(doc["layers"][0]["entries"][0]["crystal"]["shape"].contains("sync_group"));
      IM_CHECK(doc["layers"][0]["entries"][1]["crystal"]["shape"].contains("sync_group"));
      const auto& prism_sg = doc["layers"][0]["entries"][0]["crystal"]["shape"]["sync_group"];
      IM_CHECK_EQ(prism_sg["height"].get<int>(), 5);
      IM_CHECK_EQ(prism_sg["face_distance"][2].get<int>(), 1);
      IM_CHECK(!prism_sg.contains("prism_h"));  // inapplicable to this type, as for the scalars
      const auto& pyr_sg = doc["layers"][0]["entries"][1]["crystal"]["shape"]["sync_group"];
      IM_CHECK_EQ(pyr_sg["prism_h"].get<int>(), 3);
      IM_CHECK_EQ(pyr_sg["upper_h"].get<int>(), 4);
      IM_CHECK_EQ(pyr_sg["lower_h"].get<int>(), 3);
      IM_CHECK_EQ(pyr_sg["face_distance"][0].get<int>(), 6);
      IM_CHECK(!pyr_sg.contains("height"));

      const std::string tmp_path = GuiTestTempPath("lumice_sync_group_roundtrip.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false));
      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      IM_CHECK(gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h));

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);
      const auto& loaded_prism = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[0]);
      IM_CHECK_EQ(loaded_prism.height.sync_group, 5);
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(loaded_prism.face_distance[i].sync_group, (i % 2 == 0) ? 1 : 2);
      }
      const auto& loaded_pyr = gui::CrystalOf(gui::g_state, gui::g_state.layers[0].entries[1]);
      IM_CHECK_EQ(loaded_pyr.prism_h.sync_group, 3);
      IM_CHECK_EQ(loaded_pyr.upper_h.sync_group, 4);
      IM_CHECK_EQ(loaded_pyr.lower_h.sync_group, 3);
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(loaded_pyr.face_distance[i].sync_group, 6 - i);
      }

      std::remove(tmp_path.c_str());
    };
  }

  // AC1 of the Sync column: a C3 grouping BUILT BY CLICKING reaches the geometry. The 404.3 tests
  // above inject sync_group into the config and check the data path; this one starts from an
  // all-independent crystal and drives the actual widget — swatch button, then popup item — the way
  // a user would, then asserts on the mesh the simulator would trace.
  //
  // It lives in this file, not next to the other Sync-column tests in test_gui_interaction.cpp,
  // because gui_test is ONE binary (test/gui/CMakeLists.txt compiles both into add_executable):
  // the PrismFacePlaneOffsets / CountDistinct probes are already defined in this translation unit,
  // and this repo copies a test helper only across binaries, never within one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "sync_group_built_in_ui_reaches_geometry");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.modal_immediate_mode = false;  // OK is what commits; this is the full user path
      ctx->Yield(2);
      auto& entry = gui::g_state.layers[0].entries[0];

      ctx->ItemClick("**/Edit##cr");
      ctx->Yield(4);
      ctx->ItemOpen("**/Face Distance##modal");
      ctx->Yield(2);

      // Randomize all six faces first. Without randomization every face keeps the same default
      // 1.0 and "grouped faces are equal" would hold for a build that ignored sync_group entirely.
      for (int i = 0; i < 6; ++i) {
        char rnd_id[64];
        snprintf(rnd_id, sizeof(rnd_id), "**/##rnd_Face %d##modal_fd", i + 3);
        ctx->ItemClick(rnd_id);
        ctx->Yield(2);
      }

      // Face 3/5/7 (indices 0/2/4) → group 1, Face 4/6/8 (1/3/5) → group 2. Each group is opened
      // with "+ New group" on its first member and joined via its "###sync_group_N" item after that.
      const int kGroupMembers[2][3] = { { 3, 5, 7 }, { 4, 6, 8 } };
      for (int g = 0; g < 2; ++g) {
        for (int m = 0; m < 3; ++m) {
          char swatch[64];
          snprintf(swatch, sizeof(swatch), "**/##sync_Face %d##modal_fd", kGroupMembers[g][m]);
          ctx->ItemClick(swatch);
          ctx->Yield(2);
          if (m == 0) {
            ctx->ItemClick("**/###sync_new");
          } else {
            char item[64];
            snprintf(item, sizeof(item), "**/###sync_group_%d", g + 1);
            ctx->ItemClick(item);
          }
          ctx->Yield(2);
        }
      }

      ctx->ItemClose("**/Face Distance##modal");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(3);

      // The committed entry carries the grouping the clicks described.
      const auto& cr = gui::CrystalOf(gui::g_state, entry);
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(cr.face_distance[i].sync_group, (i % 2 == 0) ? 1 : 2);
        IM_CHECK_EQ(cr.face_distance[i].type, gui::ShapeDistType::kUniform);
      }
      // ...and so does the scene handed to core: face_distance's sync_group array is the canonical
      // [1,2,1,2,1,2]. Read off the commit path, not off g_state again, because the grouping being
      // present in the GUI and reaching the simulator are separate claims (404.1 found the CLI path
      // dropping sync_group while the GUI-side data was perfectly correct).
      const auto scene_j = CommitSceneJson(gui::g_state);
      const auto& sg = scene_j["crystal"][0]["shape"]["sync_group"]["face_distance"];
      for (int i = 0; i < 6; ++i) {
        IM_CHECK_EQ(sg[i].get<int>(), (i % 2 == 0) ? 1 : 2);
      }

      // White-box on the geometry: two draws, six faces. The eye cannot verify this symmetry in a
      // halo image, which is the entire reason the feature exists — so it is asserted on the mesh.
      LUMICE_CrystalMesh mesh{};
      IM_CHECK(gui::BuildCrystalMeshData(cr, 12345, &mesh));
      const auto off = PrismFacePlaneOffsets(mesh);
      IM_CHECK_EQ(CountDistinct(off, 1e-5f), (size_t)2);
      IM_CHECK(std::fabs(off[0] - off[2]) <= 1e-5f);
      IM_CHECK(std::fabs(off[2] - off[4]) <= 1e-5f);
      IM_CHECK(std::fabs(off[1] - off[3]) <= 1e-5f);
      IM_CHECK(std::fabs(off[3] - off[5]) <= 1e-5f);
      // The two groups drew separately: a mesh where all six faces collapsed to one value would
      // satisfy every equality above.
      IM_CHECK(std::fabs(off[0] - off[1]) > 1e-5f);

      // Control on the same randomized crystal: ungrouped, the six faces draw six distinct values.
      // Without it, "2 distinct" would also pass on a build that ignored face_distance randomization.
      gui::CrystalConfig ungrouped = cr;
      for (int i = 0; i < 6; ++i) {
        ungrouped.face_distance[i].sync_group = 0;
      }
      LUMICE_CrystalMesh independent{};
      IM_CHECK(gui::BuildCrystalMeshData(ungrouped, 12345, &independent));
      IM_CHECK_EQ(CountDistinct(PrismFacePlaneOffsets(independent), 1e-5f), (size_t)6);
    };
  }

  // Test: custom-spectrum end-to-end round-trip (task-323).
  // Covers all 4 file_io write paths + 1 load path introduced by the discrete-spectrum work:
  //   1. GUI project save (root["sun"]["spectrum"]="custom" + "custom_spectrum" array)
  //   2. GUI project load (reads back the array, restores kCustomSpectrumIndex)
  //   3. export JSON (light_source.spectrum emitted as a JSON array)
  //   4. commit scene (same array-form spectrum reaches the simulator)
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "custom_spectrum_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Configure a 3-entry custom spectrum.
      gui::g_state.sun.spectrum_index = gui::kCustomSpectrumIndex;
      gui::g_state.sun.custom_spectrum = { { 450.0f, 0.5f }, { 550.0f, 1.0f }, { 650.0f, 0.7f } };

      // (1)+(2) .lmc save/load round-trip.
      const std::string tmp_path = GuiTestTempPath("lumice_custom_spectrum_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);
      gui::DoNew();
      IM_CHECK(gui::g_state.sun.spectrum_index != gui::kCustomSpectrumIndex);  // reset baseline
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::g_state.sun.spectrum_index, gui::kCustomSpectrumIndex);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum.size(), (size_t)3);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[0].wavelength, 450.0f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[0].weight, 0.5f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[1].wavelength, 550.0f);
      IM_CHECK_EQ(gui::g_state.sun.custom_spectrum[2].weight, 0.7f);
      std::remove(tmp_path.c_str());

      // (3) The export path emits an array-form spectrum. Matches core light_config.cpp
      // SpectrumToJson shape ([{wavelength, weight}, ...]).
      std::string core_json = CoreJson(gui::g_state);
      auto parsed = nlohmann::json::parse(core_json);
      const auto& spec = parsed["scene"]["light_source"]["spectrum"];
      IM_CHECK(spec.is_array());
      IM_CHECK_EQ(spec.size(), (size_t)3);
      IM_CHECK_EQ(spec[0]["wavelength"].get<float>(), 450.0f);
      IM_CHECK_EQ(spec[1]["weight"].get<float>(), 1.0f);
      IM_CHECK_EQ(spec[2]["wavelength"].get<float>(), 650.0f);

      // (4) BuildScene emits the discrete spectrum on the commit path too (the discrete list
      // overrides the spectrum name string, matching the core config's own rule).
      const auto scene_j = CommitSceneJson(gui::g_state);
      const auto& commit_spec = scene_j["scene"]["light_source"]["spectrum"];
      IM_CHECK(commit_spec.is_array());
      IM_CHECK_EQ(commit_spec.size(), (size_t)3);
      IM_CHECK_EQ(commit_spec[0]["wavelength"].get<float>(), 450.0f);
      IM_CHECK_EQ(commit_spec[0]["weight"].get<float>(), 0.5f);
      IM_CHECK_EQ(commit_spec[2]["wavelength"].get<float>(), 650.0f);
    };
  }

  // Test: modal_layout_vertical roundtrip.
  // Ensures the new .lmc field survives save/load and defaults correctly on legacy files.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "modal_layout_vertical_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);  // ResetTestState pins legacy false

      // Save with horizontal (false) — distinguishable from the new production
      // default (true) so the load step below can prove it was actually read.
      const std::string tmp_path = GuiTestTempPath("lumice_modal_layout_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // Reset to the new production default via DoNew (bypasses ResetTestState pin).
      gui::DoNew();
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, true);  // new production default
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);
      IM_CHECK_EQ(gui::g_state.modal_layout_vertical, false);  // saved value survives

      std::remove(tmp_path.c_str());
    };
  }

  // Per-type GUI JSON round-trip — every
  // FilterParamVariant alternative survives SerializeFilterForGui →
  // ParseFilterFromGuiJson via the .lmc save/load path.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "per_type_filter_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // Build one entry per filter type.
      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;

      auto make_entry = [](gui::Factor param) {
        // ID-pool model: each entry needs its own crystal + filter pool slot.
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::EntryCard e;
        e.crystal_id = static_cast<int>(gui::g_state.crystals.size());
        gui::g_state.crystals.push_back(c);
        e.proportion = 25.0f;
        gui::FilterConfig f;
        if (std::holds_alternative<gui::RaypathParams>(param)) {
          f.SetRaypath(std::get<gui::RaypathParams>(std::move(param)));
        } else {
          f.SetEntryExit(std::get<gui::EntryExitParams>(std::move(param)));
        }
        gui::SetFilter(gui::g_state, e, f);
        return e;
      };

      // Build one entry per remaining filter type (raypath, entry_exit).
      layer.entries.push_back(make_entry(gui::RaypathParams{ "3-1-5" }));
      layer.entries.push_back(make_entry(gui::EntryExitParams{ "7", "4" }));
      gui::g_state.layers.push_back(layer);

      const std::string tmp_path = GuiTestTempPath("lumice_per_type_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), 2);

      const auto& f0 = gui::g_state.filters[*gui::g_state.layers[0].entries[0].filter_id];
      IM_CHECK(f0.IsRaypath());
      IM_CHECK_EQ(f0.RaypathText(), std::string("3-1-5"));

      const auto& f1 = gui::g_state.filters[*gui::g_state.layers[0].entries[1].filter_id];
      IM_CHECK(f1.IsEntryExit());
      const auto& ee = f1.EntryExitParamsValue();
      IM_CHECK_EQ(ee.entry_text, std::string("7"));
      IM_CHECK_EQ(ee.exit_text, std::string("4"));

      std::remove(tmp_path.c_str());
    };
  }

  // T2: any sum-of-products (cross-type
  // OR + AND + internal multi-value + multiple rows) survives GUI -> .lmc -> GUI
  // with FilterConfig::operator== (text-layer) equality. Covers SerializeFilterForGui
  // ("summands") and ParseFilterFromGuiJson's new-form reader.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "sop_lmc_roundtrip");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      IM_UNUSED(ctx);
      ResetTestState();

      // One entry per SoP filter shape. Each filter is built from canonical row
      // texts (the operator== identity), so the round-trip must reproduce them.
      const std::vector<std::vector<std::string>> shapes = {
        { "3-1-5" },                       // degenerate single raypath
        { "3-5", "1-3", "2-6" },           // multi-row raypath OR
        { "entry:7 & exit:4" },            // single EE
        { "entry:3 & exit:5 & len:2-6" },  // EE with length range
        { "entry:", "exit:5" },            // wildcard EE rows
        { "3-1-5", "entry:3 & exit:5" },   // cross-type OR
        { "entry:3 & 7-1" },               // AND clause (EE + raypath)
        { "entry:3,4 & 7-1", "2-6" },      // cross-type OR + AND + internal multi
      };

      gui::g_state.layers.clear();
      gui::Layer layer;
      layer.probability = 0.0f;
      std::vector<gui::FilterConfig> originals;
      for (const auto& rows : shapes) {
        gui::CrystalConfig c;
        c.type = gui::CrystalType::kPrism;
        c.height = 1.0f;
        for (int i = 0; i < 6; ++i) {
          c.face_distance[i] = 1.0f;
        }
        gui::EntryCard e;
        e.crystal_id = static_cast<int>(gui::g_state.crystals.size());
        gui::g_state.crystals.push_back(c);
        e.proportion = 100.0f;
        gui::FilterConfig f;
        gui::SumOfProducts sop;
        for (const auto& row : rows) {
          sop.push_back(gui::SummandText{ row, gui::ParseSummandText(row) });
        }
        f.param = std::move(sop);
        originals.push_back(f);
        gui::SetFilter(gui::g_state, e, f);
        layer.entries.push_back(e);
      }
      gui::g_state.layers.push_back(layer);

      const std::string tmp_path = GuiTestTempPath("lumice_sop_roundtrip.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      gui::DoNew();
      std::vector<unsigned char> tex_data;
      int tex_w = 0;
      int tex_h = 0;
      bool load_ok = gui::LoadLmcFile(tmp_path, gui::g_state, tex_data, tex_w, tex_h);
      IM_CHECK(load_ok);

      IM_CHECK_EQ(static_cast<int>(gui::g_state.layers[0].entries.size()), static_cast<int>(shapes.size()));
      for (size_t i = 0; i < shapes.size(); ++i) {
        const auto& e = gui::g_state.layers[0].entries[i];
        IM_CHECK(e.filter_id.has_value());
        const auto& loaded = gui::g_state.filters[*e.filter_id];
        // AC1: field-equal at the operator== (text) layer.
        IM_CHECK(loaded == originals[i]);
      }
      std::remove(tmp_path.c_str());
    };
  }

  // T13 — AC7 serialization equivalence: Remove EE Filter via GUI → OK →
  // entry.filter_id = nullopt → the export JSON emits no "filter" field.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "entry_exit_remove_ok_no_filter_in_json");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      // Pre-populate entry.filter with an EE filter.
      {
        gui::FilterConfig fc;
        fc.action = 0;
        fc.sym_p = true;
        fc.sym_b = true;
        fc.sym_d = true;
        gui::EntryExitParams ee;
        ee.entry_text = "3";
        ee.exit_text = "6";
        fc.SetEntryExit(ee);
        gui::SetFilter(gui::g_state, gui::g_state.layers[0].entries[0], fc);
      }
      ctx->Yield(2);
      IM_CHECK(gui::g_state.layers[0].entries[0].filter_id.has_value());

      ctx->ItemClick("**/Edit##fi");
      ctx->Yield(4);
      // H5 (333.4): the FilterEditType radio was retired; Remove Filter is a
      // single button (##filter). The pre-populated EE filter shows up as one
      // row already; Remove Filter arms the intent flag, OK writes nullopt.
      ctx->ItemClick("**/Remove Filter##filter");
      ctx->Yield(2);
      ctx->ItemClick("**/" ICON_FA_CHECK " OK##edit_modal");
      ctx->Yield(2);

      // filter must be nullopt after Remove + OK.
      IM_CHECK(!gui::g_state.layers[0].entries[0].filter_id.has_value());

      // The export JSON must not emit any "filter" field (no EE residue).
      const std::string core_json = CoreJson(gui::g_state);
      IM_CHECK(!core_json.empty());
      const auto j = nlohmann::json::parse(core_json);
      // No top-level "filter" array, or it's empty.
      const bool no_filter = !j.contains("filter") || j["filter"].empty();
      IM_CHECK(no_filter);
    };
  }

  // task-346.1 (①) AC5 mechanism-layer regression: pin the invariant that the GUI Run
  // path never bakes `exposure_offset` into the committed renderer's intensity_factor.
  // Root cause of the composite re-run 2× EV bug (see plan §3.1): the old code wrote
  // `intensity_factor = 2^exposure_offset`, which then multiplied `display_exposure_scale`
  // (already carrying the same manual EV pushed via LUMICE_SetCompositeExposure) inside
  // the compositor's single shared exposure scalar. This regression fires the second any
  // future refactor reintroduces that bake — without needing to run a real re-run scenario.
  //
  // 399.5: the two semantics are now the two arms of gui::SceneIntent, so this test asserts
  // BOTH of them in one place — kSimCommit must NOT bake, kJsonExport MUST. That is exactly
  // the divergence SceneIntent exists to carry, and it is the only field allowed to differ
  // between the two arms.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "import_export", "intensity_factor_ignores_exposure_offset_in_gui_run_path");
    t->TestFunc = [](ImGuiTestContext*) {
      const float kOffsets[] = { 0.0f, 2.5f, -3.0f, 6.0f };
      for (float offset : kOffsets) {
        ResetTestState();
        gui::g_state.renderer.exposure_offset = offset;

        const auto commit_j = CommitSceneJson(gui::g_state);
        IM_CHECK_EQ(commit_j["render"].size(), (size_t)1);
        // The GUI Run path must NEVER bake exposure_offset here. Manual + auto EV both
        // live on the display-time path (mono shader uniform / LUMICE_SetCompositeExposure).
        IM_CHECK_EQ(commit_j["render"][0]["intensity_factor"].get<float>(), 1.0f);

        // The CLI/config export path MUST bake it — the CLI has no display-time EV, so the
        // exported config has to reproduce the on-screen brightness on its own.
        const auto export_j = nlohmann::json::parse(CoreJson(gui::g_state));
        IM_CHECK_EQ(export_j["render"][0]["intensity_factor"].get<float>(), std::pow(2.0f, offset));

        // Nothing else may depend on the intent: the two documents differ in that one field
        // only. (Pins the SceneIntent divergence from silently spreading to other fields.)
        auto commit_stripped = commit_j;
        auto export_stripped = export_j;
        commit_stripped["render"][0].erase("intensity_factor");
        export_stripped["render"][0].erase("intensity_factor");
        IM_CHECK(commit_stripped == export_stripped);
      }
    };
  }

  // task-fix-stale-render-on-open (scrum-349.1) — 4-path preview-reset audit for DoOpen/DoNew.
  //
  // AC1 root cause: DoOpen's `.lmc`-else branch (no baked preview image) previously only set
  // run_intent=kNone but forgot to ClearTexture(), leaving the previous scene's rendered image
  // stuck on screen. Fix: mirror DoNew() / JSON-import — both call ClearTexture() unconditionally.
  //
  // These tests directly drive the real production DoOpen(path) overload (the same code path
  // that the interactive DoOpen() shell invokes via ShowOpenDialog + forward), so a regression
  // in the else-branch clear will surface here mechanically. The DoNew() path is implicitly
  // guarded by test_gui_interaction.cpp:59 (`gui::g_preview.HasTexture() == false` after DoNew).
  //
  // Threading note: TestFunc runs on the ImGui-test-engine scheduler thread with no active
  // OpenGL context bind, so any GL call from here (glBindTexture / glTexImage2D) crashes with
  // SIGILL. Setup uses `UpdateCpuTextureData` (pure CPU-side — tex_width_/tex_height_/tex_data_)
  // to simulate stale texture state without touching GL; ClearTexture (the code under test) is
  // itself pure CPU-side, so the fix-vs-bug distinction is fully observable via HasTexture().
  // Test 2 (baked-img branch) uses LoadLmcFile directly to observe the file-side payload, since
  // driving DoOpen through UploadTexture would require GL and can't run in TestFunc.
  {
    // AC1 core: Open a .lmc with NO baked preview clears the stale texture from a prior scene.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "open_lmc_no_preview_clears_stale_texture");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // 1) Save a .lmc without a baked preview image (save_texture=false).
      const std::string tmp_path = GuiTestTempPath("lumice_open_no_preview.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, false);
      IM_CHECK(save_ok);

      // 2) Simulate "previous scene rendered": populate g_preview's CPU-side texture state.
      //    UpdateCpuTextureData flips HasTexture() to true without GL calls (see threading note).
      const int stale_w = 8;
      const int stale_h = 4;
      std::vector<unsigned char> stale(stale_w * stale_h * 3, 0xAB);
      gui::g_preview.UpdateCpuTextureData(stale.data(), stale_w, stale_h);
      IM_CHECK(gui::g_preview.HasTexture());  // precondition: stale state truly present

      // 3) DoOpen(path) hits the `.lmc`-else branch (no baked img). Expected: ClearTexture().
      gui::DoOpen(tmp_path);
      // task-349.4: DoOpen's ClearTexture() runs synchronously on the TestFunc coroutine
      // worker thread — the CPU-side g_preview state IS empty by the time this line returns.
      // The historical Yield(3) was masking a different bug: SyncFromPoller() on the main
      // render thread re-uploaded a prior test's leaked PreviewSnapshot payload (see
      // ResetTestState() in test_gui_main.cpp for the fixture-level fix). Bounded
      // wait-until acts as defense-in-depth: if the fixture invalidation ever regresses
      // or a future test leaks state through a new path, this loop times out at
      // kDoOpenSettleYieldLimit and IM_CHECK still fails deterministically — no bug is hidden.
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return !gui::g_preview.HasTexture(); });

      // 4) AC1 assertion: pre-fix this would FAIL (stale dims retained).
      IM_CHECK(!gui::g_preview.HasTexture());

      std::remove(tmp_path.c_str());
    };
  }

  {
    // Audit: a .lmc saved with a baked preview replaces the stale texture via a real DoOpen(path)
    // call — the "has baked img" branch's UploadTexture is a GL call, which crashes (SIGILL) if
    // invoked from TestFunc's coroutine worker thread (no GL context bound there; see the
    // threading note above). Calling DoOpen(path) from GuiFunc instead runs it on the main render
    // thread, where the GL context is current — mirrors the trackball GuiFunc-upload pattern
    // (test_gui_interaction.cpp's lens_*_trackball group).
    static bool s_open_with_preview_done = false;
    static std::string s_open_with_preview_path;
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "open_lmc_with_preview_replaces_stale_texture");
    t->GuiFunc = [](ImGuiTestContext*) {
      if (!s_open_with_preview_done && !s_open_with_preview_path.empty()) {
        gui::DoOpen(s_open_with_preview_path);
        s_open_with_preview_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_open_with_preview_done = false;
      s_open_with_preview_path.clear();

      // 1) Simulate "previous scene rendered": populate g_preview's CPU-side texture state
      //    with a *different* size than the one that will be baked into the .lmc below, so a
      //    dimension match after DoOpen proves real replacement, not stale-data coincidence.
      const int stale_w = 8;
      const int stale_h = 4;
      std::vector<unsigned char> stale(stale_w * stale_h * 3, 0xAB);
      gui::g_preview.UpdateCpuTextureData(stale.data(), stale_w, stale_h);
      IM_CHECK(gui::g_preview.HasTexture());  // precondition: stale state truly present

      // 2) Save a .lmc with a baked preview (save_texture=true) holding "target" pixel data.
      const int target_w = 12;
      const int target_h = 6;
      std::vector<unsigned char> target(target_w * target_h * 3, 0x33);
      gui::g_preview.UpdateCpuTextureData(target.data(), target_w, target_h);
      const std::string tmp_path = GuiTestTempPath("lumice_open_with_preview.lmc").string();
      bool save_ok = gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/true);
      IM_CHECK(save_ok);

      // 3) Re-arm stale state (save above consumed g_preview to build the fixture; now put the
      //    "previous scene" texture back so DoOpen has something real to replace).
      gui::g_preview.UpdateCpuTextureData(stale.data(), stale_w, stale_h);
      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), stale_w);

      // 4) Drive gui::DoOpen(path) for real via GuiFunc (see threading note above), then wait
      //    for it to run (main-thread GuiFunc ticks while TestFunc's coroutine yields).
      //    task-349.4: bounded wait-until — main thread must actually tick GuiFunc, which
      //    sets s_open_with_preview_done and (via LoadLmcFile + UploadTexture) resizes
      //    g_preview to target dims. Predicate mirrors the following IM_CHECK exactly so a
      //    real regression (DoOpen wiring breaks) still fails at line :NNNN, just after
      //    kDoOpenSettleYieldLimit frames instead of a fixed 3.
      s_open_with_preview_path = tmp_path;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [target_w, target_h] {
        return gui::g_preview.HasTexture() && gui::g_preview.GetTextureWidth() == target_w &&
               gui::g_preview.GetTextureHeight() == target_h;
      });

      // 5) AC2 assertion: DoOpen's "has baked img" branch replaced the stale texture with the
      //    file's baked one — this is the real production entry point, not a file-side proxy.
      IM_CHECK(gui::g_preview.HasTexture());
      IM_CHECK_EQ(gui::g_preview.GetTextureWidth(), target_w);
      IM_CHECK_EQ(gui::g_preview.GetTextureHeight(), target_h);

      std::remove(tmp_path.c_str());
    };
  }

  {
    // Audit: Open a CLI JSON config (import path) clears the stale texture.
    // Already correct pre-fix — locked in as a regression anchor for the 4-path audit.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "open_json_import_clears_stale_texture");
    t->TestFunc = [](ImGuiTestContext*) {
      ResetTestState();

      // 1) Write a valid CLI-shape JSON to tmp (export round-trip).
      std::string json = CoreJson(gui::g_state);
      const std::string tmp_path = GuiTestTempPath("lumice_open_json_import.json").string();
      bool write_ok = gui::ExportConfigJson(tmp_path, json);
      IM_CHECK(write_ok);

      // 2) Simulate "previous scene rendered" via CPU-side texture state (see threading note).
      const int stale_w = 8;
      const int stale_h = 4;
      std::vector<unsigned char> stale(stale_w * stale_h * 3, 0x77);
      gui::g_preview.UpdateCpuTextureData(stale.data(), stale_w, stale_h);
      IM_CHECK(gui::g_preview.HasTexture());

      // 3) DoOpen(json_path) hits the JSON-import branch → ClearTexture().
      gui::DoOpen(tmp_path);

      // 4) Assertion.
      IM_CHECK(!gui::g_preview.HasTexture());

      std::remove(tmp_path.c_str());
    };
  }

  // ========== task-350: GL pixel-level regression tests for stale-render on Open ==========
  //
  // task-349.1 (which added the three open_*_clears_stale_texture tests above) validated the
  // CPU-side HasTexture() flag but never sampled the GL texture itself — the underlying bug
  // (ClearTexture() left the GL texture_ handle holding the previous scene's pixels and
  // Render() kept sampling them) survived that green test suite until owner on-screen
  // regression. This block re-verifies the same code path at the true output layer via
  // RenderExportToRgba, which internally invokes PreviewRenderer::Render() and reads back
  // the FBO — a real production-shaped Render() call, not a CPU-side proxy.
  //
  // Threading: PreviewRenderer::UploadTexture / RenderExportToRgba both require a GL
  // context and must run on the main render thread (GuiFunc). ClearTexture() itself is
  // safe from the coroutine worker (task-349.4 fixture invariant) — the deferred blank
  // is consumed by the next Render() on the main thread. See preview_renderer.hpp
  // needs_gl_blank_ contract.
  //
  // Test scaffolding (GlOpTestState / GlOpGuiFunc / kStale* / FillSolid) lives in the
  // anonymous namespace at the top of this file so the GuiFunc can be a plain non-capturing
  // function pointer (ImGuiTestGuiFunc rejects capturing lambdas).

  {
    // AC1 (GL layer): ClearTexture() followed by Render() must leave the sim layer black,
    // not sampling the previous scene's pixels. Real production Render() is invoked via
    // RenderExportToRgba on the main thread.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "gl_render_clears_stale_sim_pixels");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Frame A: upload distinctive stale pixels + render once — establishes the "stale
      // state truly present in GL storage" precondition (analogue of the CPU-side
      // HasTexture() precondition in the sibling open_*_clears_stale_texture tests).
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);  // stale magenta really sampled from GL
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      // Frame B: ClearTexture() then render — AC1 core assertion. Pre-fix (ClearTexture
      // only touching CPU state) this frame would still sample the stale magenta.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_clear = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
    };
  }

  {
    // AC2 (bg not connected out): after ClearTexture(), if a background image is loaded and
    // bg.enabled=true, the background must still show through — the fix must not hide bg
    // as collateral damage. Uses bg.alpha=0 so the shader outputs pure bg (sim contribution
    // zeroed out), making the assertion a direct color check on bg content.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "gl_render_preserves_bg_after_clear");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      g_gl_op.do_upload_bg = true;
      g_gl_op.do_clear = true;
      g_gl_op.bg_enabled = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      // Expect pure bg (green). Not stale magenta (would mean bg alpha inverted or fix
      // regressed to draw stale sim on top) and not all black (would mean bg was
      // connected-out along with the sim layer).
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
    };
  }

  {
    // Interleaved sequence: ClearTexture() then UploadTexture(fresh) with NO intervening
    // Render() — the deferred-blank flag must be dropped by UploadTexture so the very next
    // Render() shows the freshly uploaded pixels, not black. Directly locks in the "newest
    // real write wins" invariant added in task-350 (plan-review round-1 Major fix).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "gl_upload_after_clear_wins_over_pending_blank");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;              // seed with something distinctive
      g_gl_op.do_clear = true;                     // set pending blank
      g_gl_op.do_upload_fresh_after_clear = true;  // must drop the pending blank
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      // Expect fresh cyan, not black — if UploadTexture failed to clear needs_gl_blank_,
      // Render() would have overwritten the just-uploaded cyan with the 1x1 kBlack helper.
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);
    };
  }

  // ========== task-command-reset-owner: end-to-end + AC4 single-owner gate ==========
  //
  // These tests are the last line of defense for the ResetFrontendState consolidation
  // (backlog #5 / doc §4 "文档重置 owner"):
  //   - `donew_end_to_end_blanks_gl_via_owner` drives the REAL `DoNew()` (which delegates
  //     to ResetFrontendState internally) after a stale GL upload, then samples the next
  //     Render() at the GL pixel layer to confirm the owner's deferred-blank propagated
  //     through the full DoNew → owner → next-frame Render chain. This complements the
  //     existing gl_render_clears_stale_sim_pixels test (which drives ClearTexture
  //     directly) by covering the outer command handler.
  //   - `reset_primitives_are_owner_single_source` reads src/gui/app.cpp and asserts the
  //     ResetFrontendState-owned primitives (`InvalidateStagedTexture`, mesh-hash zero)
  //     have exactly one production call site — the owner. A future PR that re-scatters
  //     the primitives back into command handlers (the exact bug class that produced the
  //     task-349.1 → 350 → 351 fix cycle) fails here mechanically instead of surviving
  //     until owner on-screen regression.

  {
    // AC1 end-to-end: `DoNew()` must leave the sim layer black at the GL pixel level (not just
    // CPU-side HasTexture()==false), driven through the REAL command handler that delegates to
    // ResetFrontendState. Precondition: stale GL pixels really present. Postcondition: next
    // Render() samples black — proves ClearTexture()'s deferred-blank propagated through the
    // owner's kNewDocument branch and got consumed by the next main-thread Render.
    //
    // Threading: DoNew()'s reset primitives are all CPU-safe (no GL calls) from the coroutine
    // worker — ClearTexture is a flag flip, ClearBackground is a flag flip, InvalidateStagedTexture
    // is a poller mutex op, ResetCrystalViewToCrystal is CPU-only. The GL blank consumption happens
    // on the next Render() via GlOpGuiFunc (main thread). See preview_renderer.hpp needs_gl_blank_.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "donew_end_to_end_blanks_gl_via_owner");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Frame A: seed distinctive stale pixels via GuiFunc + render — establishes the "stale
      // pixels really present in GL storage" precondition (mirrors gl_render_clears_stale_sim_pixels).
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      // Real production DoNew() — coroutine-safe (see threading note above). Sets the pending-blank
      // via the owner's kNewDocument branch (ClearTexture flag flip).
      gui::DoNew();

      // Frame B: pure Render() — no direct GL ops requested, just consume the deferred blank
      // that DoNew's ResetFrontendState(kNewDocument) staged.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      // Expected: pure black. A regression (owner drops the ClearTexture call, or command handler
      // re-adds a texture upload after the owner call) samples the stale magenta.
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);
    };
  }

  {
    // AC1 end-to-end: `DoOpen(.json)` runs the owner's kOpenJson branch — ClearTexture +
    // ClearBackground + InvalidateStagedTexture + trackball reset. GL-level assertion: after a
    // stale sim upload, importing a CLI JSON config leaves the sim layer black on the next
    // Render(). Complements the CPU-side `open_json_import_clears_stale_texture` (HasTexture()
    // proxy) by covering the actual pixel output through the full command → owner → main-thread
    // Render() chain, which is the task-351 regression surface.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "doopen_json_end_to_end_blanks_gl_via_owner");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Prepare a valid CLI-shape JSON config on disk (round-trips through the export path).
      std::string json = CoreJson(gui::g_state);
      const std::string tmp_path = GuiTestTempPath("lumice_reset_owner_json_import.json").string();
      IM_CHECK(gui::ExportConfigJson(tmp_path, json));

      // Frame A: seed stale GL pixels — the "previous scene still showing" precondition.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      // Real production DoOpen(.json) — CPU-safe from the coroutine worker (owner's kOpenJson
      // branch only flips flags; the GL blank is consumed by the next Render()).
      gui::DoOpen(tmp_path);

      // Frame B: pure Render() — must sample black (owner's deferred blank propagated through).
      g_gl_op.Reset();
      g_gl_op.requested = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);

      std::remove(tmp_path.c_str());
    };
  }

  {
    // AC1 end-to-end: `DoOpen(.lmc no-baked)` runs the owner's kOpenLmcBlank branch — same
    // ClearTexture+ClearBackground+InvalidateStagedTexture+trackball subset as kOpenJson, but
    // sourced from an .lmc file with no baked preview payload (save_texture=false path).
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "doopen_lmc_blank_end_to_end_blanks_gl_via_owner");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Write an .lmc file WITHOUT a baked preview — DoOpen hits the kOpenLmcBlank branch.
      const std::string tmp_path = GuiTestTempPath("lumice_reset_owner_lmc_blank.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/false));

      // Frame A: seed stale GL pixels.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);

      // DoOpen(.lmc no-baked) — CPU-safe (owner's kOpenLmcBlank branch is CPU-only flag flips).
      gui::DoOpen(tmp_path);

      // Frame B: pure Render() — must sample black.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0x00);

      std::remove(tmp_path.c_str());
    };
  }

  {
    // AC1 end-to-end: `DoOpen(.lmc baked)` runs the owner's kOpenBaked branch — UploadTexture
    // with the file's payload + ClearBackground + InvalidateStagedTexture + trackball reset.
    // Postcondition is DIFFERENT from the other document-switch branches: preview must show the
    // BAKED bytes (not black), proving the owner uploads the payload rather than falling back
    // to a blanket ClearTexture. GL-level assertion catches a regression where the owner drops
    // the UploadTexture call (would sample black) or forgets to override the pending blank
    // (would sample black after the deferred-clear consumes it).
    //
    // Threading: UploadTexture is a GL call — must run on the main thread (GuiFunc). Mirrors
    // the existing `open_lmc_with_preview_replaces_stale_texture` staging pattern: TestFunc
    // arms an s_open_path, GuiFunc invokes DoOpen once when it becomes non-empty.
    static bool s_open_baked_done = false;
    static std::string s_open_baked_path;
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "doopen_lmc_baked_end_to_end_uploads_gl_via_owner");
    t->GuiFunc = [](ImGuiTestContext* ctx) {
      // First tick: run the GlOp seed/probe if requested (same body as GlOpGuiFunc).
      GlOpGuiFunc(ctx);
      // Second phase: DoOpen(baked) invocation — GL-context-bound so runs from GuiFunc.
      if (!s_open_baked_done && !s_open_baked_path.empty()) {
        gui::DoOpen(s_open_baked_path);
        s_open_baked_done = true;
      }
    };
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      s_open_baked_done = false;
      s_open_baked_path.clear();

      // Build an .lmc that holds a distinctive baked payload (kFreshCyan = 0x00,0xFF,0xFF).
      // Injecting the payload through g_preview.UpdateCpuTextureData is a CPU-side seam that
      // SaveLmcFile serializes, so the resulting file's baked bytes match kFreshCyan exactly.
      const int baked_w = 12;
      const int baked_h = 6;
      std::vector<unsigned char> baked(static_cast<size_t>(baked_w) * baked_h * 3);
      for (int i = 0; i < baked_w * baked_h; ++i) {
        baked[i * 3 + 0] = kFreshCyan[0];
        baked[i * 3 + 1] = kFreshCyan[1];
        baked[i * 3 + 2] = kFreshCyan[2];
      }
      gui::g_preview.UpdateCpuTextureData(baked.data(), baked_w, baked_h);
      const std::string tmp_path = GuiTestTempPath("lumice_reset_owner_lmc_baked.lmc").string();
      IM_CHECK(gui::SaveLmcFile(tmp_path, gui::g_state, gui::g_preview, /*save_texture=*/true));

      // Frame A: seed stale magenta pixels through the real GL path.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);

      // Drive DoOpen(.lmc baked) via GuiFunc — see threading note above.
      s_open_baked_path = tmp_path;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return s_open_baked_done; });
      IM_CHECK(s_open_baked_done);

      // Frame B: pure Render() — must sample the FRESH baked pixels (cyan), not stale magenta
      // and not black. Regression modes: owner drops UploadTexture (→ black or magenta), owner
      // fails to override the pending-blank flag (→ black), or handler bypasses owner and calls
      // ClearTexture directly (→ black).
      g_gl_op.Reset();
      g_gl_op.requested = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);

      std::remove(tmp_path.c_str());
    };
  }

  {
    // AC1 end-to-end (kRevert branch): `DoRevert()` must NOT clear the preview — it is a config
    // restore, not a document switch. GL-level assertion: after uploading distinctive pixels,
    // Revert leaves those pixels intact on the next Render(). Regression mode: a well-meaning
    // future change generalises the owner's kRevert branch to also ClearTexture (matching the
    // other reasons); this test catches it because the "stale" pixels are the current preview
    // that Revert must preserve, not stale content to be cleared.
    ImGuiTest* t = IM_REGISTER_TEST(engine, "import_export", "dorevert_preserves_gl_preview_via_owner");
    t->GuiFunc = GlOpGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();

      // Arm a revert snapshot so DoRevert actually runs its body (guarded by
      // `if (g_state.last_committed_state)` — see app.cpp DoRevert). Any snapshot works: the
      // preview retention is independent of what the config restore does.
      gui::g_state.last_committed_state = gui::GuiState::ConfigSnapshot::From(gui::g_state);

      // Frame A: upload distinctive pixels — this is now the "current preview", not stale.
      g_gl_op.Reset();
      g_gl_op.requested = true;
      g_gl_op.do_upload_stale = true;  // reuses the magenta seed; label irrelevant to the assertion
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);

      // Real production DoRevert() — CPU-safe (ApplyTo is pure assignment; the owner's kRevert
      // branch touches no GL).
      gui::DoRevert();

      // Frame B: pure Render() — must STILL sample the current preview (magenta). Not black
      // (which would mean Revert wrongly ClearTexture'd or InvalidateStagedTexture'd through
      // the wrong branch).
      g_gl_op.Reset();
      g_gl_op.requested = true;
      YieldUntilTrue(ctx, kDoOpenSettleYieldLimit, [] { return g_gl_op.done; });
      IM_CHECK(g_gl_op.export_ok);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_r), 0xFF);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_g), 0x00);
      IM_CHECK_EQ(static_cast<int>(g_gl_op.center_b), 0xFF);
    };
  }
}
