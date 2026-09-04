// The sky reference points, as a user reaches them and as they land on a frame.
//
// What this suite is for. Three propositions that the Overlay group's own suite
// (functional/test_overlay_controls.cpp) cannot state, because each of them is about something
// other than the table's layout:
//
//   * REACHABILITY, counted in interactions. The six markers live behind a collapsed section, and
//     the reason that is acceptable is a number: turning one on costs two interactions from a fresh
//     document and one from a document that already has the section open. Counted by driving the
//     real controls, not by reading the code — a count derived from the source would be a
//     restatement of the source. This is the measure the previous panel reorganisation was faulted
//     for never taking (doc/gui-layout-architecture.md §8: the two acceptance gates both asked
//     whether the shape was good and never asked how many steps a common task took), so it is taken
//     here rather than argued about.
//
//   * DISTINGUISHABILITY, in rendered pixels. Six points are only useful if a viewer can tell which
//     ring is which, and the only thing that tells them apart is colour — radius and opacity are
//     family-wide by design (LUMICE_MarkerStyle says why). So the claim is not "six rings were
//     drawn" but "six rings were drawn in six different colours, each the one its row shows", and
//     it is read off the frame.
//
//   * THE THIRD STATE of a row. A marker can be switched on and still not be on the canvas — the
//     lens does not image that direction, or `visible` crops it away — and that state has to look
//     different from "switched off", or a user who ticks the subsun under visible=upper sees a
//     ticked box, no ring, and nothing saying which of the two happened.
//
// Capture path for the pixel cases: RenderExportToRgba's own off-screen FBO, the same one
// functional/test_lens_border.cpp probes through, so nothing here depends on window size or panel
// layout. No simulation is run — the source texture stays empty, which makes the frame black
// everywhere a ring is not, and a coloured pixel therefore unambiguous.
//
// The panel case captures the DEFAULT framebuffer instead, through g_fullframe_capture's sub-region
// protocol: what it is about is how a row LOOKS, and the row is only ever drawn on screen.
//
// What a user sees when these break: a reference point that takes four clicks to switch on, six
// rings in one colour with no way to tell the anthelion from the subsun, or a ticked box with no
// ring and no explanation.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "gui/app.hpp"
#include "gui/export_fbo_renderer.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_state.hpp"
#include "test_gui_shared.hpp"

namespace {

// RAII pairing for ImGuiTestContext::SetRef, for the same mechanical reason ScopedPopups exists
// (see its comment in test_gui_shared.hpp): a fatal IM_CHECK expands to `return` in the enclosing
// lambda, so a tail-of-function SetRef("") only runs when the case passes — and the assertions it
// sits behind are exactly the ones a real regression would trip. A second copy of the one in
// test_overlay_controls.cpp, which is file-local there; promoting it to the shared header is a
// change to a file every gui_test case includes and is not this task's to make.
struct ScopedRefForMarkers {
  ScopedRefForMarkers(ImGuiTestContext* ctx, const char* ref) : ctx_(ctx) { ctx_->SetRef(ref); }
  ~ScopedRefForMarkers() { ctx_->SetRef(""); }

  ScopedRefForMarkers(const ScopedRefForMarkers&) = delete;
  ScopedRefForMarkers& operator=(const ScopedRefForMarkers&) = delete;

 private:
  ImGuiTestContext* ctx_;
};

constexpr int kProbeW = 512;
constexpr int kProbeH = 512;

// A render request marshalled to the frame loop. RenderExportToRgba must run on the thread that
// owns the GL context, which is the render thread, not the test coroutine — the same crossing
// test_lens_border.cpp and test_file_ops.cpp make for their own pixel probes.
struct MarkerRenderRequest {
  bool requested = false;
  bool done = false;
  gui::PreviewParams params;
  std::vector<gui::CurveLabelSet> labels;
  std::vector<unsigned char> rgba;

  void Reset() { *this = MarkerRenderRequest{}; }
};

MarkerRenderRequest g_req;

// A GL upload the panel case needs, marshalled the same way for the same reason: a TestFunc runs
// on the test engine's coroutine and a GL call from there is a call on the wrong thread.
bool g_upload_requested = false;
bool g_upload_done = false;

void RunMarkerRenderRequest() {
  // Blank the source texture: whatever ran before this case left one behind, and a coloured sample
  // showing through would make "a coloured pixel here is a marker" false. Deferred to the next
  // Render() by design (preview_renderer.hpp), which is the RenderExportToRgba below.
  gui::g_preview.ClearTexture();
  g_req.rgba = gui::RenderExportToRgba(gui::g_preview, g_req.params, kProbeW, kProbeH, g_req.labels);
  g_req.done = true;
  g_req.requested = false;
}

void MarkerPanelGuiFunc(ImGuiTestContext* /*ctx*/) {
  if (g_upload_requested && !g_upload_done) {
    InitSynthTexture();
    gui::g_preview.UploadTexture(g_synth_tex.data(), kSynthTexW, kSynthTexH);
    g_upload_done = true;
  }
  if (g_req.requested && !g_req.done) {
    RunMarkerRenderRequest();
  }
}

// Put a texture in the renderer and wait for it. The panel case needs one because
// RenderPreviewPanel gates its whole overlay block — the annotation request included — on
// `g_preview.HasTexture() || g_preview.HasBackground()`. That gate is right for the product: with
// no image on screen there is nothing for a view to crop a marker out of, so a row has no third
// state to be in. It does mean the case has to establish the precondition rather than assume it.
void EnsurePreviewTexture(ImGuiTestContext* ctx) {
  g_upload_requested = true;
  g_upload_done = false;
  for (int i = 0; i < 60 && !g_upload_done; ++i) {
    ctx->Yield();
  }
}

// The full-sky view every pixel case here uses. Dual fisheye at 180 with no hemisphere crop is the
// one family that images ALL SIX directions at once — zenith and nadir are opposite, so no single
// lens can show both, and a scene that could not show both would leave half the id space untested.
void InstallFullSkyView(gui::GuiState& s) {
  s.renderer.lens_type = gui::kLensTypeDualFisheyeEqualArea;
  s.renderer.fov = 180.0f;
  s.renderer.visible = gui::kVisibleFull;
  s.renderer.front = false;
  s.renderer.elevation = 0.0f;
  s.renderer.azimuth = 0.0f;
  s.renderer.roll = 0.0f;
  // Non-zero so the four sun-relative directions are not degenerate with the two absolute ones:
  // at altitude 0 the sun sits on the horizon and the anthelion opposite it, which is a legal but
  // needlessly special arrangement for a colour-separation test.
  s.sun.altitude = 25.0f;
}

// Fill the overlay half of a PreviewParams from GuiState + a settled cache, the same way
// app_panels.cpp does per frame. Deliberately NOT a hand-written mirror of the marker positions:
// the point of the pixel cases is to check the SHADER against core's answer, and a fixture that
// projected the six directions itself would be checking the fixture.
void FillMarkerOverlay(gui::PreviewParams& params, const gui::AnnotationOverlayCache& cache, const gui::GuiState& s) {
  params.overlay.markers_alpha = 1.0f;  // fully opaque: the pixel test reads a colour, not a blend
  params.overlay.markers_radius_px = s.markers_radius_px;
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    std::copy(std::begin(s.markers[i].color), std::end(s.markers[i].color), params.overlay.marker_color[i].begin());
    const gui::AnnotationOverlayCache::Point p =
        s.markers[i].show ? cache.MarkerPoint(i) : gui::AnnotationOverlayCache::Point{};
    gui::CanvasPointToShaderScreenPos(p, s.renderer.lens_type, kProbeW, kProbeH,
                                      params.overlay.marker_screen_pos[i].data());
  }
}

gui::PreviewParams BaseParams() {
  gui::PreviewParams params{};
  const auto& rc = gui::g_state.renderer;
  params.view_proj = gui::BuildPreviewViewProjFromRenderer(rc);
  params.source.max_abs_dz = gui::kDualFisheyeOverlap;
  params.source.r_scale = 1.0f / std::sqrt(1.0f + gui::kDualFisheyeOverlap);
  params.exposure.intensity_factor = 1.0f;
  params.exposure.intensity_scale = 0.0f;  // 8-bit RGB mode; no simulation has been run
  return params;
}

// Ask the render thread for one frame and wait for it. Bounded so a wiring regression fails at the
// assertion below rather than hanging the suite.
std::vector<unsigned char> RenderFrame(ImGuiTestContext* ctx, const gui::PreviewParams& params,
                                       const std::vector<gui::CurveLabelSet>& labels) {
  g_req.Reset();
  g_req.params = params;
  g_req.labels = labels;
  g_req.requested = true;
  for (int i = 0; i < 60 && !g_req.done; ++i) {
    ctx->Yield();
  }
  return g_req.rgba;
}

// Pixel at a marker_screen_pos coordinate: centre-origin, and y-DOWN because every case in this
// file renders through a full-sky lens.
//
// The y direction is a property of the LENS FAMILY, not of the buffer. The shader hands
// overlayAuxLines `pos_ovl`, which for kFullSkyLensTypes is `vec2(pos.x, -pos.y)` — so on those
// five lenses a marker uniform's y counts DOWNWARD from the centre, and a fragment at image row R
// has pos_ovl.y = R - h/2. Hence `row = h/2 + pos_y` rather than the `h/2 - pos_y` a y-up space
// would want. functional/test_lens_border.cpp's twin of this helper uses the y-up form and is right
// to: what it probes is a circle centred on the origin, where the two forms agree.
//
// Asserted rather than parameterised, because a case here that installed a view-matrix lens would
// need every OTHER coordinate in this file reconsidered too, not just this line.
bool ReadPixel(const std::vector<unsigned char>& rgba, float pos_x, float pos_y, unsigned char* out_rgb) {
  const int col = static_cast<int>(std::lround(pos_x + kProbeW * 0.5f));
  const int row = static_cast<int>(std::lround(kProbeH * 0.5f + pos_y));
  if (col < 0 || col >= kProbeW || row < 0 || row >= kProbeH) {
    return false;
  }
  const std::size_t off = (static_cast<std::size_t>(row) * kProbeW + col) * 4;
  out_rgb[0] = rgba[off + 0];
  out_rgb[1] = rgba[off + 1];
  out_rgb[2] = rgba[off + 2];
  return true;
}

// The brightest pixel on a marker's ring: walk the circumference and, at each angle, the ±2 px the
// anti-aliased edge and the row/col rounding can put it in. The BRIGHTEST rather than the one at
// the exact radius, because the smoothstep edge means the exact radius is not necessarily the
// fully-saturated sample — and the claim being made is about the colour a viewer sees, which is the
// ring's own colour at its brightest, not a blend against the background.
bool BrightestRingSample(const std::vector<unsigned char>& rgba, float cx, float cy, float radius,
                         unsigned char* out_rgb) {
  int best_sum = -1;
  unsigned char rgb[3];
  for (int a = 0; a < 64; ++a) {
    const float phi = static_cast<float>(a) * (2.0f * 3.14159265358979323846f / 64.0f);
    for (int d = -2; d <= 2; ++d) {
      const float r = radius + static_cast<float>(d);
      if (!ReadPixel(rgba, cx + std::cos(phi) * r, cy + std::sin(phi) * r, rgb)) {
        continue;
      }
      const int sum = rgb[0] + rgb[1] + rgb[2];
      if (sum > best_sum) {
        best_sum = sum;
        out_rgb[0] = rgb[0];
        out_rgb[1] = rgb[1];
        out_rgb[2] = rgb[2];
      }
    }
  }
  return best_sum > 0;
}

// Non-black pixels in a square around a point. What the label cases read: a name drawn beside a
// ring puts glyphs where there were none, and the ring itself contributes the same count to both
// frames, so the DIFFERENCE between the two frames is the text.
int NonBlackCountAround(const std::vector<unsigned char>& rgba, float cx, float cy, int half) {
  int count = 0;
  unsigned char rgb[3];
  for (int dy = -half; dy <= half; ++dy) {
    for (int dx = -half; dx <= half; ++dx) {
      if (!ReadPixel(rgba, cx + static_cast<float>(dx), cy + static_cast<float>(dy), rgb)) {
        continue;
      }
      if (rgb[0] + rgb[1] + rgb[2] > 24) {
        count++;
      }
    }
  }
  return count;
}

// 8-bit distance between two colours, as the maximum per-channel difference. Max rather than a sum
// so a single channel landing far off cannot be averaged away by two that agree.
int ColorDistance(const unsigned char* a, const unsigned char* b) {
  int d = 0;
  for (int c = 0; c < 3; ++c) {
    d = std::max(d, std::abs(static_cast<int>(a[c]) - static_cast<int>(b[c])));
  }
  return d;
}

void ExpectedRgb8(const float color[3], unsigned char* out) {
  for (int c = 0; c < 3; ++c) {
    out[c] = static_cast<unsigned char>(std::lround(std::clamp(color[c], 0.0f, 1.0f) * 255.0f));
  }
}

// Reconstruct the id of a marker row's colour swatch. A "**/" search cannot reach it however it is
// spelled: ColorButton never calls IMGUI_TEST_ENGINE_ITEM_INFO, so it registers no debug label, and
// a wildcard matches by label. Three seeds, outermost first: the window the row was submitted into,
// the table's override id (every cell widget hashes against it rather than the window's), and
// ColorEdit3's PushID of its own label. Same reconstruction test_overlay_controls.cpp makes for the
// line rows' swatches, against that table's id.
ImGuiID MarkerSwatchId(const ImGuiTestItemInfo& row_item, const char* serial_name) {
  if (row_item.Window == nullptr) {
    return 0;
  }
  const ImGuiID table_id = ImGui::GetIDWithSeed("##MarkersTable", nullptr, row_item.Window->ID);
  const ImGuiID swatch_group =
      ImGui::GetIDWithSeed((std::string("##marker_color_") + serial_name).c_str(), nullptr, table_id);
  return ImGui::GetIDWithSeed("##ColorButton", nullptr, swatch_group);
}

}  // namespace

void RegisterMarkerPanelTests(ImGuiTestEngine* engine) {
  // ===============================================================================================
  // Reachability, counted.
  // ===============================================================================================
  //
  // The count is of INTERACTIONS a user performs — an unfold and a tick — and it is taken by
  // performing them. `interactions` is incremented beside each ctx-> call that a user would have to
  // make with their own hand; a helper that batched them would make the number a property of the
  // helper.
  //
  // Two documents, because the budget is claimed for both: a fresh one (section closed, the state
  // MakeNewDocumentState opens in) and one that already carries the section open. The second is not
  // a hypothetical — the open state is serialized (overlay_markers_section_open), so a user who
  // works with reference points keeps the section open and pays one interaction from then on.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "marker_panel", "switching_a_reference_point_on_costs_at_most_three_acts");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(3);
      const int antisolar = LUMICE_ANNOTATION_MARKER_ANTISOLAR;

      // The premise the count rests on. Stated rather than assumed: if a future default opened the
      // section, or pre-ticked a marker, the arithmetic below would be measuring a different task.
      IM_CHECK(!gui::g_state.markers_section_open);
      IM_CHECK(!gui::g_state.markers[antisolar].show);

      {
        // Named ref, not the default: the Overlay group is the last one in a scrollable panel, and
        // a wildcard lookup resolves an item by its LABEL — the engine records no label for a
        // clipped item, so a control scrolled out of view reads exactly like a missing one.
        const ScopedRefForMarkers panel_ref(ctx, "//##RightPanel");

        int interactions = 0;
        ctx->ItemClick("**/Reference Points##markers");
        interactions++;
        ctx->Yield(2);
        IM_CHECK(gui::g_state.markers_section_open);

        ctx->ItemClick("**/##marker_line_antisolar");
        interactions++;
        ctx->Yield(2);
        IM_CHECK(gui::g_state.markers[antisolar].show);

        // Three is the budget; two is what it costs. Both are stated: the first is the promise, the
        // second is what a regression would move first.
        IM_CHECK_LE(interactions, 3);
        IM_CHECK_EQ(interactions, 2);
      }

      // The second document. Built by SERIALIZING the open section and reading it back, not by
      // assigning the field — what is claimed is that a saved document carries the state, and an
      // assignment would skip exactly the half that can break.
      gui::g_state.markers_section_open = true;
      gui::g_state.markers[antisolar].show = false;
      const std::string doc = gui::SerializeGuiStateJson(gui::g_state);
      gui::GuiState reopened;
      IM_CHECK(gui::DeserializeGuiStateJson(doc, reopened));
      IM_CHECK(reopened.markers_section_open);
      gui::g_state = reopened;
      ctx->Yield(3);

      {
        const ScopedRefForMarkers panel_ref(ctx, "//##RightPanel");
        int interactions = 0;
        ctx->ItemClick("**/##marker_line_antisolar");
        interactions++;
        ctx->Yield(2);
        IM_CHECK(gui::g_state.markers[antisolar].show);
        IM_CHECK_EQ(interactions, 1);
      }

      // [All] is the third path, and the one that makes the count irrelevant for the "I want them
      // all" case: one interaction, six markers. Asserted here rather than in its own case because
      // it is the same proposition — how many acts does switching a reference point on cost.
      for (gui::MarkerAppearance& m : gui::g_state.markers) {
        m.show = false;
      }
      ctx->Yield(2);
      {
        const ScopedRefForMarkers panel_ref(ctx, "//##RightPanel");
        ctx->ItemClick("**/All##markers_all");
      }
      ctx->Yield(2);
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        if (!gui::g_state.markers[i].show) {
          IM_ERRORF("[All] left marker %s off", gui::kMarkerSerialNames[i]);
          break;
        }
      }
      if (!ctx->IsError()) {
        const ScopedRefForMarkers panel_ref(ctx, "//##RightPanel");
        ctx->ItemClick("**/None##markers_none");
        ctx->Yield(2);
        IM_CHECK(!gui::AnyMarkerShown(gui::g_state));
      }
    };
  }

  // ===============================================================================================
  // Six rings, six colours, one frame.
  // ===============================================================================================
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "marker_panel", "all_six_reference_points_land_in_six_tellable_colours");
    t->GuiFunc = MarkerPanelGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      InstallFullSkyView(gui::g_state);
      for (gui::MarkerAppearance& m : gui::g_state.markers) {
        m.show = true;
        m.label = false;
      }
      ctx->Yield(3);

      // Positions from the production path — AnnotationViewInputFor reads the same GuiState the
      // panel edits, and the cache is the same class the live preview drives.
      gui::AnnotationOverlayCache cache;
      cache.Refresh(gui::MakeAnnotationViewKey(gui::AnnotationViewInputFor(gui::g_state, gui::g_state.renderer),
                                               kProbeW, kProbeH));
      IM_CHECK(cache.HasResult());
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        if (!cache.MarkerPoint(i).valid) {
          IM_ERRORF("the full-sky fixture does not image %s; the scene cannot make its point",
                    gui::kMarkerSerialNames[i]);
          return;
        }
      }

      gui::PreviewParams params = BaseParams();
      FillMarkerOverlay(params, cache, gui::g_state);
      // Where the six landed, on the log. A colour mismatch below is most often a POSITION problem
      // — two rings overlapping, so the later id paints over the earlier — and the sampled colour
      // alone cannot say which of the two it is.
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        fprintf(stderr, "[marker_panel] %-10s screen=(%.1f, %.1f)\n", gui::kMarkerSerialNames[i],
                static_cast<double>(params.overlay.marker_screen_pos[i][0]),
                static_cast<double>(params.overlay.marker_screen_pos[i][1]));
      }
      const std::vector<unsigned char> frame = RenderFrame(ctx, params, {});
      IM_CHECK_EQ(frame.size(), static_cast<std::size_t>(kProbeW) * kProbeH * 4);

      // Each ring is its OWN row's colour. This is what a per-marker colour buys and what a wiring
      // slip would take away: uploading one colour six times, or indexing the upload by request
      // order rather than by marker id, both draw six rings and fail here.
      unsigned char sampled[LUMICE_ANNOTATION_MARKER_COUNT][3];
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        const float* pos = params.overlay.marker_screen_pos[i].data();
        if (!BrightestRingSample(frame, pos[0], pos[1], gui::g_state.markers_radius_px, sampled[i])) {
          IM_ERRORF("marker %s: no lit pixel anywhere on its ring", gui::kMarkerSerialNames[i]);
          break;
        }
        unsigned char expected[3];
        ExpectedRgb8(gui::g_state.markers[i].color, expected);
        // 24/255 absorbs the smoothstep edge and the 8-bit round trip. Calibrated by breaking it on
        // purpose: the closest pair of factory colours (subsun vs nadir) differ by 92 in their
        // farthest channel, so a tolerance this size cannot let one marker's colour pass as
        // another's — which is the confusion the case exists to catch.
        const int d = ColorDistance(sampled[i], expected);
        if (d > 24) {
          IM_ERRORF("marker %s: ring sampled (%d,%d,%d), row says (%d,%d,%d)", gui::kMarkerSerialNames[i],
                    sampled[i][0], sampled[i][1], sampled[i][2], expected[0], expected[1], expected[2]);
          break;
        }
        if (ctx->IsError()) {
          break;
        }
      }
      if (ctx->IsError()) {
        return;
      }

      // ...and the six are tellable APART, which is the user-facing half. Asserted on the SAMPLED
      // colours rather than on the palette, so it is a statement about the frame: a palette whose
      // members are distinct on paper but land on the same bytes after the blend would pass the
      // first half of this case and fail here.
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        for (int j = i + 1; j < LUMICE_ANNOTATION_MARKER_COUNT; ++j) {
          const int d = ColorDistance(sampled[i], sampled[j]);
          if (d <= 24) {
            IM_ERRORF("markers %s and %s are indistinguishable in the frame (max channel delta %d)",
                      gui::kMarkerSerialNames[i], gui::kMarkerSerialNames[j], d);
            break;
          }
        }
        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // ===============================================================================================
  // The names.
  // ===============================================================================================
  //
  // Separate from the colours above because it is a separate switch — a user may want six rings and
  // no text, or the reverse — and because what it reads is a difference between two frames rather
  // than a colour in one.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "marker_panel", "turning_the_names_on_puts_each_name_beside_its_own_ring");
    t->GuiFunc = MarkerPanelGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      InstallFullSkyView(gui::g_state);
      for (gui::MarkerAppearance& m : gui::g_state.markers) {
        m.show = true;
        m.label = false;
      }
      ctx->Yield(3);

      gui::AnnotationOverlayCache cache;
      cache.Refresh(gui::MakeAnnotationViewKey(gui::AnnotationViewInputFor(gui::g_state, gui::g_state.renderer),
                                               kProbeW, kProbeH));
      IM_CHECK(cache.HasResult());

      gui::PreviewParams params = BaseParams();
      FillMarkerOverlay(params, cache, gui::g_state);
      const std::vector<unsigned char> without_names = RenderFrame(ctx, params, {});
      IM_CHECK(!without_names.empty());

      for (gui::MarkerAppearance& m : gui::g_state.markers) {
        m.label = true;
      }
      const std::vector<gui::CurveLabelSet> sets =
          gui::BuildMarkerLabelSets(cache, gui::g_state, static_cast<float>(kProbeW), static_cast<float>(kProbeH));
      // One set per marker, each carrying ONE anchor and its own colour. The shape is the claim:
      // CurveLabelSet's colour is per set, so six names in six colours is six sets, and a builder
      // that merged them would be unable to draw what the rows promise.
      IM_CHECK_EQ(sets.size(), static_cast<std::size_t>(LUMICE_ANNOTATION_MARKER_COUNT));
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        if (sets[static_cast<std::size_t>(i)].anchors.size() != 1u) {
          IM_ERRORF("marker %s: %zu anchors, expected exactly one", gui::kMarkerSerialNames[i],
                    sets[static_cast<std::size_t>(i)].anchors.size());
          break;
        }
        if (sets[static_cast<std::size_t>(i)].anchors[0].text != gui::kMarkerDisplayNames[i]) {
          IM_ERRORF("marker %s: label reads \"%s\"", gui::kMarkerSerialNames[i],
                    sets[static_cast<std::size_t>(i)].anchors[0].text.c_str());
          break;
        }
      }
      if (ctx->IsError()) {
        return;
      }

      const std::vector<unsigned char> with_names = RenderFrame(ctx, params, sets);
      IM_CHECK(!with_names.empty());

      // Each name lands NEAR ITS OWN RING, which is the half a global "more pixels are lit" count
      // cannot say: six names all stacked in one corner would raise that count just as well. The
      // window is generous (the label sits a ring's radius plus a gap below the centre, and the
      // collision pass may nudge it further) and still local — the fixture asserts below that no
      // two rings are within two windows of each other, so a name found here belongs to this ring.
      constexpr int kHalfWindow = 40;
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        const float* pos = params.overlay.marker_screen_pos[i].data();
        const int before = NonBlackCountAround(without_names, pos[0], pos[1], kHalfWindow);
        const int after = NonBlackCountAround(with_names, pos[0], pos[1], kHalfWindow);
        if (after <= before) {
          IM_ERRORF("marker %s: %d lit pixels near its ring with names off, %d with them on",
                    gui::kMarkerSerialNames[i], before, after);
          break;
        }
      }
      if (ctx->IsError()) {
        return;
      }
      // The separation the windows above rest on. Stated rather than assumed, because it is a
      // property of the FIXTURE'S VIEW: a scene whose points crowded would make every window
      // overlap and quietly turn the per-marker claim back into the global one.
      for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
        for (int j = i + 1; j < LUMICE_ANNOTATION_MARKER_COUNT; ++j) {
          const float dx = params.overlay.marker_screen_pos[i][0] - params.overlay.marker_screen_pos[j][0];
          const float dy = params.overlay.marker_screen_pos[i][1] - params.overlay.marker_screen_pos[j][1];
          const float dist = std::sqrt(dx * dx + dy * dy);
          if (dist < 2.0f * kHalfWindow) {
            IM_ERRORF("fixture: %s and %s are %.1f px apart, closer than the %d px window", gui::kMarkerSerialNames[i],
                      gui::kMarkerSerialNames[j], static_cast<double>(dist), 2 * kHalfWindow);
            break;
          }
        }
        if (ctx->IsError()) {
          break;
        }
      }
    };
  }

  // ===============================================================================================
  // The third state of a row.
  // ===============================================================================================
  //
  // Read as PIXELS of the row's name cell, because that is where the difference is and because the
  // difference has no other reading: ImGui::TextUnformatted and ImGui::TextDisabled submit no
  // addressable item, so a test engine lookup answers the same thing for both.
  //
  // Two comparisons, and they are different claims. Against "switched off" is AC7's literal
  // question. Against "switched on and imaged" is the harder one and the one the feature exists
  // for: the tick is identical in those two, so anything that told them apart has to be the name.
  {
    ImGuiTest* t =
        IM_REGISTER_TEST(engine, "marker_panel", "a_cropped_reference_point_reads_differently_from_an_unticked_one");
    t->GuiFunc = MarkerPanelGuiFunc;
    t->TestFunc = [](ImGuiTestContext* ctx) {
      const int subsun = LUMICE_ANNOTATION_MARKER_SUBSUN;

      // Capture the subsun row's NAME CELL under three states. The cell is bounded by two items that
      // ARE addressable — the colour swatch on its left and the Line checkbox on its right — so the
      // rect is read off the frame rather than computed from column widths.
      auto capture_name_cell = [ctx](std::vector<unsigned char>* out, int* out_w, int* out_h) {
        ctx->Yield(3);
        // Park the mouse off-window: a hover highlight in one capture and not another would make
        // every comparison below true for the wrong reason.
        ctx->MouseMoveToPos(ImVec2(-100.0f, -100.0f));
        ctx->Yield(3);

        ImGuiTestItemInfo line;
        ImGuiTestItemInfo swatch;
        {
          const ScopedRefForMarkers panel_ref(ctx, "//##RightPanel");
          // ScrollToItemY before ItemInfo, not merely ItemInfo: the Overlay group is the last one
          // in a scrollable panel and ItemInfo does NOT scroll — it answers with the rect the item
          // was submitted at, clipped or not. Read without this, the rect below lands outside the
          // framebuffer and the capture is a garbage region rather than the row.
          ctx->ScrollToItemY("**/##marker_line_subsun");
          ctx->Yield(3);
          line = ctx->ItemInfo("**/##marker_line_subsun");
          swatch = ctx->ItemInfo(MarkerSwatchId(line, "subsun"), ImGuiTestOpFlags_NoError);
        }
        // Fatal to this helper, which returns; each caller then checks ctx->IsError() before using
        // what it handed back — the same discipline test_overlay_controls.cpp's helpers follow, and
        // the reason a failure here ends the case instead of comparing three empty buffers.
        IM_CHECK(line.ID != 0);
        IM_CHECK(swatch.ID != 0);

        // Window and framebuffer geometry from ImGui's IO rather than GLFW directly:
        // glfwGetCurrentContext() is thread-local and returns null on the test coroutine's thread.
        const ImGuiIO& io = ImGui::GetIO();
        const float sx = io.DisplayFramebufferScale.x;
        const float sy = io.DisplayFramebufferScale.y;
        const float win_h = io.DisplaySize.y;
        const ImVec2 vp_pos = ImGui::GetMainViewport()->Pos;
        const float lx = swatch.RectFull.Max.x - vp_pos.x;
        const float rx_end = line.RectFull.Min.x - vp_pos.x;
        const float ly = line.RectFull.Min.y - vp_pos.y;
        const float lh = line.RectFull.Max.y - line.RectFull.Min.y;

        g_fullframe_capture.Reset();
        g_fullframe_capture.rect_x = static_cast<int>(std::lround(lx * sx));
        // ImGui (origin top-left, window coords) -> glReadPixels (origin bottom-left, framebuffer).
        g_fullframe_capture.rect_y = static_cast<int>(std::lround((win_h - (ly + lh)) * sy));
        g_fullframe_capture.rect_w = static_cast<int>(std::lround((rx_end - lx) * sx));
        g_fullframe_capture.rect_h = static_cast<int>(std::lround(lh * sy));
        IM_CHECK_GT(g_fullframe_capture.rect_w, 0);
        IM_CHECK_GT(g_fullframe_capture.rect_h, 0);
        // Inside the framebuffer, as a machine check rather than a look at the picture: a row
        // scrolled out of view still reports a rect, and a capture of a region that is partly
        // off-screen would come back internally consistent and compare equal to another one.
        IM_CHECK_GE(g_fullframe_capture.rect_x, 0);
        IM_CHECK_GE(g_fullframe_capture.rect_y, 0);
        fprintf(stderr, "[marker_panel] name-cell capture: has_result=%d subsun_valid=%d show=%d rect=(%d,%d,%d,%d)\n",
                gui::PreviewAnnotationOverlay().HasResult() ? 1 : 0,
                gui::PreviewAnnotationOverlay().MarkerPoint(LUMICE_ANNOTATION_MARKER_SUBSUN).valid ? 1 : 0,
                gui::g_state.markers[LUMICE_ANNOTATION_MARKER_SUBSUN].show ? 1 : 0, g_fullframe_capture.rect_x,
                g_fullframe_capture.rect_y, g_fullframe_capture.rect_w, g_fullframe_capture.rect_h);
        g_fullframe_capture.requested.store(true);
        for (int i = 0; i < 10 && !g_fullframe_capture.done.load(); ++i) {
          ctx->Yield(1);
        }
        IM_CHECK(g_fullframe_capture.done.load());
        *out = g_fullframe_capture.pixels;
        *out_w = g_fullframe_capture.width;
        *out_h = g_fullframe_capture.height;
      };

      // State 1 — ticked, and the view images it. Full sky, so nothing is cropped away.
      ResetTestState();
      EnsurePreviewTexture(ctx);
      InstallFullSkyView(gui::g_state);
      gui::g_state.markers_section_open = true;
      gui::g_state.markers[subsun].show = true;
      std::vector<unsigned char> ticked_visible;
      int w1 = 0;
      int h1 = 0;
      capture_name_cell(&ticked_visible, &w1, &h1);
      if (ctx->IsError()) {
        return;
      }

      // State 2 — ticked, and the view crops it away. The subsun is BELOW the horizon by
      // construction (it is the sun's reflection), so an upper-hemisphere crop removes it while
      // leaving the tick exactly where state 1 had it.
      gui::g_state.renderer.lens_type = gui::kLensTypeFisheyeEqualArea;
      gui::g_state.renderer.fov = 180.0f;
      gui::g_state.renderer.visible = gui::kVisibleUpper;
      std::vector<unsigned char> ticked_cropped;
      int w2 = 0;
      int h2 = 0;
      capture_name_cell(&ticked_cropped, &w2, &h2);
      if (ctx->IsError()) {
        return;
      }
      // The premise. Without it the two captures would differ for no reason at all and the case
      // would be asserting noise.
      IM_CHECK(gui::PreviewAnnotationOverlay().HasResult() ||
               !gui::PreviewAnnotationOverlay().MarkerPoint(subsun).valid);

      // State 3 — not ticked, same cropped view.
      gui::g_state.markers[subsun].show = false;
      std::vector<unsigned char> unticked;
      int w3 = 0;
      int h3 = 0;
      capture_name_cell(&unticked, &w3, &h3);
      if (ctx->IsError()) {
        return;
      }

      IM_CHECK_EQ(w1, w2);
      IM_CHECK_EQ(h1, h2);
      IM_CHECK_EQ(w1, w3);
      IM_CHECK_EQ(h1, h3);
      IM_CHECK_EQ(ticked_visible.size(), ticked_cropped.size());
      IM_CHECK_EQ(ticked_visible.size(), unticked.size());
      IM_CHECK(!ticked_visible.empty());

      auto differing_bytes = [](const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
        int n = 0;
        for (std::size_t i = 0; i < a.size(); ++i) {
          if (std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i])) > 8) {
            n++;
          }
        }
        return n;
      };

      // The harder claim: cropped and imaged look different even though the tick is the same.
      IM_CHECK_GT(differing_bytes(ticked_visible, ticked_cropped), 0);
      // AC7's literal one: cropped and unticked look different too.
      IM_CHECK_GT(differing_bytes(ticked_cropped, unticked), 0);
    };
  }
}
