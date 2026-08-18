// The inspector's form layout — the property row (BeginPropertyTable / PropertyRow, panels.cpp)
// and the width tokens it is built on (gui_constants.hpp).
//
// What this suite is for. The migration these cases guard replaced "every control is as wide as
// whatever container it happens to sit in" with "every control is one of a few named widths". That
// is a claim about geometry, and geometry is only observable in a real frame — which is why the
// cases live here rather than in gui_unit_test. Two propositions, and neither is a count of how
// many call sites adopted the helper:
//
//   1. The control column starts at the same x on every row of a page and on every page, so a
//      column of settings reads as one form. A regression here is a call site that re-stated its
//      own width, or a token that stopped being the thing deciding.
//   2. All the pages' scalar fields end up at ONE width, not five (the measured pre-migration
//      spread was 90 / 110 / 230 / 240 / 600 px for controls that all edit a single number).
//
// And one calibration guard: kPropertyLabelColWidth is a MEASURED value — the widest label any
// inspector page draws, in the theme's font. Nothing in the build recomputes it, so a font change
// or a longer label would silently start clipping labels. The first case re-derives the
// requirement from the live font atlas and fails with both numbers in the message.
//
// What a user sees when these break: labels truncated mid-word, or a form whose controls start at
// a different x on every second row.

#include <algorithm>
#include <string>
#include <vector>

#include "gui/gui_constants.hpp"
#include "test_gui_shared.hpp"

namespace {

// Every label the inspector's six pages pass to PropertyRow.
//
// MAINTAINER: this list is hand-kept, and deliberately so — the alternative is asking the UI what
// it drew, which for a label means asking ImGui about an item that has no id. It therefore proves
// only what it lists: it is a guard against the FONT growing under a fixed token, not a proof that
// no page anywhere has a longer label. Add a row's label here when you add the row.
const char* const kInspectorRowLabels[] = {
  // Sun
  "Altitude",
  "Diameter",
  "Spectrum",
  // Camera
  "Lens Type",
  "FOV",
  "Elevation",
  "Azimuth",
  "Roll",
  // Layer
  "Prob.",
  // Crystal (entry + shape page)
  "Weight",
  "Type",
  // Axis (per-row distribution combo + its two scalars, worst case over the type-dependent name)
  "Zenith",
  "Mean",
  "Std",
  "Range",
  "Amplitude",
  "Scale",
  // Filter
  "Action",
  "Symmetry",
};

// Scalar-field control ids across the pages, as they exist AFTER the [slider][input] merge: one
// DragFloat per field, id "##<label>" with no _slider / _input half.
struct ScalarProbe {
  const char* page;  // for the failure message — which page's control disagreed
  const char* ref;
};

// The inspector window, as a test ref root. Every property table below is addressed relative to it.
const char* const kInspectorRef = "//##DocumentInspector";

// One layer holding one crystal, so the Crystal page has something to show. Local rather than
// shared: the only property this suite needs from the scene is that a crystal entry exists.
void BuildOneCrystalScene() {
  auto& s = gui::g_state;
  s.layers.clear();
  s.crystals.clear();
  s.filters.clear();
  gui::Layer layer;
  gui::EntryCard entry;
  entry.crystal_id = 0;
  s.crystals.emplace_back();
  layer.entries.push_back(entry);
  layer.probability = 0.0f;  // single layer: all filter-pass rays are output
  s.layers.push_back(std::move(layer));
  s.SelectNone();
  gui::g_thumbnail_cache.OnLayerStructureChanged();
}

}  // namespace

void RegisterPropertyRowTests(ImGuiTestEngine* engine) {
  // The token has to fit the labels it is asked to right-align. Derived from the live atlas rather
  // than restated, because the whole point of the constant is that it was measured once.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "property_row", "the_label_column_fits_every_inspector_label");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      ctx->Yield(2);

      float widest = 0.0f;
      std::string widest_label;
      for (const char* label : kInspectorRowLabels) {
        const float w = ImGui::CalcTextSize(label).x;
        if (w > widest) {
          widest = w;
          widest_label = label;
        }
      }
      ctx->LogInfo("widest inspector label: \"%s\" = %.2f px; kPropertyLabelColWidth = %.1f px", widest_label.c_str(),
                   widest, gui::kPropertyLabelColWidth);
      IM_CHECK_LE(widest, gui::kPropertyLabelColWidth);
      // Non-vacuous in the other direction too: a token much wider than its widest label re-opens
      // the proximity gap that sank the "one shared label column" direction
      // (doc/gui-visual-language.md §5). One rhythm step (4 px) of slack is the allowance.
      IM_CHECK_LE(gui::kPropertyLabelColWidth - widest, 4.0f);
    };
  }

  // Proposition 1: one x for every control on a page.
  //
  // Addressed by explicit path rather than the "**/" wildcard: the wildcard resolves an item by the
  // LABEL it reported to the test engine, and ImGui::Combo's preview button reports none — so a
  // combo row, which is exactly the kind of row this case has to include, is unreachable that way.
  // The explicit form also spells out the property table each row belongs to, which is the thing
  // that decides the shared x.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "property_row", "a_pages_controls_all_start_at_one_x");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      gui::g_state.SelectSun();
      ctx->Yield(3);
      ctx->SetRef(kInspectorRef);

      // Collected first, compared after: a fatal assert inside the loop would hide every row past
      // the first disagreement, which is the one thing a "do they all agree" case must not do.
      const char* const kSunRefs[] = { "##sun_props/##Altitude", "##sun_props/##Diameter", "##sun_props/##Spectrum" };
      std::vector<float> left_edges;
      std::vector<std::string> missing;
      for (const char* ref : kSunRefs) {
        const ImGuiTestItemInfo info = ctx->ItemInfo(ref, ImGuiTestOpFlags_NoError);
        if (info.ID == 0) {
          missing.emplace_back(ref);
        } else {
          left_edges.push_back(info.RectFull.Min.x);
        }
      }
      for (const std::string& ref : missing) {
        ctx->LogError("row control not found: %s", ref.c_str());
      }
      for (size_t i = 1; i < left_edges.size(); ++i) {
        if (left_edges[i] != left_edges[0]) {
          ctx->LogError("row %d starts at x=%.1f, row 0 at x=%.1f", (int)i, left_edges[i], left_edges[0]);
        }
      }
      ctx->SetRef("");
      IM_CHECK_EQ(missing.size(), (size_t)0);
      IM_CHECK_EQ(left_edges.size(), (size_t)IM_ARRAYSIZE(kSunRefs));
      const size_t agreeing = (size_t)std::count(left_edges.begin(), left_edges.end(), left_edges[0]);
      IM_CHECK_EQ(agreeing, left_edges.size());
    };
  }

  // Proposition 2: one width for every scalar field, across pages. This is the AC the migration is
  // actually for — "how many call sites use the helper" would pass while five different widths
  // survived, so the measurement is of the widths themselves.
  {
    ImGuiTest* t = IM_REGISTER_TEST(engine, "property_row", "every_inspector_scalar_field_is_one_width");
    t->TestFunc = [](ImGuiTestContext* ctx) {
      ResetTestState();
      BuildOneCrystalScene();
      ctx->Yield(2);
      ctx->SetRef(kInspectorRef);

      struct Sample {
        std::string page;
        float width;
      };
      std::vector<Sample> samples;
      std::vector<std::string> missing;

      const ScalarProbe kSun[] = { { "Sun", "##sun_props/##Altitude" }, { "Sun", "##sun_props/##Diameter" } };
      const ScalarProbe kCamera[] = { { "Camera", "##cam_lens/##FOV##view" },
                                      { "Camera", "##cam_pose/##Elevation##view" },
                                      { "Camera", "##cam_pose/##Azimuth##view" },
                                      { "Camera", "##cam_pose/##Roll##view" } };
      const ScalarProbe kCrystal[] = { { "Crystal", "##entry_props/##Weight##prop_0_0" } };

      auto collect = [&](const ScalarProbe* probes, int count) {
        for (int i = 0; i < count; ++i) {
          const ImGuiTestItemInfo info = ctx->ItemInfo(probes[i].ref, ImGuiTestOpFlags_NoError);
          if (info.ID == 0) {
            missing.emplace_back(probes[i].ref);
          } else {
            samples.push_back({ probes[i].page, info.RectFull.GetWidth() });
          }
        }
      };

      gui::g_state.SelectSun();
      ctx->Yield(3);
      collect(kSun, IM_ARRAYSIZE(kSun));

      gui::g_state.SelectCamera();
      ctx->Yield(3);
      collect(kCamera, IM_ARRAYSIZE(kCamera));

      gui::g_state.SelectCrystal(0, 0);
      ctx->Yield(3);
      collect(kCrystal, IM_ARRAYSIZE(kCrystal));

      ctx->SetRef("");
      for (const std::string& ref : missing) {
        ctx->LogError("scalar control not found: %s", ref.c_str());
      }
      IM_CHECK_EQ(missing.size(), (size_t)0);
      IM_CHECK(!samples.empty());
      int disagreeing = 0;
      for (const Sample& s : samples) {
        if (s.width != samples[0].width) {
          ++disagreeing;
          ctx->LogError("%s field is %.1f px wide; the first sample was %.1f px", s.page.c_str(), s.width,
                        samples[0].width);
        }
      }
      IM_CHECK_EQ(disagreeing, 0);
    };
  }
}
