// The two defaults-panel cases that never touch a widget.
//
// Their 33 siblings in test/gui/functional/test_gui_defaults_panel.cpp drive the real Settings
// modal through ImGuiTestContext and stay there. These two ask questions about the panel's DATA
// rather than about its UI — which keys the field-editor registry covers, and when the row tint
// predicate fires — so they run windowless:
//   inline_ac3 — the registry and the real row set of a factory document agree in both
//     directions: every row is either registered with the expected editor kind or deliberately
//     unregistered, and nothing is registered that the document cannot produce
//   inline_ac5b — RowWouldChangeOnSave over the four states a row can be in

#include <gtest/gtest.h>

#include <cstddef>
#include <set>
#include <string>

#include "gui/defaults_diff.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/gui_state.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"

namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;
using lumice::test_user_defaults::ScopedUserConfigSource;

}  // namespace

// AC3 — coverage as a COUNTABLE deliverable, not "the mechanism works".
//
// Every leaf of the serialized document is named below with the control it must be edited
// with, and the case fails on the first row whose classification disagrees. A count would not
// do: it passes just as well when two fields swap classes, and it says nothing at all about
// WHICH field regressed. The two deliberately-unregistered leaves are named here too, so
// "read-only" is an asserted decision rather than an omission that looks like one.
TEST(DefaultsPanel, inline_ac3_registry_covers_every_row) {
  // The freshly emptied explicit dir below is what makes MakeNewDocumentState() deterministic;
  // gui_test's ResetTestState() was scaffolding around that, and nothing here reads what it set.
  const auto dir = FreshOverlayDir("panel_inline_ac3");
  ScopedUserConfigSource guard(gui::UserConfigSource::kExplicitDir, dir);
  ResetUserDefaultsChannels();

  using Kind = gui::FieldEditorKind;
  struct Expected {
    const char* key_path;
    bool registered;
    Kind kind;  // ignored when !registered
  };
  // Order follows SerializeGuiStateJson so the two can be read side by side.
  static const Expected kExpected[] = {
    { "sun.altitude", true, Kind::kFloatSlider },
    { "sun.diameter", true, Kind::kFloatSlider },
    { "sun.spectrum", true, Kind::kCombo },
    { "sim.ray_num_millions", true, Kind::kFloatSlider },
    { "sim.max_hits", true, Kind::kIntSlider },
    { "sim.infinite", true, Kind::kCheckbox },
    { "renderer.lens_type", true, Kind::kCombo },
    { "renderer.fov", true, Kind::kFloatSlider },
    { "renderer.elevation", true, Kind::kFloatSlider },
    { "renderer.azimuth", true, Kind::kFloatSlider },
    { "renderer.roll", true, Kind::kFloatSlider },
    // The named special case: serialized as a VALUE (1024) while its control edits an index.
    // Registered as editable — the registry translates — rather than left read-only.
    { "renderer.sim_resolution", true, Kind::kCombo },
    { "renderer.visible", true, Kind::kCombo },
    { "renderer.front", true, Kind::kCheckbox },
    { "renderer.background", true, Kind::kColor },
    { "renderer.ray_color", true, Kind::kColor },
    { "renderer.opacity", true, Kind::kFloatSlider },
    { "renderer.exposure_offset", true, Kind::kFloatSlider },
    { "aspect_ratio", true, Kind::kCombo },
    { "aspect_portrait", true, Kind::kCheckbox },
    { "bg_path", false, Kind::kCheckbox },
    { "bg_show", true, Kind::kCheckbox },
    { "bg_alpha", true, Kind::kFloatSlider },
    { "overlay_horizon_line", true, Kind::kCheckbox },
    { "overlay_horizon_label", true, Kind::kCheckbox },
    { "overlay_grid_line", true, Kind::kCheckbox },
    { "overlay_grid_label", true, Kind::kCheckbox },
    { "overlay_sun_circles_line", true, Kind::kCheckbox },
    { "overlay_sun_circles_label", true, Kind::kCheckbox },
    { "overlay_sun_circle_angles", false, Kind::kCheckbox },
    { "overlay_horizon_color", true, Kind::kColor },
    { "overlay_grid_color", true, Kind::kColor },
    { "overlay_sun_circles_color", true, Kind::kColor },
    { "overlay_horizon_alpha", true, Kind::kFloatSlider },
    { "overlay_grid_alpha", true, Kind::kFloatSlider },
    { "overlay_sun_circles_alpha", true, Kind::kFloatSlider },
    { "overlay_zenith_nadir_line", true, Kind::kCheckbox },
    { "overlay_zenith_nadir_color", true, Kind::kColor },
    { "overlay_zenith_nadir_alpha", true, Kind::kFloatSlider },
    { "overlay_zenith_nadir_radius_px", true, Kind::kFloatSlider },
    { "right_panel_collapsed", true, Kind::kCheckbox },
    { "modal_layout_vertical", true, Kind::kCheckbox },
  };

  // Against the REAL row set of a factory document, so this cannot drift from what the panel
  // actually lists: a field added to the serializer makes the row set larger than the table
  // below and fails here, which is the reminder to classify it.
  // A local document, not gui::g_state: BuildDefaultDiffRows takes the state it reads as a
  // parameter, so routing through the global would only add a way for a neighbouring case to
  // change this one's answer.
  const gui::GuiState document = gui::MakeNewDocumentState();
  const auto rows = gui::BuildDefaultDiffRows(document);
  std::set<std::string> row_keys;
  for (const auto& row : rows) {
    row_keys.insert(row.key_path);
  }
  EXPECT_EQ(row_keys.size(), std::size(kExpected));

  for (const auto& expected : kExpected) {
    EXPECT_TRUE(row_keys.count(expected.key_path) == 1);
    const gui::FieldEditorEntry* entry = gui::FindFieldEditor(expected.key_path);
    if (expected.registered) {
      EXPECT_TRUE(entry != nullptr);
      EXPECT_TRUE(entry->kind == expected.kind);
    } else {
      EXPECT_TRUE(entry == nullptr);
    }
  }

  // Nothing is registered that the document does not produce: an entry for a key that no longer
  // exists is dead weight the panel can never reach, and it would silently survive the loop
  // above.
  for (const auto& key : gui::RegisteredFieldEditorKeyPaths()) {
    EXPECT_TRUE(row_keys.count(key) == 1);
  }
}

// AC5b — the row tint's predicate, over the four states the issue enumerates.
//
// Asserted on the predicate rather than on pixels, deliberately and with the gap named: ImGui
// keeps a table's per-row background colour only for the row being submitted, so there is no
// frame-independent handle on "row N was tinted". What binds the predicate to what is PAINTED
// is the defaults_panel_layout reference group — the pending_changes scene contains tinted rows
// at a 40 dB floor, so a call site that stopped tinting turns that scene red.
//
// The fourth state is the one this whole predicate exists for: a key saved long ago, untouched
// since. It differs from factory forever, yet Save would not move it, so it must NOT be tinted.
TEST(DefaultsPanel, inline_ac5b_row_tint_predicate_four_states) {
  gui::DefaultDiffRow row;
  row.key_path = "probe";

  // (1) nothing saved, nothing adopted — Save writes nothing for this key.
  row.current_value = 0.5f;
  row.default_value = 0.5f;
  row.factory_value = 0.5f;
  row.has_saved_override = false;
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

  // (2) the same row, adopted: Save would ADD the key. Tinted even though the value equals the
  // factory one — presence is a change.
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/true));

  // (3) changed in the GUI, never saved: adopted by default, and Save would write the new
  // value.
  row.current_value = 0.9f;
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/true));
  // ...and un-checking it takes it back to "Save writes nothing", which is also no change.
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/false));

  // (4) saved earlier, untouched since: differs from factory, equals what is on disk. NOT
  // tinted — the state that separates this predicate from the "Differs from factory" filter.
  row.current_value = 0.9f;
  row.default_value = 0.9f;  // has_saved_override ⇒ default_value IS the stored value
  row.factory_value = 0.5f;
  row.has_saved_override = true;
  EXPECT_TRUE(!gui::RowWouldChangeOnSave(row, /*checked=*/true));
  // Un-checking it would REMOVE the key from the file — a change.
  EXPECT_TRUE(gui::RowWouldChangeOnSave(row, /*checked=*/false));
}
