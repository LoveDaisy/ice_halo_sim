// The GUI half of "print takes over the colour channel" (doc/print-mode-subtractive-ink.md §7),
// asked the questions that need no window.
//
// The rule has four instances, and the GUI's job for each is to make the exclusion VISIBLE — a
// field that silently stops mattering is the failure the whole rule exists to prevent. Three of the
// four have a control to grey out, and this file owns the gate that greys them: whether
// ConstraintFor reports the field as inapplicable, and whether the reason it gives names Print. The
// fourth (renderer.ray_color) has no control anywhere in the GUI, so its visible half is an import
// notice instead, and that is asserted here too.
//
// Deliberately NOT here, and not restated: whether the greyed state actually reaches the widget on
// screen is test/gui/functional/test_color_window.cpp (the two composite mirrors) and
// test/gui/functional/test_background_overlay.cpp (the background photo, in exported pixels).
// Whether the CLI honours the same rule is
// test/unit-correctness/server/test_print_mode_colour_exclusions.cpp.
//
// Every case carries its tone=screen arm. The gate is one `if` on a field that is 0 in every other
// test in this binary, so "it did not fire" is the easy half to get right by accident; what has to
// be shown is that it fires for print AND ONLY for print.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"
#include "gui/field_editor_registry.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "include/lumice.h"

namespace gui = lumice::gui;

namespace {

gui::GuiState StateWithTone(int tone) {
  gui::GuiState state;
  state.renderer.tone = tone;
  return state;
}

bool Mentions(const char* haystack, const char* needle) {
  return haystack != nullptr && std::string(haystack).find(needle) != std::string::npos;
}

// The three overlay families whose colour doc §7 lists, plus one marker. Checked as a group rather
// than one case each because they are one rule with one gate — a per-family case would suggest
// three independent decisions and would still not catch the interesting failure, which is a family
// left off the list.
const char* const kGatedColourFields[] = {
  "overlay_horizon_color",
  "overlay_grid_color",
  "overlay_sun_circles_color",
};

}  // namespace

// Instance 4. Under print the CLI draws every overlay line by ink density and never reads these
// colours (test_render_consumer_print_mode.cpp owns that half), so a swatch that still invited an
// edit would be offering a control over nothing.
TEST(PrintModeColourExclusionsGui, OverlayColourSwatchesAreInapplicableUnderPrint) {
  const gui::GuiState print = StateWithTone(LUMICE_TONE_PRINT);
  const gui::GuiState screen = StateWithTone(LUMICE_TONE_SCREEN);

  for (const char* key : kGatedColourFields) {
    const gui::FieldEditorConstraint under_print = gui::ConstraintFor(key, print);
    EXPECT_FALSE(under_print.enabled) << key;
    EXPECT_TRUE(Mentions(under_print.disabled_reason, "Print")) << key << ": " << under_print.disabled_reason;

    EXPECT_TRUE(gui::ConstraintFor(key, screen).enabled) << key << " must be editable under tone=screen";
  }

  // The marker family reaches the same gate through its registration loop, so one member is enough
  // to show the loop passes the gate on; a member-by-member sweep would only re-test the loop.
  const std::string marker_key = gui::MarkerFieldKey(0, gui::MarkerKeyPart::kColor);
  EXPECT_FALSE(gui::ConstraintFor(marker_key, print).enabled);
  EXPECT_TRUE(Mentions(gui::ConstraintFor(marker_key, print).disabled_reason, "Print"));
  EXPECT_TRUE(gui::ConstraintFor(marker_key, screen).enabled);
}

// The scope boundary, asserted rather than left to the reader of the registry. The lens border is a
// colour that print does not read either, and it is STILL not gated — because doc §7 enumerates
// four instances and it is not one of them. Without this case, "gate every colour that print
// ignores" would pass every other case in this file, and the boundary would quietly move.
TEST(PrintModeColourExclusionsGui, TheLensBorderColourIsNotGated) {
  EXPECT_TRUE(gui::ConstraintFor("overlay_lens_border_color", StateWithTone(LUMICE_TONE_PRINT)).enabled);
  EXPECT_TRUE(gui::ConstraintFor("overlay_lens_border_color", StateWithTone(LUMICE_TONE_SCREEN)).enabled);
}

// Instance 1's visible half. The reason matters as much as the flag here: with no image loaded
// (which is this binary's state — uploading one needs a GL context) BOTH reasons are true, and a
// gate that reported "No background image is loaded" under print would send the user off to load
// one, which would not enable the control. So the case discriminates on the TEXT, and its screen
// arm asserts the other reason is still the one given.
TEST(PrintModeColourExclusionsGui, TheBackgroundPhotoIsInapplicableUnderPrintForThePrintReason) {
  const gui::FieldEditorConstraint under_print = gui::ConstraintFor("bg_show", StateWithTone(LUMICE_TONE_PRINT));
  EXPECT_FALSE(under_print.enabled);
  EXPECT_TRUE(Mentions(under_print.disabled_reason, "Print")) << under_print.disabled_reason;

  const gui::FieldEditorConstraint under_screen = gui::ConstraintFor("bg_show", StateWithTone(LUMICE_TONE_SCREEN));
  EXPECT_FALSE(Mentions(under_screen.disabled_reason, "Print"))
      << "tone=screen must fall through to the ordinary reason, got: " << under_screen.disabled_reason;

  // The four transform fields inherit the gate through WhenBackgroundShown rather than declaring it
  // again. Asserted because that inheritance is the reason they were not touched.
  for (const char* key : { "bg_alpha", "bg_offset_x", "bg_offset_y", "bg_scale" }) {
    const gui::FieldEditorConstraint c = gui::ConstraintFor(key, StateWithTone(LUMICE_TONE_PRINT));
    EXPECT_FALSE(c.enabled) << key;
    EXPECT_TRUE(Mentions(c.disabled_reason, "Print")) << key << ": " << c.disabled_reason;
  }
}

namespace {

// A minimal CORE config document — the only import path by which renderer.ray_color can enter the
// GUI at all (both BuildScene arms pin it to core's sentinel, so a GUI session can read this
// pairing but never write one).
std::string CoreConfigDoc(const std::string& render_extra, const char* tone) {
  return std::string(R"({
    "crystal": [{"id": 1, "type": "prism", "shape": {"height": 1.0}}],
    "filter": [],
    "scene": {
      "light_source": {"type": "sun", "altitude": 20.0, "diameter": 0.5, "spectrum": "D65"},
      "ray_num": 1000, "max_hits": 4,
      "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
    },
    "render": [{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64], "tone": ")") +
         tone + "\"" + render_extra + "}]}";
}

// Import `doc` into a scratch state and return whatever notice it produced.
std::string ImportAndPeekNotice(const std::string& doc) {
  gui::ClearImportComplexFilterWarning();
  gui::GuiState scratch;
  EXPECT_TRUE(gui::DeserializeFromJson(doc, scratch));
  const std::string notice = gui::PeekImportComplexFilterWarning();
  gui::ClearImportComplexFilterWarning();
  return notice;
}

}  // namespace

// Instance 3. Four arms, because three different documents must all stay silent and only one must
// speak — the notice keys on "the document stated a tint AND the tone is print", and each of the
// three silent arms breaks exactly one conjunct.
TEST(PrintModeColourExclusionsGui, ARayColorTintImportedUnderPrintIsAnnounced) {
  const std::string spoke = ImportAndPeekNotice(CoreConfigDoc(R"(, "ray_color": [1.0, 0.0, 0.0])", "print"));
  EXPECT_NE(spoke.find("ray_color"), std::string::npos) << "the notice must name the field, got: " << spoke;

  EXPECT_TRUE(ImportAndPeekNotice(CoreConfigDoc(R"(, "ray_color": [1.0, 0.0, 0.0])", "screen")).empty())
      << "tone=screen reads the tint; there is nothing to announce";

  // The field's GUI default is {1,1,1}, NOT core's {-1,-1,-1} sentinel, so a decoder that judged
  // the loaded VALUE instead of the document would warn on every print import ever made.
  EXPECT_TRUE(ImportAndPeekNotice(CoreConfigDoc("", "print")).empty())
      << "a document that never mentioned ray_color asked for nothing";

  // The sentinel IS a stated value, and it states "no tint" — nothing is being dropped.
  EXPECT_TRUE(ImportAndPeekNotice(CoreConfigDoc(R"(, "ray_color": [-1.0, -1.0, -1.0])", "print")).empty())
      << "the {-1,-1,-1} sentinel is 'use the natural spectral colour', not a dropped tint";
}
