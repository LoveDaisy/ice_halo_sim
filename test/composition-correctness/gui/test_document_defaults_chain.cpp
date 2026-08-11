// Composition chain: what a document does NOT say.
//
// Units in the chain: file_io × gui_state.
//
// What the collaboration produces that is observable: the value a field takes when its key is
// absent from the document. Every deserialization fallback in file_io.cpp encodes one rule — an
// absent key leaves the owning struct's documented default in place — and the rule is only
// checkable across the two units together: file_io owns the fallback literal, gui_state owns the
// default it is supposed to equal, and nothing in either unit forces them to agree.
//
// When they stop agreeing nothing errors. A file written by an older build, or by hand, simply
// opens with a field set to a value its author never chose — a horizon line that is not the
// documented colour, a filter that matches something else, a crystal a little shorter than the
// default. The user sees a document that opened fine and is quietly not the one on disk.
//
// This is a real fault, not a hypothetical: commit 00fb12fc fixed one site where the struct default
// had moved and the loader's hardcoded literal had not. The same shape sat undetected at dozens of
// sibling sites, because the fallbacks EXECUTE on almost every document (line coverage reports them
// green) while nothing asserted the VALUE they produce.
//
// Two tables, not one: file_io has two independent parse entry points — DeserializeGuiStateJson
// (the GUI-native .lmc format) and DeserializeFromJson (the legacy core/CLI import) — and they do
// NOT agree on every field's missing-key semantics. Merging them would force a false choice about
// which path is "right"; keeping them apart lets each be pinned as it actually behaves.
//
// The expected value is evaluated HERE, from the owning struct, independently of file_io. That
// independence is the whole mechanism: a fallback spelled `j.value(k, Struct{}.field)` cannot drift
// from `Struct{}.field`, but a hardcoded literal can, and drift is what these tables catch.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// One row: a minimal document in which the key's PARENT object is present and the key itself is
// absent, plus the comparison that has to hold afterwards.
struct MissingKeyCase {
  const char* label;  // the comparison, stringified
  const char* doc;    // parent object PRESENT, the key ABSENT
  void (*check)(const GuiState& loaded);
};

// `actual` is written in terms of `s`; `expected` is read off the owning struct. The row's label is
// the two of them, stringified — not a hand-written description, which is one more thing that can
// drift from what the row actually checks. Containers are indexed with .at(), so a document that
// failed to produce the row's parent throws out of the case with the row's label already on the
// trace rather than reading past the end.
#define LUMICE_MISSING_KEY_ROW(doc_str, actual, expected)      \
  MissingKeyCase {                                             \
    #actual " == " #expected, doc_str, [](const GuiState& s) { \
      (void)s;                                                 \
      EXPECT_EQ(actual, expected);                             \
    }                                                          \
  }

// Documents reused across many rows. Named so a row is one line and the shape it needs is legible.
constexpr const char* kRoot = "{}";
constexpr const char* kSun = R"({"sun":{}})";
constexpr const char* kSim = R"({"sim":{}})";
constexpr const char* kRenderer = R"({"renderer":{}})";
// Every crystal below states its `type`, and has to. `type` is not a field with a missing-key
// default at all: it is the discriminant deciding how the rest of the crystal reads, so a crystal
// that omits it is refused outright and the entry falls back to a default POOL SLOT. Rows reading
// `s.crystals.at(0)` would then still find something to compare — an untouched slot — and pass
// without the parse under test having run. So the rows keep their subject the only absent key by
// stating `type`, and the refusal itself is pinned where it belongs, next to the other
// core-rejects-this-document cases (JsonImportContractChain in the sibling contract-chain file).
constexpr const char* kCrystal = R"({"layers":[{"entries":[{"crystal":{"type":"prism"}}]}]})";
// The filter document carries a raypath, and it has to. A filter object that names no rule at all
// is not loaded as a filter with default fields — it is loaded as no filter, so there would be no
// `s.filters.at(0)` for the rows below to read a default off. That disposition is a row of its own
// (kEmptyFilter, in the test body) rather than a silent property of this constant.
constexpr const char* kFilter =
    R"({"layers":[{"entries":[{"crystal":{"type":"prism"},"filter":{"raypath_text":"3-5"}}]}]})";
// entry_text carries a real value: an entry_exit filter naming neither face nor length is now the
// wildcard shape (FromLegacyEntryExit) and does not become a filter in the pool at all — that
// disposition is its own row below (kEmptyEeFilter, in the test body), same as kEmptyFilter for the
// raypath arm above it.
constexpr const char* kEeFilter =
    R"({"layers":[{"entries":[{"crystal":{"type":"prism"},"filter":{"type":"entry_exit","entry_text":"2"}}]}]})";

const MissingKeyCase kGuiNativeCases[] = {
  // -- root-level GuiState fields; the parent object is the document root, always present --
  LUMICE_MISSING_KEY_ROW(kRoot, s.aspect_preset, GuiState{}.aspect_preset),
  LUMICE_MISSING_KEY_ROW(kRoot, s.aspect_portrait, GuiState{}.aspect_portrait),
  LUMICE_MISSING_KEY_ROW(kRoot, s.bg_path, GuiState{}.bg_path),
  LUMICE_MISSING_KEY_ROW(kRoot, s.bg_show, GuiState{}.bg_show),
  LUMICE_MISSING_KEY_ROW(kRoot, s.bg_alpha, GuiState{}.bg_alpha),
  // The six line/label flags also pin the legacy `overlay_<x>` fallback chain: with the legacy key
  // absent too, each new key must land on its OWN struct default, not on a shared literal.
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_horizon_line, GuiState{}.show_horizon_line),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_horizon_label, GuiState{}.show_horizon_label),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_grid_line, GuiState{}.show_grid_line),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_grid_label, GuiState{}.show_grid_label),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_sun_circles_line, GuiState{}.show_sun_circles_line),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_sun_circles_label, GuiState{}.show_sun_circles_label),
  LUMICE_MISSING_KEY_ROW(kRoot, s.horizon_alpha, GuiState{}.horizon_alpha),
  LUMICE_MISSING_KEY_ROW(kRoot, s.grid_alpha, GuiState{}.grid_alpha),
  LUMICE_MISSING_KEY_ROW(kRoot, s.sun_circles_alpha, GuiState{}.sun_circles_alpha),
  LUMICE_MISSING_KEY_ROW(kRoot, s.show_zenith_nadir_line, GuiState{}.show_zenith_nadir_line),
  LUMICE_MISSING_KEY_ROW(kRoot, s.zenith_nadir_alpha, GuiState{}.zenith_nadir_alpha),
  LUMICE_MISSING_KEY_ROW(kRoot, s.zenith_nadir_radius_px, GuiState{}.zenith_nadir_radius_px),
  LUMICE_MISSING_KEY_ROW(kRoot, s.right_panel_collapsed, GuiState{}.right_panel_collapsed),
  LUMICE_MISSING_KEY_ROW(kRoot, s.modal_layout_vertical, GuiState{}.modal_layout_vertical),

  // -- root.sun / root.sim / root.renderer --
  LUMICE_MISSING_KEY_ROW(kSun, s.sun.altitude, SunConfig{}.altitude),
  LUMICE_MISSING_KEY_ROW(kSun, s.sun.diameter, SunConfig{}.diameter),
  LUMICE_MISSING_KEY_ROW(kSun, s.sun.spectrum_index, SunConfig{}.spectrum_index),
  LUMICE_MISSING_KEY_ROW(kSim, s.sim.ray_num_millions, SimConfig{}.ray_num_millions),
  LUMICE_MISSING_KEY_ROW(kSim, s.sim.max_hits, SimConfig{}.max_hits),
  LUMICE_MISSING_KEY_ROW(kSim, s.sim.infinite, SimConfig{}.infinite),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.lens_type, RenderConfig{}.lens_type),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.fov, RenderConfig{}.fov),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.elevation, RenderConfig{}.elevation),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.azimuth, RenderConfig{}.azimuth),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.roll, RenderConfig{}.roll),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.sim_resolution_index, RenderConfig{}.sim_resolution_index),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.visible, RenderConfig{}.visible),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.front, RenderConfig{}.front),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.opacity, RenderConfig{}.opacity),
  LUMICE_MISSING_KEY_ROW(kRenderer, s.renderer.exposure_offset, RenderConfig{}.exposure_offset),

  // -- root.layers[] (v2 inline format) --
  LUMICE_MISSING_KEY_ROW(R"({"layers":[{}]})", s.layers.at(0).probability, Layer{}.probability),
  LUMICE_MISSING_KEY_ROW(R"({"layers":[{"entries":[{}]}]})", s.layers.at(0).entries.at(0).proportion,
                         EntryCard{}.proportion),

  // -- the inline crystal (ParseCrystal, shared by both entry points) --
  // `crystal.type` has no row of its own in either table, and the absence is the statement: it is
  // the one crystal key whose omission is not answered with a default (see kCrystal above).
  LUMICE_MISSING_KEY_ROW(kCrystal, s.crystals.at(0).name, CrystalConfig{}.name),
  // ParseShapeDist: `type` is owned by ShapeDist; `center` comes from the per-scalar default the
  // CALL SITE passes, so it is pinned against that scalar's own struct default.
  LUMICE_MISSING_KEY_ROW(R"({"layers":[{"entries":[{"crystal":{"type":"prism","shape":{"height":{}}}}]}]})",
                         s.crystals.at(0).height.type, ShapeDist{}.type),
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"prism","shape":{"height":{"type":"uniform","mean":2.0}}}}]}]})",
      s.crystals.at(0).height.spread, ShapeDist{}.spread),
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"prism","shape":{"height":{"type":"uniform","std":0.5}}}}]}]})",
      s.crystals.at(0).height.center, CrystalConfig{}.height.center),
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"prism","shape":{"face_distance":[{"type":"uniform","std":0.1}]}}}]}]})",
      s.crystals.at(0).face_distance[0].center, CrystalConfig{}.face_distance[0].center),
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"pyramid","shape":{"prism_h":{"type":"uniform","std":0.1}}}}]}]})",
      s.crystals.at(0).prism_h.center, CrystalConfig{}.prism_h.center),
  // ParseAxisDist: `mean` / `std` are owned by AxisDist. `type` is pinned explicitly in each row so
  // the field under test is the only one the document leaves out.
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"prism","axis":{"zenith":{"type":"uniform","std":360.0}}}}]}]})",
      s.crystals.at(0).zenith.mean, AxisDist{}.mean),
  LUMICE_MISSING_KEY_ROW(
      R"({"layers":[{"entries":[{"crystal":{"type":"prism","axis":{"zenith":{"type":"uniform","mean":0.0}}}}]}]})",
      s.crystals.at(0).zenith.std, AxisDist{}.std),

  // -- the inline filter (ParseFilterFromGuiJson) --
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).name, FilterConfig{}.name),
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).action, FilterConfig{}.action),
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).sym_p, FilterConfig{}.sym_p),
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).sym_b, FilterConfig{}.sym_b),
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).sym_d, FilterConfig{}.sym_d),
  // No `type`: the legacy arm reads the document as a raypath, so the text it does carry survives.
  // (What happens when it carries no text either is the kEmptyFilter row in the test body — that
  // one is not a "missing key takes the struct default" statement at all, which is why it is not
  // here: the answer is that there is no filter to read a default off.)
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).param.size(), 1u),
  LUMICE_MISSING_KEY_ROW(kFilter, s.filters.at(0).param.at(0).text, std::string("3-5")),
  // Guard row, deliberately first of the EE group: EntryExitParamsValue() asserts on the wrong
  // variant arm, so a `type` that stopped selecting the EE arm would otherwise surface as a
  // bad_variant_access rather than as the plain statement that the arm is wrong.
  // entry_text itself is not tested for its own missing-key default here (mirrors kFilter/raypath_text
  // above it): entry_text is kEeFilter's anchor predicate, so its absence is the kEmptyEeFilter row
  // in the test body instead, not a "missing key takes the struct default" statement.
  LUMICE_MISSING_KEY_ROW(kEeFilter, s.filters.at(0).IsEntryExit(), true),
  LUMICE_MISSING_KEY_ROW(kEeFilter, s.filters.at(0).EntryExitParamsValue().exit_text, EntryExitParams{}.exit_text),
  LUMICE_MISSING_KEY_ROW(kEeFilter, s.filters.at(0).EntryExitParamsValue().length_mode, EntryExitParams{}.length_mode),
  LUMICE_MISSING_KEY_ROW(kEeFilter, s.filters.at(0).EntryExitParamsValue().min_len, EntryExitParams{}.min_len),
  LUMICE_MISSING_KEY_ROW(kEeFilter, s.filters.at(0).EntryExitParamsValue().max_len, EntryExitParams{}.max_len),

  // -- legacy v1 .lmc pool format (root.crystals + root.scattering) --
  LUMICE_MISSING_KEY_ROW(R"({"crystals":[],"scattering":[{}]})", s.layers.at(0).probability, Layer{}.probability),
  LUMICE_MISSING_KEY_ROW(R"({"crystals":[],"scattering":[{"entries":[{}]}]})", s.layers.at(0).entries.at(0).proportion,
                         EntryCard{}.proportion),
};

// Documents for the legacy core/CLI path, where a crystal needs an id and a scattering entry
// pointing at it before any of its fields exist to check.
constexpr const char* kCoreCrystal =
    R"({"crystal":[{"id":1,"type":"prism"}],"scene":{"scattering":[{"entries":[{"crystal":1}]}]}})";
constexpr const char* kCoreFilter = R"({"crystal":[{"id":1,"type":"prism"}],"filter":[{"id":1,"type":"raypath"}],
        "scene":{"scattering":[{"entries":[{"crystal":1,"filter":1}]}]}})";
constexpr const char* kCoreComplex = R"({"crystal":[{"id":1,"type":"prism"}],
        "filter":[{"id":1,"type":"raypath","raypath":[3,5]},{"id":2,"type":"complex","composition":[[1]]}],
        "scene":{"scattering":[{"entries":[{"crystal":1,"filter":2}]}]}})";
constexpr const char* kCoreColor =
    R"({"crystal":[{"id":1,"type":"prism"}],"scene":{"scattering":[{"entries":[{"crystal":1}]}]},
        "raypath_color":{"classes":[{"match":[{"crystal":1}]}]}})";

const MissingKeyCase kCoreJsonCases[] = {
  LUMICE_MISSING_KEY_ROW(R"({"scene":{}})", s.sim.max_hits, SimConfig{}.max_hits),
  LUMICE_MISSING_KEY_ROW(R"({"scene":{"light_source":{}}})", s.sun.altitude, SunConfig{}.altitude),
  LUMICE_MISSING_KEY_ROW(R"({"scene":{"light_source":{}}})", s.sun.diameter, SunConfig{}.diameter),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{"lens":{}}]})", s.renderer.lens_type, RenderConfig{}.lens_type),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{"lens":{}}]})", s.renderer.fov, RenderConfig{}.fov),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{"view":{}}]})", s.renderer.elevation, RenderConfig{}.elevation),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{"view":{}}]})", s.renderer.azimuth, RenderConfig{}.azimuth),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{"view":{}}]})", s.renderer.roll, RenderConfig{}.roll),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{}]})", s.renderer.front, RenderConfig{}.front),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{}]})", s.renderer.opacity, RenderConfig{}.opacity),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{}]})", s.renderer.exposure_offset, RenderConfig{}.exposure_offset),
  LUMICE_MISSING_KEY_ROW(kCoreCrystal, s.crystals.at(0).name, CrystalConfig{}.name),

  // The next three rows expect CORE's default, not the GUI struct's, and say where core states it.
  // On this path the struct that owns the field and the struct that owns the WIRE FORMAT are not
  // the same one, and for these three they disagree. The GUI-native table asserts the GUI default
  // for the same fields. That the two formats differ here is the fact being pinned — not a
  // discrepancy waiting to be reconciled.
  LUMICE_MISSING_KEY_ROW(kCoreFilter, s.filters.at(0).sym_p, false),
  LUMICE_MISSING_KEY_ROW(kCoreFilter, s.filters.at(0).sym_b, false),
  LUMICE_MISSING_KEY_ROW(kCoreFilter, s.filters.at(0).sym_d, false),
  LUMICE_MISSING_KEY_ROW(R"({"render":[{}]})", s.renderer.visible, kVisibleUpper),
  LUMICE_MISSING_KEY_ROW(
      R"({"crystal":[{"id":1,"type":"prism"}],"scene":{"scattering":[{"prob":0,"entries":[{"crystal":1}]}]}})",
      s.layers.at(0).entries.at(0).proportion, EntryCard{}.proportion),

  LUMICE_MISSING_KEY_ROW(kCoreFilter, s.filters.at(0).name, FilterConfig{}.name),
  LUMICE_MISSING_KEY_ROW(kCoreFilter, s.filters.at(0).action, FilterConfig{}.action),
  LUMICE_MISSING_KEY_ROW(kCoreComplex, s.filters.at(0).name, FilterConfig{}.name),
  LUMICE_MISSING_KEY_ROW(kCoreComplex, s.filters.at(0).action, FilterConfig{}.action),

  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).visible, ColorClassConfig{}.visible),
  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).solo, ColorClassConfig{}.solo),
  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).match.at(0).layer_idx, ColorClassRefConfig{}.layer_idx),
  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).match.at(0).sym_p, ColorClassRefConfig{}.sym_p),
  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).match.at(0).sym_b, ColorClassRefConfig{}.sym_b),
  LUMICE_MISSING_KEY_ROW(kCoreColor, s.raypath_color.at(0).match.at(0).sym_d, ColorClassRefConfig{}.sym_d),
};

#undef LUMICE_MISSING_KEY_ROW

TEST(DocumentDefaultsChain, GuiNativeAbsentKeyTakesTheOwningStructDefault) {
  DoNew();
  for (const MissingKeyCase& c : kGuiNativeCases) {
    SCOPED_TRACE(std::string(c.label) + "  <- " + c.doc);
    GuiState loaded;
    if (!DeserializeGuiStateJson(c.doc, loaded)) {
      ADD_FAILURE() << c.label << ": document failed to parse: " << c.doc;
      continue;  // nothing loaded for this row; the rest still get checked
    }
    c.check(loaded);
  }

  // The one filter key whose absence does NOT take the owning struct default, stated here because
  // this is the file a reader comes to with the question. A `filter` object naming no rule at all
  // leaves nothing for a default to be read off: the entry is loaded without a filter rather than
  // with a default-constructed one, because a default FilterConfig lowered into the scene would be
  // a term matching every ray, and under filter_out that hides all of them.
  constexpr const char* kEmptyFilter = R"({"layers":[{"entries":[{"crystal":{"type":"prism"},"filter":{}}]}]})";
  GuiState empty_filter;
  ASSERT_TRUE(DeserializeGuiStateJson(kEmptyFilter, empty_filter)) << kEmptyFilter;
  EXPECT_TRUE(empty_filter.filters.empty()) << "an empty filter object became a filter in the pool";
  EXPECT_FALSE(empty_filter.layers.at(0).entries.at(0).filter_id.has_value())
      << "the entry was given a filter the document did not describe";

  // Symmetric with kEmptyFilter above, but for the entry_exit legacy arm: a `type":"entry_exit"`
  // filter naming neither face nor length is the wildcard shape, and
  // FromLegacyEntryExit now answers it the same way FromLegacyRaypath answers an empty
  // raypath_text — no filter enters the pool for it.
  constexpr const char* kEmptyEeFilter =
      R"({"layers":[{"entries":[{"crystal":{"type":"prism"},"filter":{"type":"entry_exit"}}]}]})";
  GuiState empty_ee_filter;
  ASSERT_TRUE(DeserializeGuiStateJson(kEmptyEeFilter, empty_ee_filter)) << kEmptyEeFilter;
  EXPECT_TRUE(empty_ee_filter.filters.empty())
      << "an entry_exit filter naming neither face nor length became a filter in the pool";
  EXPECT_FALSE(empty_ee_filter.layers.at(0).entries.at(0).filter_id.has_value())
      << "the entry was given a filter the document did not describe";
}

TEST(DocumentDefaultsChain, CoreJsonAbsentKeyTakesTheOwningStructDefault) {
  DoNew();
  for (const MissingKeyCase& c : kCoreJsonCases) {
    SCOPED_TRACE(std::string(c.label) + "  <- " + c.doc);
    GuiState loaded;
    if (!DeserializeFromJson(c.doc, loaded)) {
      ADD_FAILURE() << c.label << ": document failed to parse: " << c.doc;
      continue;
    }
    c.check(loaded);
  }
}

// The tables above quantify over "absent key -> struct default". Three fields do NOT obey it, and
// each is recorded here as an executable statement of what it does instead. Two of them changed
// meaning once core stopped having an answer to mirror; the third is a documented historical
// fallback. They are asserted, not skipped, because "we know this one differs" is only worth
// anything if it stays differing in the specific way written down.

// A typeless axis slot: core now REJECTS such a document outright, so there is no core default left
// to mirror and the loader falls back to its own struct default. The GUI still opens the file — it
// is an editor, and a document you cannot open is a document you cannot fix — and it says what it
// substituted. All three parts are asserted: "loads", "loads at the right value" and "tells the
// user" are three separate ways this can go wrong and only the first is obvious.
TEST(DocumentDefaultsChain, TypelessAxisSlotLoadsAsTheStructDefaultAndIsAnnounced) {
  for (const char* slot : { "zenith", "azimuth", "roll" }) {
    SCOPED_TRACE(slot);
    DoNew();
    ClearImportComplexFilterWarning();
    GuiState loaded = InitDefaultState();

    // Crystal 7 carries the typeless slot; crystal 3 stays well-formed, so a warning naming
    // "crystal id=7" cannot be a fixed string that happens to read correctly.
    nlohmann::json axis;
    axis["zenith"] = { { "type", "gauss" }, { "mean", 90.0f }, { "std", 1.0f } };
    axis[slot] = { { "mean", 20.0f }, { "std", 5.0f } };
    nlohmann::json root;
    root["crystal"] = nlohmann::json::array(
        { { { "id", 3 }, { "type", "Prism" }, { "height", 1.0 }, { "face_distance", { 1, 1, 1, 1, 1, 1 } } },
          { { "id", 7 },
            { "type", "Prism" },
            { "height", 1.0 },
            { "face_distance", { 1, 1, 1, 1, 1, 1 } },
            { "axis", axis } } });
    root["filter"] = nlohmann::json::array();
    root["scene"]["light_source"] = { { "altitude", 20.0 }, { "diameter", 0.5 }, { "spectrum", "D65" } };
    root["scene"]["ray_num"] = 1000;
    root["scene"]["max_hits"] = 8;
    root["scene"]["scattering"] = nlohmann::json::array(
        { { { "prob", 0.5 },
            { "entries", nlohmann::json::array({ { { "crystal", 7 }, { "proportion", 100.0 } } }) } } });

    if (!DeserializeFromJson(root.dump(), loaded)) {
      ADD_FAILURE() << slot << ": GUI must load a document core would reject";
      continue;  // nothing loaded for this slot; the rest still get checked
    }
    if (loaded.crystals.empty()) {
      ADD_FAILURE() << slot << ": no crystals loaded";
      continue;
    }
    const AxisDist& parsed = (std::string(slot) == "zenith")  ? loaded.crystals[0].zenith :
                             (std::string(slot) == "azimuth") ? loaded.crystals[0].azimuth :
                                                                loaded.crystals[0].roll;
    EXPECT_EQ(parsed.type, AxisDist{}.type) << "a typeless slot must take this struct's own default";
    EXPECT_FLOAT_EQ(parsed.mean, 20.0f) << "the keys that WERE written must still be read";
    EXPECT_FLOAT_EQ(parsed.std, 5.0f);

    const std::string warning = PeekImportComplexFilterWarning();
    EXPECT_FALSE(warning.empty()) << "the substitution must be surfaced, not just performed";
    EXPECT_NE(warning.find("id=7"), std::string::npos) << "must name the crystal, got: " << warning;
    EXPECT_NE(warning.find(std::string("axis.") + slot), std::string::npos) << "must name the slot, got: " << warning;

    ClearImportComplexFilterWarning();
  }

  // The layer-probability half.
  DoNew();
  ClearImportComplexFilterWarning();
  GuiState layers = InitDefaultState();
  ASSERT_TRUE(DeserializeFromJson(R"({
    "crystal": [{"id": 1, "type": "Prism", "height": 1.0, "face_distance": [1,1,1,1,1,1]}],
    "filter": [],
    "scene": {
      "light_source": {"altitude": 20.0, "diameter": 0.5, "spectrum": "D65"}, "ray_num": 1000, "max_hits": 8,
      "scattering": [
        {"prob": 0.5, "entries": [{"crystal": 1, "proportion": 100.0}]},
        {"entries": [{"crystal": 1, "proportion": 100.0}]}
      ]
    }
  })",
                                  layers))
      << "GUI must load a legacy document core would reject";
  ASSERT_EQ(layers.layers.size(), 2u);
  EXPECT_FLOAT_EQ(layers.layers[0].probability, 0.5f) << "a present `prob` must be read, not defaulted";
  EXPECT_FLOAT_EQ(layers.layers[1].probability, 0.0f) << "absent `prob` must load as core's 0.0f, not 1.0f";
  const std::string layer_warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(layer_warning.empty()) << "the substitution must be surfaced, not just performed";
  EXPECT_NE(layer_warning.find("1"), std::string::npos) << "must name the offending layer, got: " << layer_warning;
  EXPECT_NE(layer_warning.find("prob"), std::string::npos) << "must name the field, got: " << layer_warning;
  ClearImportComplexFilterWarning();
}

// A scattering layer with no `prob`: same shape as the axis slot above, and folded into the same
// case for that reason — one substitution, one announcement, one clear, twice over. Layer 0 keeps
// its `prob` so a report of "layer 1" cannot be a hardcoded 0.
// file_io.cpp ParseCrystal: absent pyramid `upper_h` / `lower_h` fall back to 0.0f while
// CrystalConfig{} defaults both to 0.2f. Unlike the two above, this divergence IS written down at
// the call site, which says the two fields "deliberately do NOT" take CrystalConfig's default.
// Asserted as it behaves rather than left out, so that "is this field covered by the universal
// invariant?" has one answer for every known divergence. Changing this changes how already-written
// documents load, which is an owner decision, not a refactor.
TEST(DocumentDefaultsChain, PyramidHeightsKeepTheirHistoricalZeroFallback) {
  DoNew();
  GuiState loaded;
  ASSERT_TRUE(
      DeserializeGuiStateJson(R"({"layers":[{"entries":[{"crystal":{"type":"pyramid","shape":{}}}]}]})", loaded));
  ASSERT_EQ(loaded.crystals.size(), 1u);
  EXPECT_FLOAT_EQ(loaded.crystals[0].upper_h.center, 0.0f) << "documented historical fallback, not the struct default";
  EXPECT_FLOAT_EQ(loaded.crystals[0].lower_h.center, 0.0f);
  EXPECT_NE(CrystalConfig{}.upper_h.center, 0.0f) << "if the struct default became 0 too, this case is now vacuous "
                                                     "and the divergence it records has silently disappeared";
}

}  // namespace
}  // namespace lumice::gui
