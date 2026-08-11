// Composition chain: what a core JSON document on disk is allowed to have done to it.
//
// Units in the chain: app (DoOpen / RequestConfigJsonExport) × file_io (DeserializeFromJson /
// ParseShapeDist / the downgrade counters / ConfigJsonExportNeedsOverwriteConfirm) × gui_state.
//
// The GUI's expressive power is strictly smaller than the core config JSON's, so an import
// degrades — and the owner ruling is that the degradation is confined to the copy in memory,
// with the export at the bottom of this file as the one deliberate way back out to disk:
//
//   (1) The source file is not rewritten. Nothing enforces that today except that no one has
//       written the line yet, which is not a guarantee; a later "normalize the config on load and
//       write it back" would destroy a document the user never asked to change, and every existing
//       test would still pass.
//   (2) The imported .json is not the save target. Save writes .lmc — the format that CAN carry
//       the whole GUI state — through the Save-As dialog, rather than overwriting the source with
//       a lossy re-emission.
//   (3) What WAS degraded reaches the user. The counter that records it is process-wide and
//       take-on-read, so this is two propositions, not one: the load must report its own
//       downgrades, and it must not report someone else's.
//
// Case (3) is why the drain discipline is pinned in both directions here. The .lmc branch has
// carried a pre-load drain + post-load take since it was written; the .json branch had neither,
// so its downgrades were invisible — and a post-load take added without the matching pre-load
// drain would have made it worse than silent, attributing MakeNewDocumentState's read of the
// user's personal defaults to whatever document happened to be opened next.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// A core JSON whose crystal height is a gauss distribution: a document the GUI cannot edit as
// written, and therefore one whose import degrades. The `%s` stand-in is the height value.
std::string DocWithHeight(const char* height_json) {
  return std::string(R"({
    "crystal": [{"id": 1, "type": "prism", "shape": {"height": )") +
         height_json + R"(, "face_distance": [1, 1, 1, 1, 1, 1]}}],
    "filter": [],
    "scene": {"light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
              "ray_num": 1000, "max_hits": 8,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]},
    "render": [{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}]
  })";
}

// Degrades on the way in (gauss → uniform); loads with nothing to report.
std::string GaussHeightDoc() {
  return DocWithHeight(R"({"type": "gauss", "mean": 2.0, "std": 0.3})");
}
std::string UniformHeightDoc() {
  return DocWithHeight(R"({"type": "uniform", "mean": 2.0, "std": 0.3})");
}

// A file that removes itself, so a failing assertion cannot leave the temp directory seeded for
// the next run.
//
// Ownership is spelled out rather than left to the optimiser. The first version declared only the
// destructor and returned a NAMED local from WriteTempFile below, which made correctness depend on
// NRVO — an optional elision, not a guarantee. Where the compiler took it (clang/gcc) the file
// survived; where it did not (MSVC) the source object was destroyed on return and deleted the
// fixture before the case could read it, so seven cases failed on Windows alone with
// `premise: the fixture was written`. Moving now transfers the duty and clears the source, so a
// moved-from object's destructor is a no-op no matter what the compiler elides.
struct TempFile {
  std::filesystem::path path;

  TempFile() = default;
  explicit TempFile(std::filesystem::path p) : path(std::move(p)) {}
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
  TempFile(TempFile&& other) noexcept : path(std::move(other.path)) { other.path.clear(); }
  TempFile& operator=(TempFile&& other) noexcept {
    if (this != &other) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
      path = std::move(other.path);
      other.path.clear();
    }
    return *this;
  }

  ~TempFile() {
    if (path.empty()) {
      return;  // moved-from: the duty went with the path
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the case
  }
};

TempFile WriteTempFile(const char* name, const std::string& text) {
  TempFile f{ std::filesystem::temp_directory_path() / name };
  {
    std::ofstream out(f.path, std::ios::binary);
    out << text;
  }  // closed before the handle leaves this scope, so the caller reads a flushed file
  return f;
}

// Bytes, not a parse: the proposition is that the file on disk is the same file, which a
// re-parse-and-compare would happily miss for any rewrite that is semantically equivalent.
std::string ReadAllBytes(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The GUI keeps the imported crystal's height distribution here regardless of how the document
// spelled it; the type is what the downgrade rewrites.
const ShapeDist& ImportedHeight() {
  return g_state.crystals.at(0).height;
}

}  // namespace

// (1) Owner principle 2, as a falsifiable guard rather than a comment.
//
// The premise assertions are the load-bearing half: a byte comparison over an import that did not
// actually degrade proves nothing about the case the principle is about. So the case first shows
// the document really was rewritten in memory (gauss became uniform), and only then that the bytes
// behind it were left alone.
TEST(JsonImportContractChain, ImportingAJsonThatDegradesLeavesTheSourceFileByteIdentical) {
  const TempFile doc = WriteTempFile("lumice_json_import_contract_writeback.json", GaussHeightDoc());
  const std::string before = ReadAllBytes(doc.path);
  ASSERT_FALSE(before.empty()) << "premise: the fixture was written";

  ClearImportComplexFilterWarning();
  DoOpen(doc.path);

  ASSERT_FALSE(g_state.crystals.empty()) << "premise: the document imported at all";
  ASSERT_EQ(ImportedHeight().type, ShapeDistType::kUniform)
      << "premise: this document degrades on import — without that the guard below covers nothing";
  EXPECT_FLOAT_EQ(ImportedHeight().center, 2.0f) << "the degradation keeps the value, only the family changes";

  EXPECT_EQ(ReadAllBytes(doc.path), before)
      << "the import rewrote its own source file: a degradation is allowed to change the copy in "
         "memory, never the document the user opened";
}

// (2) The imported .json is not the save target (app.cpp: current_file_path.clear() on the import
// branch). The .lmc control beside it is what stops "clear the path on every open" from passing —
// that would satisfy the .json half while breaking Save for the format that CAN hold the state.
//
// Only the recorded target is asserted, not the dialog that follows from it: PerformSave()'s
// empty-path branch calls ShowSaveDialog(), a native NFD dialog that blocks and cannot be driven
// from a windowless test. That branch therefore has no automated regression cover — a known,
// deliberate gap recorded here rather than papered over, since the two halves of this proposition
// are pinned unevenly.
TEST(JsonImportContractChain, AnImportedJsonIsNotTheSaveTarget) {
  const TempFile json_doc = WriteTempFile("lumice_json_import_contract_target.json", UniformHeightDoc());
  ClearImportComplexFilterWarning();
  DoOpen(json_doc.path);
  EXPECT_TRUE(g_state.current_file_path.empty())
      << "an imported .json became the save target, so Save would overwrite it with a lossy "
         "re-emission instead of writing a .lmc";

  const TempFile lmc_doc{ std::filesystem::temp_directory_path() / "lumice_json_import_contract_target.lmc" };
  ASSERT_TRUE(SaveLmcFile(lmc_doc.path, g_state, g_preview, /*save_texture=*/false));
  DoOpen(lmc_doc.path);
  EXPECT_EQ(g_state.current_file_path, lmc_doc.path) << "a .lmc open must still set the save target";
}

// (3a) The import reports its own downgrade. Before this was wired, ParseShapeDist counted the
// gauss→uniform rewrite and the .json branch simply never read the counter, so the user was told
// nothing at all.
TEST(JsonImportContractChain, ADowngradeDuringJsonImportReachesTheUser) {
  const TempFile doc = WriteTempFile("lumice_json_import_contract_notice.json", GaussHeightDoc());
  ClearImportComplexFilterWarning();
  DoOpen(doc.path);

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "the shape-distribution downgrade was performed but never surfaced";
  EXPECT_NE(warning.find("uniform"), std::string::npos) << "must name what happened, got: " << warning;
  ClearImportComplexFilterWarning();
}

// (3b) …and only its own. The counter is process-wide and take-on-read, so anything that ran the
// deserializer earlier — MakeNewDocumentState reading the user's personal defaults is the real
// instance, and is unreachable from here — leaves a count that a bare post-load take would hand to
// this document. Seeding it through a direct DeserializeFromJson reproduces exactly that state.
//
// This is the case that fails for an AC4 fix that adds the post-load take without the pre-load
// drain: the import below degrades nothing, and must therefore say nothing.
TEST(JsonImportContractChain, AJsonImportDoesNotInheritAnEarlierReadsDowngrade) {
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(GaussHeightDoc(), scratch)) << "premise: the seeding read succeeds";
  ASSERT_EQ(scratch.crystals.at(0).height.type, ShapeDistType::kUniform)
      << "premise: the seeding read left a downgrade count behind";

  const TempFile doc = WriteTempFile("lumice_json_import_contract_carryover.json", UniformHeightDoc());
  ClearImportComplexFilterWarning();
  DoOpen(doc.path);

  ASSERT_EQ(ImportedHeight().type, ShapeDistType::kUniform);
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
      << "this document degraded nothing; the notice describes an earlier read: " << PeekImportComplexFilterWarning();
  ClearImportComplexFilterWarning();
}

// ---------------------------------------------------------------------------------------------
// (4) The other half of the same import contract: a document core would REJECT outright.
//
// The three cases above are about a document core accepts and the GUI cannot hold as written — a
// capability downgrade. These are the opposite shape: a field core requires (`.at(key)`, which
// throws) that the document does not carry at all. The GUI used to read every one of them with
// `.value(key, default)`, which cannot tell "absent" from "present and equal to the default", so a
// malformed document opened as a silently invented one.
//
// The disposition is per-field, and it is not uniformly "drop it": where the missing field is a
// discriminant selecting how the REST of the unit is read, or where no value of its type is the
// neutral one, the unit it belongs to is dropped rather than guessed. Where the field lives in a
// singleton scope with no collection to drop from, the value stays as it was and only the silence
// ends. Either way the report goes out through the channel the import already had.
// ---------------------------------------------------------------------------------------------

namespace {

// A crystal object the caller shapes, wired into an otherwise complete document with a single
// scattering entry referencing crystal id 1. `%s` is the whole crystal object.
std::string DocWithCrystals(const std::string& crystals_json) {
  return std::string(R"({
    "crystal": )") +
         crystals_json + R"(,
    "filter": [],
    "scene": {"light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
              "ray_num": 1000, "max_hits": 8,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 0, "proportion": 1.0}]}]},
    "render": [{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}]
  })";
}

// A complete, well-formed document apart from the one part the caller replaces. Every other field
// the parse touches is stated, so a warning that appears can only be about the part under test —
// the notice channel appends, so a document malformed in two places would let a row pass on the
// wrong message.
std::string DocWithParts(const char* light_source_json, const char* render_json, const char* extra_root_json) {
  return std::string(R"({
    "crystal": [{"id": 0, "type": "prism", "shape": {"height": 2.0, "face_distance": [1, 1, 1, 1, 1, 1]}}],
    "filter": [],
    "scene": {"light_source": )") +
         light_source_json + R"(,
              "ray_num": 1000, "max_hits": 8,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 0, "proportion": 1.0}]}]},
    "render": )" +
         render_json + extra_root_json + R"(
  })";
}

constexpr const char* kWellFormedLightSource = R"({"type": "sun", "altitude": 20, "spectrum": "D65"})";
constexpr const char* kWellFormedRender =
    R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}])";

}  // namespace

// D-1, the case the ruling puts in its own severity band: `id` is the key of the map crystals are
// collected into, so an absent one is not a guessed value, it is a collision. Two crystals that both
// omit `id` both land on key 0 and the second silently destroys the first — an existing crystal the
// document DID state is gone, and a later Save-As freezes the loss.
//
// The two crystals are made distinguishable by height on purpose: the surviving slot's height is
// what says WHICH of the two won, and the assertion that it is neither of them is what says the
// question no longer has an answer because neither was accepted.
TEST(JsonImportContractChain, TwoCrystalsMissingIdDoNotSilentlyCollide) {
  const std::string doc = DocWithCrystals(R"([
    {"type": "prism", "shape": {"height": 2.0, "face_distance": [1, 1, 1, 1, 1, 1]}},
    {"type": "prism", "shape": {"height": 5.0, "face_distance": [1, 1, 1, 1, 1, 1]}}
  ])");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch)) << "the document still loads; only the two crystals are refused";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "two crystals collided on one map key and the user was told nothing";
  EXPECT_NE(warning.find("id"), std::string::npos) << "must name the field that was missing, got: " << warning;

  ASSERT_EQ(scratch.crystals.size(), 1u) << "the entry's fallback slot, and nothing else";
  const float center = scratch.crystals.at(0).height.center;
  EXPECT_FLOAT_EQ(center, CrystalConfig{}.height.center)
      << "the surviving crystal is one of the two the document wrote (height " << center
      << "), so one of them was silently overwritten by the other";
  ClearImportComplexFilterWarning();
}

// D-2: `type` is the discriminant — it decides whether `shape.*` is read as a prism's keys or a
// pyramid's. Reading it with a default picks one of the two geometries on the user's behalf, and
// "prism" is not the neutral answer, it is a specific crystal.
TEST(JsonImportContractChain, AJsonCrystalMissingTypeIsDroppedNotAssumedPrism) {
  const std::string doc = DocWithCrystals(R"([
    {"id": 0, "shape": {"height": 5.0, "face_distance": [1, 1, 1, 1, 1, 1]}}
  ])");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "a crystal with no `type` was assumed to be a prism, silently";
  EXPECT_NE(warning.find("type"), std::string::npos) << "must name the field that was missing, got: " << warning;

  ASSERT_EQ(scratch.crystals.size(), 1u);
  EXPECT_FLOAT_EQ(scratch.crystals.at(0).height.center, CrystalConfig{}.height.center)
      << "the refused crystal was loaded anyway, as a prism nobody asked for";
  ClearImportComplexFilterWarning();
}

// The same refusal, reached through the OTHER caller of the shared crystal parse: the GUI-native
// .lmc v2 form inlines its crystal in the entry and carries no `id`, so D-1 does not apply there —
// but D-2 does, and it arrives by inheriting the shared function's new return type rather than by a
// second edit. That inheritance is exactly the kind of path that gets assumed rather than checked,
// which is why it is pinned here: the refusal must land on the same default slot an entry with no
// inline crystal at all gets, not on a half-built one.
TEST(JsonImportContractChain, ALmcInlineCrystalMissingTypeFallsBackToADefaultSlot) {
  const std::string lmc = R"({
    "layers": [{"prob": 1.0, "entries": [
      {"crystal": {"shape": {"height": 5.0, "face_distance": [1, 1, 1, 1, 1, 1]}}, "proportion": 1.0}
    ]}]
  })";

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeGuiStateJson(lmc, scratch));

  ASSERT_EQ(scratch.layers.size(), 1u);
  ASSERT_EQ(scratch.layers.at(0).entries.size(), 1u);
  ASSERT_EQ(scratch.crystals.size(), 1u) << "the entry still needs a valid pool slot to point at";
  EXPECT_EQ(scratch.layers.at(0).entries.at(0).crystal_id, 0);
  EXPECT_FLOAT_EQ(scratch.crystals.at(0).height.center, CrystalConfig{}.height.center)
      << "the inline crystal was accepted as a prism instead of refused";
  EXPECT_NE(PeekImportComplexFilterWarning().find("type"), std::string::npos)
      << "got: " << PeekImportComplexFilterWarning();
  ClearImportComplexFilterWarning();
}

// D-5: a shape scalar written as a distribution object with no `type`. Core requires it there
// (Distribution::from_json), and the axis half of that same core rule is already reported by
// ParseAxisDist — the shape half sat silent in the same file, which is the shape of "fixed one of
// the two places" this case exists to close.
//
// The disposition here is NOT the refusal the crystal keys above get: `type` on a distribution is a
// field of ShapeDist with a documented default, and the ruling puts this one on the same footing as
// its axis twin — load at that default, and say so. What changes is only the silence.
TEST(JsonImportContractChain, AJsonShapeDistMissingTypeLoadsAtNoRandomAndWarns) {
  const std::string doc = DocWithCrystals(R"([
    {"id": 0, "type": "prism",
     "shape": {"height": 2.0, "face_distance": [{"mean": 2.0, "std": 0.1}, 1, 1, 1, 1, 1]}}
  ])");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));
  ASSERT_EQ(scratch.crystals.size(), 1u) << "premise: the crystal itself was accepted";

  const ShapeDist& fd0 = scratch.crystals.at(0).face_distance[0];
  EXPECT_EQ(fd0.type, ShapeDist{}.type) << "the value loads at the owning struct's default, unchanged by this fix";
  EXPECT_FLOAT_EQ(fd0.center, 2.0f) << "the `mean` the document did state must survive";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "the shape half of the rule stayed silent while the axis half reports";
  EXPECT_NE(warning.find("type"), std::string::npos) << "must name the field that was missing, got: " << warning;
  EXPECT_NE(warning.find("face_distance"), std::string::npos)
      << "must name WHICH slot was rewritten, the way the axis twin does, got: " << warning;
  ClearImportComplexFilterWarning();
}

// The one shape scalar deliberately left out of the rule above, and the contrast is the point: on
// the core side `prism_h` is still a hard requirement whose disposition is being settled there, so
// changing what the GUI makes of a typeless `prism_h` object now would move a value core has not
// finished defining. It keeps today's answer, silently, until that lands.
//
// Asserted as a negative rather than trusted: the exception lives in one boolean argument at one
// call site, which is exactly the kind of thing that gets flipped by a later tidy-up with nothing
// failing.
TEST(JsonImportContractChain, AJsonPrismHMissingTypeStaysSilentUnlikeOtherShapeDists) {
  const std::string doc = DocWithCrystals(R"([
    {"id": 0, "type": "pyramid",
     "shape": {"prism_h": {"mean": 2.0, "std": 0.1}, "face_distance": [1, 1, 1, 1, 1, 1]}}
  ])");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));
  ASSERT_EQ(scratch.crystals.size(), 1u) << "premise: the crystal itself was accepted";

  const ShapeDist& prism_h = scratch.crystals.at(0).prism_h;
  EXPECT_EQ(prism_h.type, ShapeDist{}.type) << "the value it loads at is unchanged from before this fix";
  EXPECT_FLOAT_EQ(prism_h.center, 2.0f);

  EXPECT_EQ(PeekImportComplexFilterWarning().find("prism_h"), std::string::npos)
      << "prism_h is held out of the rule until core settles it; got: " << PeekImportComplexFilterWarning();
  ClearImportComplexFilterWarning();
}

// D-4: `spectrum` is the key that decides how the rest of that object reads — a string names one of
// the built-in spectra, an array is a discrete custom one. Absent, there is nothing to discriminate
// on, and D65 is not the neutral answer to "which spectrum": it is a specific one, picked for the
// user out of a document that declined to say.
//
// `light_source` is a singleton, so there is no collection to drop this from and no downstream
// fallback to hand it to; what the ruling's "refuse" buys here is therefore the report, not a
// different value. The loaded spectrum stays exactly what it is today — the silence is what ends.
TEST(JsonImportContractChain, AJsonLightSourceMissingSpectrumWarnsAndKeepsDefault) {
  const std::string doc = DocWithParts(R"({"type": "sun", "altitude": 20})", kWellFormedRender, "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.sun.spectrum_index, SunConfig{}.spectrum_index) << "the value it loads at is unchanged";
  EXPECT_FLOAT_EQ(scratch.sun.altitude, 20.0f) << "premise: the light_source object was read at all";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "a spectrum was chosen for the user and never mentioned";
  EXPECT_NE(warning.find("spectrum"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// D-6: same shape as D-4 one level over — `lens.type` selects the whole projection branch the
// preview inverts, and `linear` is one specific projection rather than an absence of one. Also a
// singleton (the GUI keeps a single renderer), so again the value stays and the silence goes.
//
// `fov` beside it deliberately gets no such treatment: core itself reads that one as optional, so
// its absence is a document saying nothing, not a document being malformed.
TEST(JsonImportContractChain, AJsonRenderLensMissingTypeWarnsAndKeepsLinear) {
  const std::string doc =
      DocWithParts(kWellFormedLightSource, R"([{"id": 1, "lens": {"fov": 60}, "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.renderer.lens_type, RenderConfig{}.lens_type) << "the value it loads at is unchanged";
  EXPECT_FLOAT_EQ(scratch.renderer.fov, 60.0f) << "premise: the lens object was read at all";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "a projection was chosen for the user and never mentioned";
  EXPECT_NE(warning.find("lens"), std::string::npos) << "must name the object, got: " << warning;
  ClearImportComplexFilterWarning();
}

// D-7: a colour class with no colour. No colour is the neutral one — black and white both read as
// deliberate choices in a composite, and a class whose colour is undefined has no rendering meaning
// at all — so the class goes rather than getting one assigned. The file already drops whole filters
// and whole colour refs it cannot express; this is that same move one level out.
//
// The z_order assertion is the second half, and it is not incidental: z_order must be a compact
// permutation of [0, size) over the vector that actually holds the classes, so assigning it from the
// SOURCE array index — correct only while nothing is ever dropped — would punch a hole in that
// invariant the moment dropping became possible. Hence the surviving class is the second one, and
// its expected z_order is 0 rather than merely "non-negative".
TEST(JsonImportContractChain, AJsonColorClassMissingColorIsDroppedNotDefaultColored) {
  const std::string doc = DocWithParts(kWellFormedLightSource, kWellFormedRender, R"(,
    "raypath_color": {"mode": "painter", "classes": [
      {"match": [{"crystal": 0, "layer": 0}]},
      {"color": [0.25, 0.5, 0.75], "match": [{"crystal": 0, "layer": 0}]}
    ]})");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  ASSERT_EQ(scratch.raypath_color.size(), 1u) << "the colourless class was kept and given a colour nobody chose";
  EXPECT_FLOAT_EQ(scratch.raypath_color.at(0).color[0], 0.25f) << "the survivor is the class that HAD a colour";
  EXPECT_EQ(scratch.raypath_color.at(0).z_order, 0)
      << "z_order came from the source array index, so dropping a class left a hole in a range that "
         "must be a compact permutation of [0, size)";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "a colour class was dropped and the user was told nothing";
  EXPECT_NE(warning.find("color"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// ---------------------------------------------------------------------------------------------
// The export half of the same contract: what the GUI is allowed to write over.
//
// Import degrades in memory; export is where that degraded copy can reach the disk. The four cases
// below pin the decision RequestConfigJsonExport makes — write, or ask first — and both answers to
// the question it raises. They stop at the state and the bytes: that a modal is actually rendered
// for the pending flag is a proposition about the frame loop, and lives in gui_test
// (file_ops/exporting_over_an_existing_config_asks_before_overwriting).
// ---------------------------------------------------------------------------------------------

namespace {

// Distinguishable from anything the export could produce, so "unchanged" is checkable by content.
constexpr const char* kExistingContent = "{\"not\": \"written by the gui\"}\n";
constexpr const char* kExportedContent = "{\"exported\": true}\n";

}  // namespace

// Nothing at the target: the export is not a destructive act, and asking would be noise.
TEST(ConfigJsonExportContractChain, ExportingToAFreshPathWritesWithoutAsking) {
  const TempFile target{ std::filesystem::temp_directory_path() / "lumice_export_contract_fresh.json" };
  std::error_code ec;
  std::filesystem::remove(target.path, ec);
  CancelPendingConfigJsonExport();
  g_show_export_overwrite_confirm_popup = false;

  RequestConfigJsonExport(target.path, kExportedContent);

  EXPECT_FALSE(g_show_export_overwrite_confirm_popup) << "nothing was there to lose; the prompt is noise";
  EXPECT_EQ(ReadAllBytes(target.path), kExportedContent);
}

// Something at the target: hold everything, ask, and — the part that matters — leave the file alone
// while the question is open.
TEST(ConfigJsonExportContractChain, ExportingOverAnExistingFileAsksBeforeWriting) {
  const TempFile target = WriteTempFile("lumice_export_contract_existing.json", kExistingContent);
  CancelPendingConfigJsonExport();
  g_show_export_overwrite_confirm_popup = false;

  RequestConfigJsonExport(target.path, kExportedContent);

  EXPECT_TRUE(g_show_export_overwrite_confirm_popup) << "the overwrite happened without asking";
  EXPECT_EQ(g_pending_export_json_path, target.path);
  EXPECT_EQ(ReadAllBytes(target.path), kExistingContent) << "the file was written before the user answered";

  CancelPendingConfigJsonExport();
  g_show_export_overwrite_confirm_popup = false;
}

// Answering yes writes what was held, not what the state happens to say now.
TEST(ConfigJsonExportContractChain, ConfirmingWritesTheHeldDocumentAndClearsThePending) {
  const TempFile target = WriteTempFile("lumice_export_contract_confirm.json", kExistingContent);
  CancelPendingConfigJsonExport();
  g_show_export_overwrite_confirm_popup = false;
  RequestConfigJsonExport(target.path, kExportedContent);
  ASSERT_TRUE(g_show_export_overwrite_confirm_popup) << "premise: the prompt was raised";

  ConfirmPendingConfigJsonExport();

  EXPECT_EQ(ReadAllBytes(target.path), kExportedContent);
  EXPECT_TRUE(g_pending_export_json_path.empty()) << "a resolved export must not stay pending";
  EXPECT_TRUE(g_pending_export_json_content.empty());
}

// What the prompt says is the point of asking. "A file already exists — overwrite?" is a question
// about a filename, and answering it yes is not the acknowledgement the ruling asks for: the user
// has to be told that what goes back is only the part of that document the GUI can express. Pinned
// because wording is exactly the kind of thing a later tidy-up shortens into a generic prompt, with
// nothing failing.
TEST(ConfigJsonExportContractChain, TheOverwritePromptSaysWhatIsLost) {
  const std::string text = kExportOverwriteWarningText;
  EXPECT_NE(text.find("lost"), std::string::npos) << "must say something is lost, got: " << text;
  EXPECT_NE(text.find("cannot represent"), std::string::npos)
      << "must say WHAT is lost — what the GUI cannot represent — not merely that a file changes: " << text;
  EXPECT_NE(text.find(".lmc"), std::string::npos) << "must name the lossless alternative, got: " << text;
}

// Answering no leaves the document exactly as it was — the whole reason the prompt exists.
TEST(ConfigJsonExportContractChain, CancellingLeavesTheExistingFileUntouched) {
  const TempFile target = WriteTempFile("lumice_export_contract_cancel.json", kExistingContent);
  CancelPendingConfigJsonExport();
  g_show_export_overwrite_confirm_popup = false;
  RequestConfigJsonExport(target.path, kExportedContent);
  ASSERT_TRUE(g_show_export_overwrite_confirm_popup) << "premise: the prompt was raised";
  g_show_export_overwrite_confirm_popup = false;  // the render call would consume it

  CancelPendingConfigJsonExport();

  EXPECT_EQ(ReadAllBytes(target.path), kExistingContent);
  EXPECT_TRUE(g_pending_export_json_path.empty());
  EXPECT_TRUE(g_pending_export_json_content.empty());

  // A cancelled export must stay cancelled: confirming afterwards has nothing held and must not
  // fall back on some other path (the empty one, or the last one written).
  ConfirmPendingConfigJsonExport();
  EXPECT_EQ(ReadAllBytes(target.path), kExistingContent);
}

}  // namespace lumice::gui
