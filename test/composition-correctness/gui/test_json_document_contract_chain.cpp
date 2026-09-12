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

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <system_error>

// Core's own lens decoder is the oracle for D-6c: the composition target links lumice_obj for
// exactly this kind of cross-check (test/CMakeLists.txt names the exemption).
#include "config/render_config.hpp"
#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "util/lens_fov_default.hpp"

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

// The Miller-index fallback is the one import-time downgrade in ParseCrystal that reported only to
// the log panel, while its twelve neighbours in the same function also queue an import notice. The
// four inputs below are the whole set the conversion owner refuses -- a wrong index count, a
// non-zero k, a negative index, and an h:l ratio no face can be built at -- and each of them leaves
// the wedge angle at a default that then reads as a value the document stated. Which is exactly the
// case a user hits by copying the GUI's own four-index label `{1, 0, -1, 1}` into the JSON.
//
// Table-driven with a non-fatal report per row, so one refused input failing does not hide the
// other three: an ASSERT here would stop at the first row and report a quarter of the truth.
TEST(JsonImportContractChain, AJsonMillerIndexRefusalReachesTheUserNotOnlyTheLog) {
  struct Row {
    const char* indices_json;
    const char* why;
  };
  const Row rows[] = {
    { "[1, 0, -1, 1]", "four indices: the GUI's own label notation, written straight into the JSON" },
    { "[1, 0]", "two indices: a triple left half-written" },
    { "[1, 1, 2]", "k != 0: a face this crystal model cannot express" },
    { "[1, 0, -1]", "a negative index" },
  };

  for (const Row& row : rows) {
    const std::string doc = DocWithCrystals(std::string(R"([
    {"id": 0, "type": "pyramid",
     "shape": {"prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.3, "upper_indices": )") +
                                            row.indices_json + R"(}}
  ])");

    ClearImportComplexFilterWarning();
    GuiState scratch;
    // Non-fatal + `continue` rather than ASSERT: a premise that fails on one row must not take the
    // other three rows' verdicts with it, which is what returning out of the function would do.
    if (!DeserializeFromJson(doc, scratch)) {
      ADD_FAILURE() << "premise: the document parses at all -- " << row.why;
      continue;
    }
    if (scratch.crystals.size() != 1u) {
      ADD_FAILURE() << "premise: the crystal was accepted -- " << row.why;
      continue;
    }

    EXPECT_FLOAT_EQ(scratch.crystals.at(0).upper_alpha, CrystalConfig{}.upper_alpha)
        << "a refused triple must leave the angle where it was, not half-apply -- " << row.why;

    const std::string warning = PeekImportComplexFilterWarning();
    EXPECT_FALSE(warning.empty()) << "the log panel is not where an import problem is looked for -- " << row.why;
    EXPECT_NE(warning.find("upper_indices"), std::string::npos)
        << "must name the field that was refused, got: " << warning;
    // Same two-decimal formatting the message itself uses (file_io.cpp's FormatAngleDegrees), so
    // this stays correct if the default wedge angle ever changes rather than pinning today's "28".
    std::ostringstream expected_angle;
    expected_angle << std::fixed << std::setprecision(2) << CrystalConfig{}.upper_alpha;
    EXPECT_NE(warning.find(expected_angle.str()), std::string::npos)
        << "must say which angle was kept, or the user cannot tell a refusal from a stated value, got: " << warning;
    ClearImportComplexFilterWarning();
  }
}

// The contrast that keeps the rule above from over-firing: `h == 0` is a legal document saying this
// side of the crystal has no pyramidal cap. It changes the angle to 0 and must stay silent, because
// a notice here would train the user to dismiss the notices that matter.
TEST(JsonImportContractChain, AJsonMillerIndexWithNoConeIsNotAnImportWarning) {
  const std::string doc = DocWithCrystals(R"([
    {"id": 0, "type": "pyramid",
     "shape": {"prism_h": 1.0, "upper_h": 0.3, "lower_h": 0.3, "upper_indices": [0, 0, 1]}}
  ])");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));
  ASSERT_EQ(scratch.crystals.size(), 1u) << "premise: the crystal was accepted";

  EXPECT_FLOAT_EQ(scratch.crystals.at(0).upper_alpha, 0.0f) << "h == 0 means no cone, which is an angle of 0";
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
      << "a legal document must import in silence, got: " << PeekImportComplexFilterWarning();
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
// `fov` beside it is optional to core as well, and gets the same warn-and-default treatment — see
// D-6b below: its absence is a document saying nothing, not a document being malformed, but the
// angle chosen for it is still one the author never wrote, so it is named rather than kept quiet.
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

// D-6b: a lens that states neither `fov` nor `f`. Core's LensParam::from_json loads such a document
// at a documented default (90 degrees; 30 for `globe`) and logs a WARNING naming the angle; this is
// the GUI half of that contract. The value is compared against the shared util/ function rather
// than a literal on purpose: the literal numbers are pinned in test_lens_fov_default.cpp, and what
// THIS chain has to prove is that the GUI reaches for the same authority core does — the same
// document must not render at 90 in the CLI and preview at something else in the GUI, on a key the
// author never wrote. Before this branch existed the GUI defaulted to a flat RenderConfig{}.fov,
// which is 90 for `globe` too; the globe row is what would have caught that.
TEST(JsonImportContractChain, AJsonRenderLensMissingFovAndFDefaultsAndWarns) {
  const std::string doc =
      DocWithParts(kWellFormedLightSource, R"([{"id": 1, "lens": {"type": "linear"}, "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.renderer.lens_type, kLensTypeLinear) << "premise: the lens object was read at all";
  EXPECT_FLOAT_EQ(scratch.renderer.fov, lumice::LensDefaultFovDegrees(false));

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "an angle was chosen for the user and never mentioned";
  EXPECT_NE(warning.find("fov"), std::string::npos) << "must name the field, got: " << warning;
  EXPECT_EQ(warning.find("no \"type\""), std::string::npos) << "the document DID state its type, got: " << warning;
  ClearImportComplexFilterWarning();
}

TEST(JsonImportContractChain, AJsonRenderLensMissingFovAndFOnGlobeDefaultsToThirty) {
  const std::string doc =
      DocWithParts(kWellFormedLightSource, R"([{"id": 1, "lens": {"type": "globe"}, "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.renderer.lens_type, kLensTypeGlobe) << "premise: the lens object was read at all";
  EXPECT_FLOAT_EQ(scratch.renderer.fov, lumice::LensDefaultFovDegrees(true));
  EXPECT_NE(scratch.renderer.fov, RenderConfig{}.fov)
      << "control: the globe default must differ from the GUI's own flat default, or this row proves nothing";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_NE(warning.find("fov"), std::string::npos) << "must name the field, got: " << warning;
  EXPECT_NE(warning.find("30"), std::string::npos) << "must name the angle it chose, got: " << warning;
  ClearImportComplexFilterWarning();
}

// Control for D-6b: a lens that does state `fov` is not told it was defaulted.
TEST(JsonImportContractChain, AJsonRenderLensWithStatedFovIsSilent) {
  const std::string doc = DocWithParts(kWellFormedLightSource, kWellFormedRender, "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_FLOAT_EQ(scratch.renderer.fov, 60.0f);
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty()) << PeekImportComplexFilterWarning();
}

// D-6c: a lens that states `f` (focal length) instead of `fov`. Core converts it per projection in
// LensParam::from_json; the GUI's import path used to have no `f` branch at all, so such a document
// fell into D-6b's "no fov" default and rendered at 90 in the preview while the CLI rendered it at
// whatever `f` meant. The oracle here is core's own decoder, not a literal and not the shared
// util/lens_focal.hpp function: both sides are driven through their real production entry points
// on the same `{type, f}` object, so a wrong formula-family mapping on EITHER side goes red, which
// "each side calls the util and the results happen to agree" could not show. The literal angles
// themselves are pinned in test_lens_focal.cpp.
//
// Five rows, one per formula family that has a solution (the dual fisheyes and `globe` share their
// sibling's family, and rectangular ignores `f` by convention — see the header for the collapse).
struct StatedFRow {
  const char* type;
  float f;
};

constexpr StatedFRow kStatedFRows[] = {
  { "linear", 24.0f },
  { "fisheye_equal_area", 12.0f },
  { "fisheye_equidistant", 12.0f },
  { "fisheye_stereographic", 6.0f },
  { "fisheye_orthographic", 24.0f },
};

TEST(JsonImportContractChain, AJsonRenderLensWithStatedFLoadsAtCoresFov) {
  for (const StatedFRow& row : kStatedFRows) {
    const nlohmann::json lens = { { "type", row.type }, { "f", row.f } };
    const nlohmann::json render =
        nlohmann::json::array({ { { "id", 1 }, { "lens", lens }, { "resolution", { 64, 64 } } } });
    const std::string doc = DocWithParts(kWellFormedLightSource, render.dump().c_str(), "");

    ClearImportComplexFilterWarning();
    GuiState scratch;
    EXPECT_TRUE(DeserializeFromJson(doc, scratch)) << row.type;

    const lumice::LensParam core_lens = lens.get<lumice::LensParam>();
    EXPECT_FLOAT_EQ(scratch.renderer.fov, core_lens.fov_) << row.type << " f=" << row.f;
    EXPECT_NE(scratch.renderer.fov, lumice::LensDefaultFovDegrees(false))
        << row.type << ": control — the row must not coincide with the no-fov default, or it proves nothing";
    EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
        << row.type << ": the author wrote how wide the lens is; got: " << PeekImportComplexFilterWarning();
  }
  ClearImportComplexFilterWarning();
}

// The GUI's own LensType → formula-family map is a second copy of core's (the enums are different
// types, so neither can be shared); this row exercises the members that share a family with one of
// the five above — the dual fisheyes and `globe` — so a map that pairs one of them with the wrong
// family is caught here and nowhere else.
constexpr StatedFRow kSharedFamilyRows[] = {
  { "globe", 12.0f },
  { "dual_fisheye_equal_area", 12.0f },
  { "dual_fisheye_equidistant", 12.0f },
  { "dual_fisheye_stereographic", 6.0f },
  { "dual_fisheye_orthographic", 24.0f },
};

TEST(JsonImportContractChain, AJsonRenderLensWithStatedFOnASharedFamilyMemberMatchesCore) {
  for (const StatedFRow& row : kSharedFamilyRows) {
    const nlohmann::json lens = { { "type", row.type }, { "f", row.f } };
    const nlohmann::json render =
        nlohmann::json::array({ { { "id", 1 }, { "lens", lens }, { "resolution", { 64, 64 } } } });
    const std::string doc = DocWithParts(kWellFormedLightSource, render.dump().c_str(), "");

    ClearImportComplexFilterWarning();
    GuiState scratch;
    EXPECT_TRUE(DeserializeFromJson(doc, scratch)) << row.type;

    const lumice::LensParam core_lens = lens.get<lumice::LensParam>();
    EXPECT_FLOAT_EQ(scratch.renderer.fov, core_lens.fov_) << row.type << " f=" << row.f;
  }
  ClearImportComplexFilterWarning();
}

// The domain edge: an `f` too short for its projection to reach the frame edge at all. Core rejects
// the document (test_json.cpp, LensConfigOrthographic.FCalcTooShortThrows); the GUI's contract is
// to keep what it can and say what it could not, so it warns and lands on the shared default —
// the same angle the no-`fov`-no-`f` branch takes, and one the author is told about.
TEST(JsonImportContractChain, AJsonRenderLensWithATooShortFWarnsAndDefaults) {
  const std::string doc =
      DocWithParts(kWellFormedLightSource,
                   R"([{"id": 1, "lens": {"type": "fisheye_equal_area", "f": 4}, "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.renderer.lens_type, kLensTypeFisheyeEqualArea) << "premise: the lens object was read at all";
  EXPECT_FLOAT_EQ(scratch.renderer.fov, lumice::LensDefaultFovDegrees(false));

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "an angle was chosen for the user and never mentioned";
  EXPECT_NE(warning.find("\"f\""), std::string::npos) << "must name the field it could not use, got: " << warning;
  EXPECT_NE(warning.find("90"), std::string::npos) << "must name the angle it chose, got: " << warning;
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
// A CLI-authored config's ev_mode must survive the import, and — because this is the only path
// where the GUI reads core's own wire format — must read the same two words core writes.
//
// The absent case is the one that matters most in practice and is asserted against a SEEDED
// non-default: every config written before this key existed comes through here, and "it loaded as
// relative" has to mean the reader chose relative, not that it never wrote the field at all.
TEST(JsonImportContractChain, AJsonRenderEvModeIsImportedAndAbsenceMeansRelative) {
  struct Row {
    const char* render_json;
    int expected;
    const char* label;
  };
  const Row kRows[] = {
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64], "ev_mode": "absolute"}])", 1,
      "absolute" },
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64], "ev_mode": "relative"}])", 0,
      "relative" },
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}])", 0, "<absent>" },
  };
  for (const Row& row : kRows) {
    const std::string doc = DocWithParts(kWellFormedLightSource, row.render_json, "");
    GuiState scratch;
    scratch.renderer.ev_mode = 1;  // seed non-default: an unread field must be visibly overwritten
    if (!DeserializeFromJson(doc, scratch)) {
      // Non-fatal per row: the absent case is the one that matters most and it is last, so a
      // fatal assert on either of the two before it would leave it unreported.
      ADD_FAILURE() << row.label << ": the import rejected the document outright";
      continue;
    }
    EXPECT_EQ(scratch.renderer.ev_mode, row.expected) << row.label;
  }
}

// The same three states for the print mode's two fields, imported from a CORE config document
// (not a .lmc). `paper` is the one worth the extra rows: its default is WHITE, so an importer that
// never read the key would satisfy any assertion that left a channel at 1.
TEST(JsonImportContractChain, AJsonRenderToneAndPaperAreImportedAndAbsenceMeansScreenAndWhite) {
  struct Row {
    const char* render_json;
    int expected_tone;
    float expected_paper0;
    const char* label;
  };
  const Row kRows[] = {
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64],
           "tone": "print", "paper": [0.9, 0.85, 0.8]}])",
      1, 0.9f, "print" },
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64],
           "tone": "screen", "paper": [0.7, 0.7, 0.7]}])",
      0, 0.7f, "screen" },
    { R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}])", 0, 1.0f, "<absent>" },
  };
  for (const Row& row : kRows) {
    const std::string doc = DocWithParts(kWellFormedLightSource, row.render_json, "");
    GuiState scratch;
    // Both seeded off what the row expects, so an unread field is visibly wrong rather than
    // accidentally right: tone to Print for the two rows expecting Screen, paper to a value no
    // row expects.
    scratch.renderer.tone = 1;
    scratch.renderer.paper[0] = 0.123f;
    if (!DeserializeFromJson(doc, scratch)) {
      // Non-fatal per row, for the reason its ev_mode twin above states: the absent case is last
      // and matters most.
      ADD_FAILURE() << row.label << ": the import rejected the document outright";
      continue;
    }
    EXPECT_EQ(scratch.renderer.tone, row.expected_tone) << row.label;
    EXPECT_NEAR(scratch.renderer.paper[0], row.expected_paper0, 1e-5f) << row.label;
  }
}

// The other half of AC2's "warn and fall back", on the one path where core's own warning cannot
// fire: DeserializeFromJson decodes the document itself and never calls core's parser, so a silent
// GUI here would mean the same malformed config reports differently depending on which side opened
// it. Both halves asserted — the value AND the notice — because either alone passes with the other
// missing.
TEST(JsonImportContractChain, AJsonRenderToneOfAnUnknownValueWarnsAndLoadsAsScreen) {
  const std::string doc = DocWithParts(
      kWellFormedLightSource,
      R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64], "tone": "glossy"}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  scratch.renderer.tone = 1;  // seed non-default so a reader that never wrote the field is visible
  ASSERT_TRUE(DeserializeFromJson(doc, scratch)) << "a malformed appearance value must not sink the document";

  EXPECT_EQ(scratch.renderer.tone, 0) << "an unrecognised tone must land on screen";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "a display mode was chosen for the user and never mentioned";
  EXPECT_NE(warning.find("tone"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// The control arm: a RECOGNISED tone must be silent. Without it, a decoder that warned on every
// import would satisfy the case above while making a correct config noisy.
TEST(JsonImportContractChain, AJsonRenderToneOfAKnownValueIsSilent) {
  const std::string doc = DocWithParts(
      kWellFormedLightSource,
      R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64], "tone": "print"}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));
  EXPECT_EQ(scratch.renderer.tone, 1);
  EXPECT_TRUE(PeekImportComplexFilterWarning().empty()) << PeekImportComplexFilterWarning();
  ClearImportComplexFilterWarning();
}

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
// The deliberate-capability-boundary half: fields core reads, the GUI has no place to put, and
// the GUI is not going to grow one.
//
// These are NOT the same proposition as D-4/D-6/D-7 above. Those are about a value the GUI CAN
// hold, arrived at by a guess the document never authorised — the fix there is to stop guessing in
// silence. Here the GUI has no field at all: `SunConfig` has no azimuth, `GuiState::RenderConfig`
// has no lens shift, `SimConfig` has no geometry clock. The owner ruling is that this asymmetry is
// intentional and stays (the sun is fixed at azimuth 0 in the GUI; the CLI keeps the freedom to
// write another value), so what is under test is not the value — the value is unreachable by
// construction — but whether the boundary says anything when a document crosses it.
//
// That distinction is why "does the loaded state differ" is the wrong oracle for these three and
// is deliberately not asserted: it cannot differ, in any build, ever. The only observable is the
// notice, and the only defect these cases can catch is its absence — which is exactly the state
// they were written against.
//
// The sibling half of the same boundary is the export arm, pinned in test_scene_commit_chain.cpp:
// what the GUI commits to core carries azimuth 0 and geom_clock 0 no matter what was imported.

// A complete document apart from an extra key inside `scene`, which is where geom_clock lives.
// DocWithParts' third parameter appends at ROOT level, one level too high for this.
namespace {
std::string DocWithSceneExtra(const char* scene_extra_json) {
  return std::string(R"({
    "crystal": [{"id": 0, "type": "prism", "shape": {"height": 2.0, "face_distance": [1, 1, 1, 1, 1, 1]}}],
    "filter": [],
    "scene": {"light_source": {"type": "sun", "altitude": 20, "spectrum": "D65"},
              "ray_num": 1000, "max_hits": 8)") +
         scene_extra_json + R"(,
              "scattering": [{"prob": 1.0, "entries": [{"crystal": 0, "proportion": 1.0}]}]},
    "render": [{"id": 1, "lens": {"type": "linear", "fov": 60}, "resolution": [64, 64]}]
  })";
}
}  // namespace

// B-1: a non-zero `scene.light_source.azimuth` rotates the whole sky in core, and the GUI drops it
// on the floor. Nothing downstream can recover it — the sun's azimuth is not a GuiState field —
// so the image the user is shown is the document's, turned by however many degrees they wrote.
TEST(JsonImportContractChain, AJsonSunAzimuthIsRefusedOutLoudNotDroppedInSilence) {
  const std::string doc =
      DocWithParts(R"({"type": "sun", "altitude": 20, "azimuth": 30, "spectrum": "D65"})", kWellFormedRender, "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_FLOAT_EQ(scratch.sun.altitude, 20.0f) << "premise: the light_source object was read at all";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "the sky was rotated back to azimuth 0 and the user was told nothing";
  EXPECT_NE(warning.find("azimuth"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// The other side of B-1, and the reason the warning is conditional rather than unconditional: a
// document that states azimuth 0, or states none at all, has asked for exactly what the GUI does.
// Warning there would train the user to dismiss the notice without reading it, which is the failure
// mode that makes every OTHER case in this file worthless.
TEST(JsonImportContractChain, AJsonSunAzimuthOfZeroIsNotWorthMentioning) {
  const char* const kRows[] = {
    R"({"type": "sun", "altitude": 20, "azimuth": 0, "spectrum": "D65"})",
    R"({"type": "sun", "altitude": 20, "spectrum": "D65"})",
  };
  for (const char* light_source : kRows) {
    const std::string doc = DocWithParts(light_source, kWellFormedRender, "");

    ClearImportComplexFilterWarning();
    GuiState scratch;
    if (!DeserializeFromJson(doc, scratch)) {
      // Non-fatal: a fatal assert here would hide the second row entirely.
      ADD_FAILURE() << "the import rejected the document outright: " << light_source;
      continue;
    }
    EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
        << "nothing was lost, so there is nothing to report: " << light_source << " -> "
        << PeekImportComplexFilterWarning();
    ClearImportComplexFilterWarning();
  }
  ClearImportComplexFilterWarning();
}

// scene.ray_allocation now has a home in the document (SimConfig::ray_allocation_adaptive), so a
// CLI-authored config carrying it is READ — the old behaviour was to drop it in silence, without
// even the WarnUnsupportedByDesign notice the fields with nowhere to go get. Three things pinned:
// both spellings land where core would put them; a recognised value is silent (the control arm
// without which "no warning" is unfalsifiable); and ABSENCE means the GUI DOCUMENT default
// (adaptive), which is deliberately not core's own default for an absent key (proportional): the
// moment a config is opened here it is a GUI document, and the GUI always writes the key back out,
// so core's absent-key default is never consulted on this road. The absent row is seeded off the
// expected value so a reader that never wrote the field is visibly wrong, not accidentally right.
TEST(JsonImportContractChain, AJsonSceneRayAllocationIsImportedAndAbsenceMeansTheGuiDefault) {
  struct Row {
    const char* spelled;  // nullptr = key absent
    bool expected_adaptive;
    const char* label;
  };
  const Row kRows[] = {
    { "adaptive", true, "adaptive" },
    { "proportional", false, "proportional" },
    { nullptr, SimConfig{}.ray_allocation_adaptive, "<absent>" },
  };
  ASSERT_TRUE(SimConfig{}.ray_allocation_adaptive)
      << "premise: the GUI document default is adaptive; if this moved, the <absent> row's meaning moved with it";
  for (const Row& row : kRows) {
    nlohmann::json doc = nlohmann::json::parse(DocWithParts(kWellFormedLightSource, kWellFormedRender, ""));
    if (row.spelled) {
      doc["scene"]["ray_allocation"] = row.spelled;
    }
    ClearImportComplexFilterWarning();
    GuiState scratch;
    scratch.sim.ray_allocation_adaptive = !row.expected_adaptive;  // seed off the expectation
    if (!DeserializeFromJson(doc.dump(), scratch)) {
      // Non-fatal per row: the absent row is last and matters most.
      ADD_FAILURE() << row.label << ": the import rejected the document outright";
      continue;
    }
    EXPECT_EQ(scratch.sim.ray_allocation_adaptive, row.expected_adaptive) << row.label;
    EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
        << row.label << ": the key has a home now, so nothing was lost and nothing is worth a notice; got: "
        << PeekImportComplexFilterWarning();
    ClearImportComplexFilterWarning();
  }
  ClearImportComplexFilterWarning();
}

// B-2: `render[].lens_shift` moves the optical axis off the image centre. The export arm has
// carried a comment since it was written saying it stays zero because no GUI control exists; the
// import arm said nothing at all, which is the same statement made where the user could not read
// it.
TEST(JsonImportContractChain, AJsonLensShiftIsRefusedOutLoudNotDroppedInSilence) {
  const std::string doc = DocWithParts(
      kWellFormedLightSource,
      R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "lens_shift": [8, -12], "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_FLOAT_EQ(scratch.renderer.fov, 60.0f) << "premise: the render object was read at all";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "the optical axis was recentred and the user was told nothing";
  EXPECT_NE(warning.find("lens_shift"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// A zero shift is what the GUI does anyway, on both components. Same reason as the azimuth twin.
TEST(JsonImportContractChain, AJsonLensShiftOfZeroIsNotWorthMentioning) {
  const std::string doc = DocWithParts(
      kWellFormedLightSource,
      R"([{"id": 1, "lens": {"type": "linear", "fov": 60}, "lens_shift": [0, 0], "resolution": [64, 64]}])", "");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
      << "nothing was lost, so there is nothing to report: " << PeekImportComplexFilterWarning();
  ClearImportComplexFilterWarning();
}

// B-3: `scene.geom_clock` selects core's GPU K-shape pool size. The GUI commits 0 (the pool
// disabled) unconditionally, so a document asking for a pool gets a different simulation than it
// wrote — and, until now, no indication of it.
TEST(JsonImportContractChain, AJsonGeomClockIsRefusedOutLoudNotDroppedInSilence) {
  const std::string doc = DocWithSceneExtra(R"(, "geom_clock": 64)");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_EQ(scratch.sim.max_hits, 8) << "premise: the scene object was read at all";

  const std::string warning = PeekImportComplexFilterWarning();
  EXPECT_FALSE(warning.empty()) << "the shape pool was disabled and the user was told nothing";
  EXPECT_NE(warning.find("geom_clock"), std::string::npos) << "must name the field, got: " << warning;
  ClearImportComplexFilterWarning();
}

// geom_clock 0 IS "pool disabled", which is what the GUI commits. Same reason as the two twins.
TEST(JsonImportContractChain, AJsonGeomClockOfZeroIsNotWorthMentioning) {
  const std::string doc = DocWithSceneExtra(R"(, "geom_clock": 0)");

  ClearImportComplexFilterWarning();
  GuiState scratch;
  ASSERT_TRUE(DeserializeFromJson(doc, scratch));

  EXPECT_TRUE(PeekImportComplexFilterWarning().empty())
      << "nothing was lost, so there is nothing to report: " << PeekImportComplexFilterWarning();
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

// Read back a file THE EXPORT WROTE, with line endings normalised to LF.
//
// `ExportConfigJson` opens its stream in text mode (`file_io.cpp`, since 3bf557fd 2026-03-19 —
// unlike SaveLmcFile, which is explicitly binary), so on Windows the runtime translates the `\n`
// this file hands it into `\r\n` and a byte compare against the literal fails there and only
// there. That translation is not what these two cases are about: their proposition is that the
// content the GUI was handed is the content that landed on disk, and the line ending is an
// incidental of the platform's text mode.
//
// ⛔ Deliberately NOT used by the "source file untouched" cases below. There the byte comparison
// IS the proposition, and it is a comparison of one file against its own earlier bytes — both
// sides written by this file in binary, so no translation is in play to paper over.
std::string ReadAllBytesLf(const std::filesystem::path& path) {
  std::string s = ReadAllBytes(path);
  s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
  return s;
}

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
  EXPECT_EQ(ReadAllBytesLf(target.path), kExportedContent);
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

  EXPECT_EQ(ReadAllBytesLf(target.path), kExportedContent);
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
