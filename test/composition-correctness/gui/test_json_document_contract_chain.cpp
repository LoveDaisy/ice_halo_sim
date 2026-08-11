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
struct TempFile {
  std::filesystem::path path;
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path, ec);  // best-effort: a teardown failure must not fail the case
  }
};

TempFile WriteTempFile(const char* name, const std::string& text) {
  TempFile f{ std::filesystem::temp_directory_path() / name };
  std::ofstream out(f.path, std::ios::binary);
  out << text;
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
