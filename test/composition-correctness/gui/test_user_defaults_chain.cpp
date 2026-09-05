// Composition chain: personal defaults — diff, adopt, read back.
//
// Units in the chain: defaults_panel's two suppliers, defaults_diff × user_defaults, over gui_state.
//
// What the collaboration produces that is observable: the settings panel's rows, and whether
// pressing Adopt leaves the next launch actually starting from the adopted value. Both halves are
// needed for the feature to be true, and each fails silently on its own — an adoption that does not
// reach disk looks identical to one that does until the user restarts, and a row whose displayed
// value disagrees with the stored one by a rounding digit can never be cleared no matter how many
// times it is adopted.
//
// Derived from the src call graph: defaults_panel.cpp -> user_defaults is 12 call sites and ->
// defaults_diff is 8, the two heaviest edges out of that panel.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/user_defaults.hpp"
#include "support/user_defaults_test_env.hpp"

namespace lumice::gui {
namespace {

using lumice::test_user_defaults::FreshOverlayDir;
using lumice::test_user_defaults::ResetUserDefaultsChannels;

const DefaultDiffRow* FindRow(const std::vector<DefaultDiffRow>& rows, const std::string& key_path) {
  for (const DefaultDiffRow& row : rows) {
    if (row.key_path == key_path) {
      return &row;
    }
  }
  return nullptr;
}

// A key that exists in every serialized document and is a plain scalar, so a row for it is always
// available to probe with.
constexpr const char* kProbeKey = "bg_alpha";

// The process-wide baseline this binary starts from is kDisabled (installed by the gtest global
// environment it shares with gui_unit_test), so a write-side case has nowhere to write until it
// points the process at a directory of its own. Draining the consumable channels on both ends is
// what stops one case's override state from being read as the next one's.
class UserDefaultsChain : public ::testing::Test {
 protected:
  void SetUp() override { ResetUserDefaultsChannels(); }
  void TearDown() override { ResetUserDefaultsChannels(); }

  // Install a fresh, empty override directory for the rest of this case.
  const std::filesystem::path& UseFreshConfigDir(const char* tag) {
    dir_ = FreshOverlayDir(tag);
    guard_.emplace(UserConfigSource::kExplicitDir, dir_);
    return dir_;
  }

 private:
  std::filesystem::path dir_;
  std::optional<lumice::test_user_defaults::ScopedUserConfigSource> guard_;
};

// ---------------------------------------------------------------------------------------------
// E20 (factory document) — what a new document IS, before any personal default has been layered on.
//
// MakeNewDocumentState is the one function main.cpp, DoNew() and DoOpen()'s import path all call to
// produce a fresh document, so it — not whatever global the result was last stored in — is what
// "the default state" means. Every case below describes a DIFFERENCE from this document; if the
// document itself came out malformed, those differences would be measured against nothing.
//
// The override directory is supplied explicitly and freshly emptied, so this describes the factory
// document rather than whatever personal defaults exist on the machine running the test.
TEST_F(UserDefaultsChain, ANewDocumentIsAWholeDocumentBeforeAnyOverrideIsLayeredOn) {
  const GuiState state = MakeNewDocumentState(UseFreshConfigDir("factory_document"));

  // ASSERT before indexing: a document with no layer is not a smaller document, it is one the
  // editor cannot open, and every EXPECT after this line would be reporting on that instead.
  ASSERT_EQ(state.layers.size(), 1u) << "a new document has exactly one scattering layer";
  EXPECT_EQ(state.layers[0].entries.size(), 1u) << "that layer has exactly one entry card";
  EXPECT_FALSE(state.crystals.empty()) << "the entry card references a crystal that must exist";
  EXPECT_FALSE(state.dirty) << "a document nobody has edited is not modified";
  EXPECT_EQ(state.sim_state, GuiState::SimState::kIdle) << "nothing has been run yet";
  EXPECT_TRUE(state.raypath_color.empty()) << "a new document carries no colour classes";
}

// ---------------------------------------------------------------------------------------------
// E20 — the closed loop: a row that needs adoption, adopted, reads back as what was adopted.
//
// Each step is separately checkable and each has its own way of being wrong, so they are separate
// assertions in one case rather than one end-to-end boolean: a green end-to-end check tells you the
// loop closed, not which link would have failed. The three claims below all describe what ONE
// adoption does to ONE row, and they share the adoption to say so — split across three cases they
// were three copies of the same six-line staging with a different closing assertion.
TEST_F(UserDefaultsChain, AdoptingARowMakesTheDocumentReadBackTheAdoptedValue) {
  UseFreshConfigDir("adopt");

  GuiState current;
  current.bg_alpha = 0.375f;  // deliberately not the factory value

  nlohmann::json doc = nlohmann::json::object();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);

  const DefaultDiffRow* row = FindRow(rows, kProbeKey);
  ASSERT_NE(row, nullptr) << "the walk did not produce a row for " << kProbeKey;
  EXPECT_TRUE(RowNeedsAdoption(*row)) << "a value differing from the effective default is exactly what needs adopting";
  EXPECT_TRUE(RowWouldChangeOnSave(*row, /*checked=*/true));
  EXPECT_FALSE(RowWouldChangeOnSave(*row, /*checked=*/false)) << "an unchecked row must be a no-op";
  EXPECT_FALSE(row->has_saved_override);
  EXPECT_EQ(row->default_value, row->factory_value) << "with nothing saved the two are the same value";

  ASSERT_TRUE(ApplyCheckedRowsToDoc(doc, rows, { kProbeKey }, current));

  // Read back through the same document shape the panel writes, not through a hand-built key path:
  // this is the step where "what the panel wrote" and "what the loader looks for" would drift.
  const std::vector<DefaultDiffRow> after = BuildDefaultDiffRows(current, doc);
  const DefaultDiffRow* adopted = FindRow(after, kProbeKey);
  ASSERT_NE(adopted, nullptr);
  EXPECT_TRUE(adopted->has_saved_override) << "the key was adopted but the document does not record it";
  EXPECT_EQ(adopted->default_value, adopted->current_value)
      << "after adoption the effective default must BE the adopted value; otherwise the row keeps "
         "asking to be adopted forever";
  EXPECT_FALSE(RowNeedsAdoption(*adopted));
  // The distinction the header calls out as having been confused before: `default_value` is the
  // EFFECTIVE default (factory with any saved override on top) and `factory_value` is the literal
  // factory one. Once a key has been adopted they disagree, and asking "does this differ from
  // factory" of default_value answers "no" for precisely the keys the question is about.
  EXPECT_NE(adopted->default_value, adopted->factory_value)
      << "the effective default absorbed the adoption but factory_value moved with it, so the "
         "'differs from factory' filter can no longer see any customised key";
  EXPECT_EQ(adopted->factory_value, nlohmann::json(GuiState{}.bg_alpha));
}

// Presence and value are independent, and the panel needs both. A user may deliberately save a
// value that equals the factory one — that is still their default, and it has to remain visible and
// revertible. A has_saved_override derived from "differs from factory" would erase it.
TEST_F(UserDefaultsChain, SavingAValueEqualToFactoryIsStillASavedOverride) {
  UseFreshConfigDir("equal_to_factory");

  GuiState current;  // untouched: current == factory for every key
  nlohmann::json doc = nlohmann::json::object();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);
  ASSERT_NE(FindRow(rows, kProbeKey), nullptr);
  ASSERT_TRUE(ApplyCheckedRowsToDoc(doc, rows, { kProbeKey }, current));
  ASSERT_TRUE(DocHasKeyPath(doc, kProbeKey)) << "the adoption did not reach the document at all";

  const DefaultDiffRow* saved = FindRow(BuildDefaultDiffRows(current, doc), kProbeKey);
  ASSERT_NE(saved, nullptr);
  EXPECT_TRUE(saved->has_saved_override)
      << "the key is in the override document, so the panel must keep showing it as saved even "
         "though its value equals factory";
}

// ---------------------------------------------------------------------------------------------
// E20 (precision half) — the displayed value and the stored value must agree to the digit.
//
// If the panel prints 0.50 for a stored 0.4999, the row reports a difference the user cannot
// resolve: adopting writes what is displayed, the loader reads what is stored, and the row comes
// back every single time. RoundTripPrecisionForAxisPresetStd exists so display and storage pick
// precision the same way, and FormatAxisPresetStd is the one renderer that uses it.
TEST_F(UserDefaultsChain, StdDisplayIsExactlyEnoughToReadBackTheSameFloatAndNoLonger) {
  const float kValues[] = {
    0.3f,  // the common case: must stay short
    0.5f,
    1.0f,
    9.0f,
    std::nextafter(10.0f, 0.0f),  // needs 7 digits — the clamp target for an upper bound
    std::nextafter(15.0f, 1e9f),  // needs 8
    std::nextafter(0.0f, 1.0f),
  };
  for (float v : kValues) {
    const std::string shown = FormatAxisPresetStd(v);
    EXPECT_EQ(std::strtof(shown.c_str(), nullptr), v)
        << "displayed as '" << shown
        << "', which does not read back as the same float — a row rendered this way can never be "
           "cleared by adopting it";
  }

  // The other half of the precision rule. Escalating unconditionally would render every tuned value
  // as a run of trailing digits that reads like precision it does not carry.
  EXPECT_EQ(FormatAxisPresetStd(0.3f), "0.3");
  EXPECT_EQ(FormatAxisPresetStd(1.0f), "1");
}

// ---------------------------------------------------------------------------------------------
// E20 (preset-library half) — write, read, erase over the override document.
//
// The three document operations have to compose: what the writer stores is what the reader reports,
// and an erase leaves no skeleton behind for the next person who opens the file by hand.
TEST_F(UserDefaultsChain, PresetOverrideDocumentWriteReadEraseCompose) {
  nlohmann::json doc = nlohmann::json::object();

  EXPECT_FALSE(ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kColumn).has_value())
      << "an empty document stores nothing";

  WriteAxisPresetZenithStdToDoc(doc, AxisPreset::kColumn, 0.75f);
  const std::optional<float> read_back = ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kColumn);
  ASSERT_TRUE(read_back.has_value());
  EXPECT_FLOAT_EQ(*read_back, 0.75f);

  // A second preset must survive the first one's erase — the operations are surgical edits of a
  // shared document, not whole-document rewrites.
  WriteAxisPresetZenithStdToDoc(doc, AxisPreset::kPlate, 0.25f);
  EraseAxisPresetZenithStdFromDoc(doc, AxisPreset::kColumn);

  EXPECT_FALSE(ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kColumn).has_value());
  const std::optional<float> other = ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kPlate);
  ASSERT_TRUE(other.has_value()) << "erasing one preset took the other with it";
  EXPECT_FLOAT_EQ(*other, 0.25f);

  EraseAxisPresetZenithStdFromDoc(doc, AxisPreset::kPlate);
  EXPECT_FALSE(doc.contains("presets")) << "the last erase left an empty skeleton behind: " << doc.dump();

  // A preset with no adjustable face has no key at all, so all three operations are no-ops for it,
  // and the clamp refuses out loud rather than storing something nothing ever reads.
  WriteAxisPresetZenithStdToDoc(doc, AxisPreset::kRandom, 0.5f);
  EXPECT_FALSE(ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kRandom).has_value());
  const AxisPresetClampResult refused = ClampAxisPresetZenithStdForSave(AxisPreset::kRandom, 0.5f);
  EXPECT_FALSE(refused.accepted);
  EXPECT_FALSE(refused.message.empty()) << "a refusal the user cannot read is a control that does nothing";
}

// The read is RAW on purpose: it reports what the document says, not what the classifier would
// accept. Folding the clamp in would collapse "what is stored" and "what is in effect", and a
// hand-edited file holding an out-of-domain value is exactly where the two differ.
TEST_F(UserDefaultsChain, DocumentReadIsRawAndClampingIsASeparateDecision) {
  nlohmann::json doc = nlohmann::json::object();
  const float kOutOfDomain = 1e6f;
  WriteAxisPresetZenithStdToDoc(doc, AxisPreset::kColumn, kOutOfDomain);

  const std::optional<float> stored = ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kColumn);
  ASSERT_TRUE(stored.has_value());
  EXPECT_FLOAT_EQ(*stored, kOutOfDomain) << "the reader silently normalised a value it was asked to report";

  const AxisPresetClampResult clamped = ClampAxisPresetZenithStdForSave(AxisPreset::kColumn, kOutOfDomain);
  EXPECT_TRUE(clamped.clamped) << "the clamp decision is where an out-of-domain value is caught";
  EXPECT_NE(clamped.stored_value, kOutOfDomain);
  EXPECT_FALSE(clamped.message.empty()) << "a clamp the user is not told about is a value that changed by itself";

  // The domain sentence beside the control is UI copy with a constraint on it: it must describe the
  // bound. A placeholder here leaves the user reading a number with no stated meaning.
  for (AxisPreset preset : { AxisPreset::kColumn, AxisPreset::kPlate, AxisPreset::kParry, AxisPreset::kLowitz }) {
    EXPECT_FALSE(DescribeAxisPresetZenithStdDomain(preset).empty())
        << "preset " << AxisPresetLabel(preset) << " has an adjustable std with no description of what it may be";
  }
}

// ---------------------------------------------------------------------------------------------
// E20 (in-memory adoption) — the accessor answering "is anything saved" is not the same question as
// "what distribution does this preset give".
TEST_F(UserDefaultsChain, OverridePresenceIsNotTheSameQuestionAsTheResultingDistribution) {
  UseFreshConfigDir("presence");

  EXPECT_FALSE(GetUserAxisPresetZenithStdOverride(AxisPreset::kColumn).has_value()) << "nothing has been adopted yet";

  // Adopt a value that is (very likely) also what the factory row carries. The composed
  // distribution is then indistinguishable from the un-overridden one — only the presence accessor
  // can still tell the panel there is something here to show and revert.
  const AxisPresetEntry& entry = AxisPresetEntryFor(AxisPreset::kColumn);
  const float factory_std = entry.zenith.std;
  AdoptAxisPresetZenithStdOverrideInMemory(AxisPreset::kColumn, factory_std);

  const std::optional<float> present = GetUserAxisPresetZenithStdOverride(AxisPreset::kColumn);
  ASSERT_TRUE(present.has_value()) << "an adopted value that equals factory is still an adopted value";
  EXPECT_FLOAT_EQ(*present, factory_std);

  AdoptAxisPresetZenithStdOverrideInMemory(AxisPreset::kColumn, std::nullopt);
  EXPECT_FALSE(GetUserAxisPresetZenithStdOverride(AxisPreset::kColumn).has_value())
      << "reverting to the factory value must remove the override, not store the factory number";
}

// ---------------------------------------------------------------------------------------------
// The override file's provenance stamp, across the write path the panel actually uses.
//
// The unit-level half of these claims (the stamp is applied, malformed ones are reported, a
// stamp-only document loads as nothing) lives in gui_unit_test against WriteUserDefaultsFile
// directly. What only shows up here is whether the two units still agree once the panel's route
// runs end to end — WriteActiveOverlayDoc resolving the directory, the store stamping the write,
// the diff engine walking what came back.

// The stamp has to arrive through the route the panel takes, not only when the store is called
// directly. defaults_panel commits through WriteActiveOverlayDoc; if the stamping had been put at
// a call site rather than in the store, THIS is the case that would notice, because it is the only
// one that never mentions WriteUserDefaultsFile.
TEST_F(UserDefaultsChain, TheSchemaStampArrivesThroughThePanelsOwnWritePath) {
  const std::filesystem::path& dir = UseFreshConfigDir("stamp_via_panel_path");

  ASSERT_TRUE(WriteActiveOverlayDoc(nlohmann::json{ { kProbeKey, 0.625f } }));

  std::ifstream in(dir / kUserDefaultsFileName);
  ASSERT_TRUE(in.is_open()) << "the write reported success but produced no file";
  const nlohmann::json on_disk = nlohmann::json::parse(in);
  ASSERT_TRUE(on_disk.contains(kUserDefaultsOverlaySchemaVersionKey))
      << "the panel's write path reached disk unstamped: " << on_disk.dump();
  EXPECT_EQ(on_disk[kUserDefaultsOverlaySchemaVersionKey].get<int>(), kUserDefaultsOverlaySchemaVersion);
  EXPECT_EQ(on_disk[kProbeKey].get<float>(), 0.625f) << "and it must still carry what was being saved";
}

// Reverting every personal default used to leave `{}` on disk and now leaves a file holding one
// key the user never set and cannot see. Everything downstream that answers "does this user have
// personal defaults?" has to keep answering no.
//
// Run as a round trip rather than as an assertion about IsOverlayDocEffectivelyEmpty because the
// regression this guards against is a disagreement BETWEEN units: the writer adding a key and a
// reader still testing plain emptiness. Each unit is self-consistent in that state; only the
// composition is wrong.
TEST_F(UserDefaultsChain, RevertingEveryDefaultLeavesOnlyTheStampAndStillReadsAsNoPersonalDefaults) {
  const std::filesystem::path& dir = UseFreshConfigDir("stamp_revert_all");

  GuiState current;
  current.bg_alpha = 0.375f;  // deliberately not the factory value, so the row asks to be adopted

  // Save one personal default the way the panel does, then confirm it really is on disk — the
  // revert below would otherwise be reverting nothing and the case would pass vacuously.
  nlohmann::json doc = ReadActiveOverlayDoc();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);
  ASSERT_NE(FindRow(rows, kProbeKey), nullptr);
  ASSERT_TRUE(ApplyCheckedRowsToDoc(doc, rows, { kProbeKey }, current));
  ASSERT_TRUE(WriteActiveOverlayDoc(doc));
  // ASSERT on the pointer before dereferencing it: a missing row here is a nullptr deref that
  // aborts this single-process binary and takes every case after it down too.
  const std::vector<DefaultDiffRow> saved_rows = BuildDefaultDiffRows(current, ReadActiveOverlayDoc());
  const DefaultDiffRow* saved = FindRow(saved_rows, kProbeKey);
  ASSERT_NE(saved, nullptr);
  ASSERT_TRUE(saved->has_saved_override)
      << "the precondition never landed, so this case would prove nothing about reverting it";

  // Now the "reset all my defaults" gesture: every row present, none checked.
  nlohmann::json reverted = ReadActiveOverlayDoc();
  const std::vector<DefaultDiffRow> all_rows = BuildDefaultDiffRows(current, reverted);
  ASSERT_TRUE(ApplyCheckedRowsToDoc(reverted, all_rows, {}, current));
  ASSERT_TRUE(WriteActiveOverlayDoc(reverted));

  std::ifstream in(dir / kUserDefaultsFileName);
  ASSERT_TRUE(in.is_open());
  const nlohmann::json on_disk = nlohmann::json::parse(in);
  ASSERT_TRUE(on_disk.is_object());
  EXPECT_EQ(on_disk.size(), static_cast<size_t>(1))
      << "after reverting everything the file must hold the stamp and nothing else: " << on_disk.dump();
  EXPECT_TRUE(on_disk.contains(kUserDefaultsOverlaySchemaVersionKey)) << on_disk.dump();

  // The claim that matters: that file means "no personal defaults", exactly as `{}` did.
  ResetUserDefaultsChannels();
  const GuiState after = MakeNewDocumentState(dir);
  EXPECT_EQ(SerializeGuiStateJson(after), SerializeGuiStateJson(MakeNewDocumentState(FreshOverlayDir("stamp_none"))))
      << "a stamp-only file produced a different document than an empty directory";
  EXPECT_EQ(TakeUserDefaultsDowngradeCount(), 0) << "and it is not a degradation — nothing about it is wrong";
  const std::vector<DefaultDiffRow> reverted_rows = BuildDefaultDiffRows(current, ReadActiveOverlayDoc());
  const DefaultDiffRow* reverted_row = FindRow(reverted_rows, kProbeKey);
  ASSERT_NE(reverted_row, nullptr);
  EXPECT_FALSE(reverted_row->has_saved_override)
      << "the panel still shows the reverted key as saved, so the user can never clear it";
}

// AC5 path 3 — saving a personal default goes through the same serializer the other two paths do,
// so every lens type has to reach disk under its own name and come back as itself.
//
// The round trip is the assertion, not the write: a lens type that lands on a neighbour's spelling
// writes a perfectly well-formed file and reads back as a DIFFERENT lens, which is the silent half
// of this defect — the loud half (the writer walking off the end of its name table) only shows up
// on builds where that read happens to fault.
TEST_F(UserDefaultsChain, EveryLensTypeSurvivesTheUserDefaultsRoundTrip) {
  constexpr const char* kLensKey = "renderer.lens_type";
  // Core's wire vocabulary, as literals. The round trip alone would NOT catch a wrong entry in the
  // writer's table: write and read walk the same table, so a misspelled name is written, read back,
  // and lands on its own index again — perfectly self-consistent and perfectly wrong for anyone
  // else reading the file. Measured, not assumed: with entry 8 deliberately misspelled, the round
  // trip half of this case stayed green while the two literal-spelling cases went red. The
  // on-disk assertion below is what makes this path see it.
  constexpr const char* kSpellings[] = {
    "linear",
    "fisheye_equal_area",
    "fisheye_equidistant",
    "fisheye_stereographic",
    "dual_fisheye_equal_area",
    "dual_fisheye_equidistant",
    "dual_fisheye_stereographic",
    "rectangular",
    "fisheye_orthographic",
    "dual_fisheye_orthographic",
    "globe",
  };
  static_assert(sizeof(kSpellings) / sizeof(kSpellings[0]) == kLensTypeCount,
                "this case must probe every lens type, not a prefix of them");

  for (int value = 0; value < kLensTypeCount; ++value) {
    const std::filesystem::path& dir = UseFreshConfigDir(("lens_type_" + std::to_string(value)).c_str());

    GuiState current;
    current.renderer.lens_type = value;

    nlohmann::json doc = ReadActiveOverlayDoc();
    const std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);
    // Non-fatal per value: a failure on one lens type must not take the other ten readings with it,
    // and "which values fail" is the diagnostic that separates a short table from a wrong entry.
    if (FindRow(rows, kLensKey) == nullptr) {
      ADD_FAILURE() << "value " << value << ": the panel produced no " << kLensKey << " row to adopt";
      continue;
    }
    if (!ApplyCheckedRowsToDoc(doc, rows, { kLensKey }, current) || !WriteActiveOverlayDoc(doc)) {
      ADD_FAILURE() << "value " << value << ": adopting the lens type never reached disk";
      continue;
    }

    std::ifstream in(dir / kUserDefaultsFileName);
    if (!in.is_open()) {
      ADD_FAILURE() << "value " << value << ": no personal-defaults file was written";
      continue;
    }
    const nlohmann::json on_disk = nlohmann::json::parse(in);
    EXPECT_EQ(on_disk["renderer"]["lens_type"].get<std::string>(), kSpellings[value]) << "value " << value;

    ResetUserDefaultsChannels();
    const GuiState after = MakeNewDocumentState(dir);
    EXPECT_EQ(after.renderer.lens_type, value)
        << "a new document started from a personal default that is not the one adopted";
    EXPECT_EQ(TakeUserDefaultsDowngradeCount(), 0) << "value " << value << ": nothing about this value is wrong";
  }
}

// ---------------------------------------------------------------------------------------------
// The GPU-backend preference: stored, then read back by the next launch.
//
// The closed loop this feature IS. MakeNewDocumentState is what startup, DoNew() and DoOpen()'s
// import all call, so calling it after writing the file is "restart the process and make a new
// document" as far as this chain is concerned — there is no other path a new document arrives by.
//
// Both values are exercised, not just `true`: "stored false" and "stored nothing" are different
// states of the file that happen to produce the same GuiState today, and a reader that collapsed
// them would pass a true-only test.
TEST_F(UserDefaultsChain, TheGpuBackendPreferenceSurvivesIntoTheNextNewDocument) {
  const std::filesystem::path& dir = UseFreshConfigDir("app_use_gpu");

  EXPECT_FALSE(MakeNewDocumentState(dir).use_gpu_backend) << "nothing stored means the factory value";

  nlohmann::json doc = nlohmann::json::object();
  WriteUseGpuBackendToDoc(doc, true);
  ASSERT_TRUE(WriteUserDefaultsFile(dir, doc));

  // The ORDER assertion. ResetIneligibleScalarFields runs unconditionally on this path and puts
  // the field back to `false`; only if the app-preferences read runs AFTER it does the stored
  // value survive. Reversed, this is the one assertion in the suite that goes red — everything
  // else about the file would still be true.
  EXPECT_TRUE(MakeNewDocumentState(dir).use_gpu_backend) << "the stored preference did not reach a new document";

  WriteUseGpuBackendToDoc(doc, false);
  ASSERT_TRUE(WriteUserDefaultsFile(dir, doc));
  EXPECT_FALSE(MakeNewDocumentState(dir).use_gpu_backend);

  EraseUseGpuBackendFromDoc(doc);
  ASSERT_TRUE(WriteUserDefaultsFile(dir, doc));
  EXPECT_FALSE(MakeNewDocumentState(dir).use_gpu_backend) << "erasing it returns the field to the factory value";

  EXPECT_EQ(TakeUserDefaultsDowngradeCount(), 0) << "none of the above is a degradation";
}

// The same name in the file's two halves addresses two different things, and only one of them
// decides the field. Stated from the `app` key's side; test_user_defaults.cpp's
// ac3_ineligible_keys_cannot_be_smuggled_in states the other half. Together they are what stops an
// implementation that reads the wrong location from passing: each alone is satisfied by one.
TEST_F(UserDefaultsChain, OnlyTheAppRootKeyDecidesTheGpuBackendPreference) {
  const std::filesystem::path& dir = UseFreshConfigDir("app_use_gpu_top_level");

  nlohmann::json doc = nlohmann::json::object();
  doc["use_gpu_backend"] = true;  // the document half — inert by construction
  ASSERT_TRUE(WriteUserDefaultsFile(dir, doc));
  EXPECT_FALSE(MakeNewDocumentState(dir).use_gpu_backend) << "the document half must not decide this field";

  // Both present and DISAGREEING: the app key wins, which is the only reading under which the
  // top-level key is genuinely inert rather than merely redundant.
  doc["use_gpu_backend"] = false;
  WriteUseGpuBackendToDoc(doc, true);
  ASSERT_TRUE(WriteUserDefaultsFile(dir, doc));
  EXPECT_TRUE(MakeNewDocumentState(dir).use_gpu_backend);
}

}  // namespace
}  // namespace lumice::gui
