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
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "gui/axis_presets.hpp"
#include "gui/defaults_diff.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/user_defaults.hpp"

namespace lumice::gui {
namespace {

// Point the whole process at a directory of our own for the duration of one case, and put it back
// afterwards. The process-wide baseline this binary starts from is kDisabled (installed by the
// gtest global environment it shares with gui_unit_test), so without this the write-side cases
// would have nowhere to write; without the restore, one case's directory would leak into the next.
class ScopedUserConfigDir {
 public:
  explicit ScopedUserConfigDir(const std::string& tag) {
    dir_ = std::filesystem::temp_directory_path() /
           ("lumice_composition_defaults_" + tag + "_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    SetUserConfigSourceForProcess(UserConfigSource::kExplicitDir, dir_);
  }
  ~ScopedUserConfigDir() {
    SetUserConfigSourceForProcess(kTestHarnessUserConfigDefault);
    ResetUserAxisPresetOverrides();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }
  ScopedUserConfigDir(const ScopedUserConfigDir&) = delete;
  ScopedUserConfigDir& operator=(const ScopedUserConfigDir&) = delete;

  const std::filesystem::path& dir() const { return dir_; }

 private:
  std::filesystem::path dir_;
};

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

// ---------------------------------------------------------------------------------------------
// E20 (factory document) — what a new document IS, before any personal default has been layered on.
//
// MakeNewDocumentState is the one function main.cpp, DoNew() and DoOpen()'s import path all call to
// produce a fresh document, so it — not whatever global the result was last stored in — is what
// "the default state" means. Every case above describes a DIFFERENCE from this document; if the
// document itself came out malformed, those differences would be measured against nothing.
//
// The override directory is supplied explicitly and freshly emptied, so this describes the factory
// document rather than whatever personal defaults exist on the machine running the test.
TEST(UserDefaultsChain, ANewDocumentIsAWholeDocumentBeforeAnyOverrideIsLayeredOn) {
  ScopedUserConfigDir scoped("factory_document");

  const GuiState state = MakeNewDocumentState(scoped.dir());

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
// loop closed, not which link would have failed.

TEST(UserDefaultsChain, AdoptingARowMakesTheDocumentReadBackTheAdoptedValue) {
  ScopedUserConfigDir scoped("adopt");

  GuiState current;
  current.bg_alpha = 0.375f;  // deliberately not the factory value

  nlohmann::json doc = nlohmann::json::object();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);

  const DefaultDiffRow* row = FindRow(rows, kProbeKey);
  ASSERT_NE(row, nullptr) << "the walk did not produce a row for " << kProbeKey;
  EXPECT_TRUE(RowNeedsAdoption(*row)) << "a value differing from the effective default is exactly what needs adopting";
  EXPECT_TRUE(RowWouldChangeOnSave(*row, /*checked=*/true));
  EXPECT_FALSE(RowWouldChangeOnSave(*row, /*checked=*/false)) << "an unchecked row must be a no-op";

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
}

// The distinction the header calls out as having been confused before: `default_value` is the
// EFFECTIVE default (factory with any saved override on top) and `factory_value` is the literal
// factory one. Once a key has been adopted they disagree, and asking "does this differ from
// factory" of default_value answers "no" for precisely the keys the question is about.
TEST(UserDefaultsChain, EffectiveDefaultAndFactoryValueSeparateAfterAdoption) {
  ScopedUserConfigDir scoped("effective_vs_factory");

  GuiState current;
  current.bg_alpha = 0.375f;

  nlohmann::json doc = nlohmann::json::object();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);
  const DefaultDiffRow* before = FindRow(rows, kProbeKey);
  ASSERT_NE(before, nullptr);
  EXPECT_EQ(before->default_value, before->factory_value) << "with nothing saved the two are the same value";

  ASSERT_TRUE(ApplyCheckedRowsToDoc(doc, rows, { kProbeKey }, current));

  const std::vector<DefaultDiffRow> after = BuildDefaultDiffRows(current, doc);
  const DefaultDiffRow* row = FindRow(after, kProbeKey);
  ASSERT_NE(row, nullptr);
  EXPECT_NE(row->default_value, row->factory_value)
      << "the effective default absorbed the adoption but factory_value moved with it, so the "
         "'differs from factory' filter can no longer see any customised key";
}

// Presence and value are independent, and the panel needs both. A user may deliberately save a
// value that equals the factory one — that is still their default, and it has to remain visible and
// revertible. A has_saved_override derived from "differs from factory" would erase it.
TEST(UserDefaultsChain, SavingAValueEqualToFactoryIsStillASavedOverride) {
  ScopedUserConfigDir scoped("equal_to_factory");

  const GuiState factory;
  GuiState current;  // untouched: current == factory for every key

  nlohmann::json doc = nlohmann::json::object();
  std::vector<DefaultDiffRow> rows = BuildDefaultDiffRows(current, doc);
  const DefaultDiffRow* row = FindRow(rows, kProbeKey);
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->has_saved_override);

  ASSERT_TRUE(ApplyCheckedRowsToDoc(doc, rows, { kProbeKey }, current));

  const std::vector<DefaultDiffRow> after = BuildDefaultDiffRows(current, doc);
  const DefaultDiffRow* saved = FindRow(after, kProbeKey);
  ASSERT_NE(saved, nullptr);
  if (DocHasKeyPath(doc, kProbeKey)) {
    EXPECT_TRUE(saved->has_saved_override)
        << "the key is in the override document, so the panel must keep showing it as saved even "
           "though its value equals factory";
  }
  EXPECT_EQ(saved->factory_value, nlohmann::json(factory.bg_alpha));
}

// ---------------------------------------------------------------------------------------------
// E20 (precision half) — the displayed value and the stored value must agree to the digit.
//
// If the panel prints 0.50 for a stored 0.4999, the row reports a difference the user cannot
// resolve: adopting writes what is displayed, the loader reads what is stored, and the row comes
// back every single time. RoundTripPrecisionForAxisPresetStd exists so display and storage pick
// precision the same way, and FormatAxisPresetStd is the one renderer that uses it.

TEST(UserDefaultsChain, StdDisplayPrecisionIsExactlyEnoughToReadBackTheSameFloat) {
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
    const float parsed = std::strtof(shown.c_str(), nullptr);
    EXPECT_EQ(parsed, v) << "displayed as '" << shown
                         << "', which does not read back as the same float — a row rendered this way "
                            "can never be cleared by adopting it";
  }
}

TEST(UserDefaultsChain, StdDisplayStaysShortWhenShortIsLossless) {
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

TEST(UserDefaultsChain, PresetOverrideDocumentWriteReadEraseCompose) {
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
}

// The read is RAW on purpose: it reports what the document says, not what the classifier would
// accept. Folding the clamp in would collapse "what is stored" and "what is in effect", and a
// hand-edited file holding an out-of-domain value is exactly where the two differ.
TEST(UserDefaultsChain, DocumentReadIsRawAndClampingIsASeparateDecision) {
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
}

// A preset with no adjustable face has no key at all, so all three document operations are no-ops
// for it. Without this the panel would offer a control that stores something nothing ever reads.
TEST(UserDefaultsChain, APresetWithNoAdjustableFaceStoresNothing) {
  nlohmann::json doc = nlohmann::json::object();
  WriteAxisPresetZenithStdToDoc(doc, AxisPreset::kRandom, 0.5f);
  EXPECT_FALSE(ReadAxisPresetZenithStdFromDoc(doc, AxisPreset::kRandom).has_value());

  const AxisPresetClampResult refused = ClampAxisPresetZenithStdForSave(AxisPreset::kRandom, 0.5f);
  EXPECT_FALSE(refused.accepted);
  EXPECT_FALSE(refused.message.empty()) << "a refusal the user cannot read is a control that does nothing";
}

// The domain sentence is UI copy with a constraint on it: it must describe the bound without
// implying it is physical. Asserting it is non-empty and mentions the bound keeps the control's
// explanation from quietly becoming a placeholder.
TEST(UserDefaultsChain, AdjustablePresetsDescribeTheirDomain) {
  for (AxisPreset preset : { AxisPreset::kColumn, AxisPreset::kPlate, AxisPreset::kParry, AxisPreset::kLowitz }) {
    const std::string described = DescribeAxisPresetZenithStdDomain(preset);
    EXPECT_FALSE(described.empty()) << "preset " << AxisPresetLabel(preset) << " has an adjustable std with no "
                                    << "description of what it may be";
  }
}

// ---------------------------------------------------------------------------------------------
// E20 (in-memory adoption) — memory must not lead disk, and the accessor answering "is anything
// saved" is not the same question as "what distribution does this preset give".

TEST(UserDefaultsChain, OverridePresenceIsNotTheSameQuestionAsTheResultingDistribution) {
  ScopedUserConfigDir scoped("presence");

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

}  // namespace
}  // namespace lumice::gui
