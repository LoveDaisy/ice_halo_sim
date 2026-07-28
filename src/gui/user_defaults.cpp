#include "gui/user_defaults.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <system_error>
#include <utility>

#include "gui/file_io.hpp"
#include "gui/gui_logger.hpp"
#include "util/path_utils.hpp"

namespace lumice::gui {

namespace {

// Degradation bookkeeping for the current load. Same shape as file_io.cpp's
// g_shape_dist_downgrade_count: a TU-local counter consumed (and zeroed) by its Take* function.
// Override-file loading is single-threaded (startup / New / import), so no synchronization is
// needed and no accumulator has to be threaded through every helper.
int g_downgrade_count = 0;
std::vector<std::string> g_downgrade_notices;

// Loaded preset-library overrides, indexed by AxisPreset. Only the presets whose kAxisPresets row
// has an adjustable face can ever be populated.
struct AxisPresetOverride {
  bool has_zenith_std = false;
  float zenith_std = 0.0f;
};
constexpr std::size_t kAxisPresetSlotCount = 6;  // AxisPreset has 6 enumerators
static_assert(kAxisPresetSlotCount == static_cast<std::size_t>(AxisPreset::kCustom) + 1,
              "kAxisPresetSlotCount must track AxisPreset's enumerator count (kCustom is its "
              "last member) so the g_axis_overrides index never goes out of bounds silently");

// Value type (not a bare array) so MakeNewDocumentState() can replace the whole thing with one
// unconditional assignment instead of a loop gated behind a branch. code-review round 4: gating
// the reset inside `if (dir)` was the second time a branch wrapped around this reset produced a
// stale-slot leak (round 2's Major, fixed by 806eff19, was the first) — the fix this time is to
// make the assignment itself unconditional rather than adding another conditional reset.
struct AxisPresetOverrides {
  AxisPresetOverride slots[kAxisPresetSlotCount];
};
AxisPresetOverrides g_axis_overrides;

// Process-wide personal-defaults source, installed once from argv by each binary's main() (see
// SetUserConfigSourceForProcess). kAutoDetect is the unset value, so a binary that never calls
// the setter keeps the pre-switch behavior.
UserConfigSource g_process_user_config_source = UserConfigSource::kAutoDetect;
std::filesystem::path g_process_user_config_explicit_dir;

std::optional<std::string> ReadEnv(const char* key) {
  const char* value = std::getenv(key);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

// Clamp a stored zenith std into the domain ClassifyAxisPreset still recognizes as this preset:
//   Column / Plate / Parry : (0, kColumnPlateParryZenithStdUpperBound)
//   Lowitz                 : (kLowitzZenithStdLowerBound, inf)
// Both domains are OPEN, so the clamp target is the next representable float inside the
// interval rather than the bound itself — landing exactly on the bound would fail the
// classifier's strict inequality and turn the preset into "Custom".
// Returns true when the value was modified.
bool ClampZenithStdToPresetDomain(AxisPreset preset, float& value) {
  constexpr float kInf = std::numeric_limits<float>::infinity();
  const float original = value;
  if (preset == AxisPreset::kLowitz) {
    if (!(value > kLowitzZenithStdLowerBound)) {
      value = std::nextafter(kLowitzZenithStdLowerBound, kInf);
    }
  } else {
    if (!(value > 0.0f)) {
      value = std::nextafter(0.0f, kInf);
    } else if (!(value < kColumnPlateParryZenithStdUpperBound)) {
      value = std::nextafter(kColumnPlateParryZenithStdUpperBound, 0.0f);
    }
  }
  return value != original;
}

// The ONE sentence describing a clamp, shared by the load path and the two edit paths (the panel's
// warning cell and the modal's "Save as <preset>" gesture). Three call sites wording the same
// event differently is how a user ends up believing three different things happened.
//
// COPY CONSTRAINT: it must not present the bound as a physical fact. Plate's dead zone [10, 15) is
// the clearest case — a user asking for std=12 is refused because 12 is where the Lowitz criterion
// starts, i.e. for an implementation reason. Saying "not physically possible" there would be a
// lie, and one the user cannot check.
std::string DescribeAxisPresetClamp(AxisPreset preset, float requested, float stored) {
  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  // Both names: the label is what the user pressed, the dotted key is what they would search for
  // if they open the override file to check. Naming only one leaves whichever half of that they
  // are holding unmatched.
  return std::string("Preset '") + entry.label + "' (presets.axis." + entry.override_json_name + "): a zenith std of " +
         FormatAxisPresetStd(requested) + " is outside the range this preset is still recognised in, so " +
         FormatAxisPresetStd(stored) + " was stored instead. Allowed: " + DescribeAxisPresetZenithStdDomain(preset) +
         " — that boundary is where the neighbouring preset's criterion begins, not a physical limit.";
}

// Reset the ineligible fields that a hand-edited override file could otherwise smuggle in.
//
// Most ineligible fields are structurally unreachable from the override file: kDisplay and the
// `kView \ serialized` set have no JSON key at all, and the collections are cleared wholesale
// by the caller. The exception is a field that is registered kStructSoft with
// auto_diff_excluded — an ordinary serializable scalar that DeserializeGuiStateJson will
// happily read. Today that is exactly `use_gpu_backend`.
//
// Keep kIneligibleScalarResetFieldCount (user_defaults.hpp) in step with this body; the AC1
// test asserts the constant equals the population of the predicate above, so adding another
// such field turns that test red rather than leaving a silent hole here.
void ResetIneligibleScalarFields(GuiState& state) {
  const GuiState factory{};
  state.use_gpu_backend = factory.use_gpu_backend;
}

}  // namespace

int TakeUserDefaultsDowngradeCount() {
  int n = g_downgrade_count;
  g_downgrade_count = 0;
  return n;
}

std::vector<std::string> TakeUserDefaultsDowngradeNotices() {
  std::vector<std::string> notices;
  notices.swap(g_downgrade_notices);
  return notices;
}

void NoteUserDefaultsDowngrade(std::string notice) {
  ++g_downgrade_count;
  GUI_LOG_WARNING("[GUI] User defaults: {}", notice);
  g_downgrade_notices.push_back(std::move(notice));
}

std::optional<std::filesystem::path> GetUserConfigDir() {
  std::optional<std::filesystem::path> dir;
#if defined(_WIN32)
  dir = ComputeWindowsConfigDir(ReadEnv("APPDATA"));
#elif defined(__APPLE__)
  dir = ComputeMacConfigDir(ReadEnv("HOME"));
#else
  dir = ComputeLinuxConfigDir(ReadEnv("XDG_CONFIG_HOME"), ReadEnv("HOME"));
#endif
  if (!dir) {
    GUI_LOG_WARNING(
        "[GUI] GetUserConfigDir: no per-user config directory available from the environment; "
        "user defaults are disabled for this session");
    return std::nullopt;
  }

  std::error_code ec;
  std::filesystem::create_directories(*dir, ec);
  if (!std::filesystem::is_directory(*dir)) {
    GUI_LOG_WARNING("[GUI] GetUserConfigDir: cannot create '{}' ({}); user defaults are disabled for this session",
                    PathToU8(*dir), ec.message());
    return std::nullopt;
  }
  return dir;
}

std::optional<std::filesystem::path> GetActiveUserConfigDir() {
  switch (g_process_user_config_source) {
    case UserConfigSource::kDisabled:
      return std::nullopt;
    case UserConfigSource::kExplicitDir:
      return g_process_user_config_explicit_dir;
    case UserConfigSource::kAutoDetect:
      return GetUserConfigDir();
  }
  return GetUserConfigDir();  // unreachable; silences -Wreturn-type on some compilers
}

void SetUserConfigSourceForProcess(UserConfigSource source, std::filesystem::path explicit_dir) {
  g_process_user_config_source = source;
  g_process_user_config_explicit_dir = std::move(explicit_dir);
}

nlohmann::json ReadOverlayJsonIfPresent(const std::filesystem::path& dir) {
  const std::filesystem::path file = dir / kUserDefaultsFileName;

  // Short-circuit BEFORE the try/catch: "the user has never saved a default" is the normal
  // first-run state, not a degradation. Folding it into the failure path would make the
  // load-time notice fire for every fresh install.
  std::error_code ec;
  if (!std::filesystem::exists(file, ec) || ec) {
    return nlohmann::json::object();
  }

  std::ifstream in(file);
  if (!in.is_open()) {
    ++g_downgrade_count;
    GUI_LOG_WARNING("[GUI] User defaults: cannot open '{}'; ignoring it", PathToU8(file));
    return nlohmann::json::object();
  }

  try {
    nlohmann::json doc = nlohmann::json::parse(in);
    if (!doc.is_object()) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: '{}' is not a JSON object; ignoring it", PathToU8(file));
      return nlohmann::json::object();
    }
    return doc;
  } catch (const std::exception& e) {
    ++g_downgrade_count;
    GUI_LOG_WARNING("[GUI] User defaults: '{}' is not valid JSON ({}); ignoring it", PathToU8(file), e.what());
    return nlohmann::json::object();
  }
}

void ApplyUserDefaultsOverlay(GuiState& state, const nlohmann::json& doc) {
  if (!doc.is_object() || doc.empty()) {
    return;
  }

  // Layer the sparse override onto a full factory document rather than handing the fragment
  // straight to the deserializer. Both routes end at the same values (the deserializer is
  // already "missing key = factory value"), but this one keeps the deserializer's own
  // "no renderer key found" diagnostic honest: it is meant to flag a malformed .lmc, and it
  // would otherwise fire on every startup for a user whose defaults touch no renderer setting.
  nlohmann::json merged;
  GuiState overlaid;
  try {
    merged = nlohmann::json::parse(SerializeGuiStateJson(GuiState{}));
    merged.merge_patch(doc);
    if (!DeserializeGuiStateJson(merged.dump(), overlaid)) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: override document could not be applied; using factory defaults");
      return;
    }
  } catch (const std::exception& e) {
    // A field-level type error (`"bg_alpha": "not a number"`) throws out of the middle of the
    // merge. Discard the whole overlay: a half-applied set of defaults is harder to reason
    // about than none, and `state` is left exactly as the caller had it.
    ++g_downgrade_count;
    GUI_LOG_WARNING("[GUI] User defaults: override document has a malformed value ({}); using factory defaults",
                    e.what());
    return;
  }

  state = std::move(overlaid);
}

// Parse the preset-library half of an override document: root["presets"]["axis"]["<preset>"]
// ["zenith_std"]. Pure function — it neither reads nor writes g_axis_overrides; the caller
// (MakeNewDocumentState()) decides how the result replaces the global, unconditionally on every
// call. A user may retune a built-in preset, but only within the domain ClassifyAxisPreset still
// recognizes as that preset — otherwise the preset library would be able to define a "Column"
// the classifier calls Custom. Values outside the domain are therefore clamped, and each clamp
// is recorded in TakeUserDefaultsDowngradeNotices(): a silent clamp is a silent data loss.
AxisPresetOverrides ParseAxisPresetOverrides(const nlohmann::json& root) {
  AxisPresetOverrides result{};

  if (!root.is_object() || !root.contains("presets")) {
    return result;
  }
  const nlohmann::json& presets = root["presets"];
  if (!presets.is_object() || !presets.contains("axis")) {
    return result;
  }
  const nlohmann::json& axis = presets["axis"];
  if (!axis.is_object()) {
    return result;
  }

  for (const auto& entry : kAxisPresets) {
    if (!entry.has_adjustable_zenith_std) {
      continue;  // no key on disk for this preset — see AxisPresetEntry's field comments
    }
    if (!axis.contains(entry.override_json_name)) {
      continue;
    }
    const nlohmann::json& node = axis[entry.override_json_name];
    if (!node.is_object() || !node.contains("zenith_std")) {
      continue;
    }
    const nlohmann::json& value_node = node["zenith_std"];
    if (!value_node.is_number()) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: presets.axis.{}.zenith_std is not a number; ignoring it",
                      entry.override_json_name);
      continue;
    }
    float value = value_node.get<float>();
    if (!std::isfinite(value)) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: presets.axis.{}.zenith_std is not finite; ignoring it",
                      entry.override_json_name);
      continue;
    }

    const float original = value;
    if (ClampZenithStdToPresetDomain(entry.id, value)) {
      // Never silent: at load time the user is not looking at the preset panel, so a clamp with
      // no trace would be indistinguishable from the value having been dropped.
      NoteUserDefaultsDowngrade(DescribeAxisPresetClamp(entry.id, original, value));
    }

    const auto slot = static_cast<std::size_t>(entry.id);
    result.slots[slot] = { true, value };
  }
  return result;
}

int RoundTripPrecisionForAxisPresetStd(float value) {
  char buffer[40];
  for (int precision = 6; precision < 9; ++precision) {
    std::snprintf(buffer, sizeof(buffer), "%.*g", precision, static_cast<double>(value));
    if (std::strtof(buffer, nullptr) == value) {
      return precision;
    }
  }
  return 9;
}

std::string FormatAxisPresetStd(float value) {
  // Shortest form that reads back as the same float, not a fixed precision. A fixed %.6g renders
  // the tuned values a user types correctly (0.3 stays "0.3") but collapses the clamp target —
  // nextafter(10, 0) is 9.99999905, which %.6g rounds to "10" and turns the notice into a
  // contradiction: "10 was stored instead. Allowed: less than 10." Escalating only when the short
  // form is lossy keeps the common case short AND the boundary case honest.
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.*g", RoundTripPrecisionForAxisPresetStd(value), static_cast<double>(value));
  return buffer;
}

std::string DescribeAxisPresetZenithStdDomain(AxisPreset preset) {
  if (preset == AxisPreset::kLowitz) {
    return "greater than " + FormatAxisPresetStd(kLowitzZenithStdLowerBound);
  }
  return "greater than 0 and less than " + FormatAxisPresetStd(kColumnPlateParryZenithStdUpperBound);
}

AxisPresetClampResult ClampAxisPresetZenithStdForSave(AxisPreset preset, float raw_value) {
  AxisPresetClampResult result;

  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  // Both halves tested, not just the predicate: override_json_name is what a write indexes the
  // document with, and a nullptr there is not a refusal but a crash. The static_assert in
  // axis_presets.hpp makes the two agree, so this second clause is unreachable today — it is here
  // so that "refuses cleanly" does not depend on that assert still being in place.
  if (!entry.has_adjustable_zenith_std || entry.override_json_name == nullptr) {
    // Second of two defenses (the UI draws no input for these). A warning rather than an assert:
    // an assert is compiled out of the release build, which is the build the requirement — that
    // an unadjustable preset never reaches the override file — is actually about.
    GUI_LOG_WARNING("[GUI] User defaults: preset '{}' has no adjustable zenith std; nothing was saved", entry.label);
    result.message = std::string(entry.label) + " has no adjustable value, so nothing was saved.";
    return result;
  }
  if (!std::isfinite(raw_value)) {
    GUI_LOG_WARNING("[GUI] User defaults: preset '{}' zenith std is not finite; nothing was saved", entry.label);
    result.message = "That value is not a number, so nothing was saved.";
    return result;
  }

  float stored = raw_value;
  result.clamped = ClampZenithStdToPresetDomain(preset, stored);
  result.accepted = true;
  result.stored_value = stored;
  result.message = result.clamped ? DescribeAxisPresetClamp(preset, raw_value, stored) : std::string();
  return result;
}

std::optional<float> ReadAxisPresetZenithStdFromDoc(const nlohmann::json& doc, AxisPreset preset) {
  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  if (!entry.has_adjustable_zenith_std || entry.override_json_name == nullptr || !doc.is_object()) {
    return std::nullopt;
  }
  // find() rather than the operator[] chain: the document is user-editable, and operator[] on a
  // non-object throws a type_error that would take the caller down over a hand-edit.
  const auto presets = doc.find("presets");
  if (presets == doc.end() || !presets->is_object()) {
    return std::nullopt;
  }
  const auto axis = presets->find("axis");
  if (axis == presets->end() || !axis->is_object()) {
    return std::nullopt;
  }
  const auto node = axis->find(entry.override_json_name);
  if (node == axis->end() || !node->is_object()) {
    return std::nullopt;
  }
  const auto value = node->find("zenith_std");
  if (value == node->end() || !value->is_number()) {
    return std::nullopt;
  }
  const float stored = value->get<float>();
  if (!std::isfinite(stored)) {
    return std::nullopt;
  }
  return stored;
}

void WriteAxisPresetZenithStdToDoc(nlohmann::json& doc, AxisPreset preset, float stored_value) {
  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  if (!entry.has_adjustable_zenith_std || entry.override_json_name == nullptr) {
    return;
  }
  if (!doc.is_object()) {
    doc = nlohmann::json::object();
  }
  // Surgical: ONE key is touched. The GuiState half of the document and every other preset survive
  // by construction rather than by each caller remembering to preserve them.
  doc["presets"]["axis"][entry.override_json_name]["zenith_std"] = stored_value;
}

void EraseAxisPresetZenithStdFromDoc(nlohmann::json& doc, AxisPreset preset) {
  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  if (!entry.has_adjustable_zenith_std || entry.override_json_name == nullptr || !doc.is_object()) {
    return;
  }
  const auto presets_it = doc.find("presets");
  if (presets_it == doc.end() || !presets_it->is_object()) {
    return;
  }
  const auto axis_it = presets_it->find("axis");
  if (axis_it != presets_it->end() && axis_it->is_object()) {
    axis_it->erase(entry.override_json_name);
    if (axis_it->empty()) {
      presets_it->erase("axis");
    }
  }
  if (presets_it->empty()) {
    doc.erase("presets");
  }
}

void AdoptAxisPresetZenithStdOverrideInMemory(AxisPreset preset, std::optional<float> stored_value) {
  const auto slot = static_cast<std::size_t>(preset);
  if (slot >= kAxisPresetSlotCount) {
    return;
  }
  // Whole-struct assignment, never "clear the fields then refill them". This scrum has already
  // spent three code-review rounds on partial updates to this exact global.
  g_axis_overrides.slots[slot] = stored_value ? AxisPresetOverride{ true, *stored_value } : AxisPresetOverride{};
}

AxisPresetWriteResult SaveAxisPresetZenithStdOverride(AxisPreset preset, float raw_value) {
  AxisPresetWriteResult result;

  // The decision half is shared with the panel's uncommitted-edit path; only the IO below is this
  // function's own.
  const AxisPresetClampResult clamp = ClampAxisPresetZenithStdForSave(preset, raw_value);
  if (!clamp.accepted) {
    result.message = clamp.message;
    return result;
  }
  const float stored = clamp.stored_value;

  const auto dir = GetActiveUserConfigDir();
  if (!dir) {
    GUI_LOG_WARNING("[GUI] User defaults: no user-config directory available; nothing was saved");
    result.message = "Your personal defaults file could not be written, so nothing was saved.";
    return result;
  }
  nlohmann::json doc = ReadOverlayJsonIfPresent(*dir);
  // Read the whole document, touch ONE key, write the whole document back. A wholesale rewrite
  // here would take the GuiState half of the file (and every other preset) with it.
  WriteAxisPresetZenithStdToDoc(doc, preset, stored);
  if (!WriteUserDefaultsFile(*dir, doc)) {
    // In-memory state deliberately untouched on a failed write: "what this session resolves" must
    // not disagree with "what the next launch reads".
    result.message = "Your personal defaults file could not be written, so nothing was saved.";
    return result;
  }

  AdoptAxisPresetZenithStdOverrideInMemory(preset, stored);

  result.written = true;
  result.clamped = clamp.clamped;
  result.stored_value = stored;
  result.message = clamp.message;
  return result;
}

bool RevertOneAxisPresetOverride(AxisPreset preset) {
  const AxisPresetEntry& entry = AxisPresetEntryFor(preset);
  if (!entry.has_adjustable_zenith_std || entry.override_json_name == nullptr) {
    return false;  // nothing can be stored for it, so there is nothing to restore
  }

  const auto dir = GetActiveUserConfigDir();
  if (!dir) {
    GUI_LOG_WARNING("[GUI] User defaults: no user-config directory available; nothing was changed");
    return false;
  }
  nlohmann::json doc = ReadOverlayJsonIfPresent(*dir);
  EraseAxisPresetZenithStdFromDoc(doc, preset);

  if (!WriteUserDefaultsFile(*dir, doc)) {
    return false;
  }
  // Disk first, memory second (see the header): a failed write above returns with the in-memory
  // override still in place, which is what the next launch would read back anyway.
  AdoptAxisPresetZenithStdOverrideInMemory(preset, std::nullopt);
  return true;
}

AxisDist EffectiveAxisPresetZenith(const AxisPresetEntry& entry) {
  AxisDist zenith = entry.zenith;
  if (const auto stored = GetUserAxisPresetZenithStdOverride(entry.id)) {
    zenith.std = *stored;
  }
  return zenith;
}

std::optional<float> GetUserAxisPresetZenithStdOverride(AxisPreset preset) {
  const auto slot = static_cast<std::size_t>(preset);
  if (slot >= kAxisPresetSlotCount || !g_axis_overrides.slots[slot].has_zenith_std) {
    return std::nullopt;
  }
  return g_axis_overrides.slots[slot].zenith_std;
}

void ResetUserAxisPresetOverrides() {
  g_axis_overrides = AxisPresetOverrides{};
}

bool WriteUserDefaultsFile(const std::filesystem::path& dir, const nlohmann::json& doc) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (!std::filesystem::is_directory(dir)) {
    GUI_LOG_WARNING("[GUI] User defaults: cannot create '{}' ({}); nothing was saved", PathToU8(dir), ec.message());
    return false;
  }

  const std::filesystem::path file = dir / kUserDefaultsFileName;
  std::ofstream out(file, std::ios::trunc);
  if (!out.is_open()) {
    GUI_LOG_WARNING("[GUI] User defaults: cannot write '{}'; nothing was saved", PathToU8(file));
    return false;
  }
  out << doc.dump(2) << '\n';
  out.flush();
  if (!out.good()) {
    GUI_LOG_WARNING("[GUI] User defaults: write to '{}' failed; the file may be incomplete", PathToU8(file));
    return false;
  }
  return true;
}

GuiState MakeNewDocumentState(std::optional<std::filesystem::path> override_dir) {
  GuiState state{};

  // An explicit override_dir still outranks everything (tests inject one directly); only the
  // no-arg production path consults the process-wide source installed from argv.
  std::optional<std::filesystem::path> dir = override_dir ? std::move(override_dir) : GetActiveUserConfigDir();
  const nlohmann::json doc = dir ? ReadOverlayJsonIfPresent(*dir) : nlohmann::json{};
  if (dir) {
    ApplyUserDefaultsOverlay(state, doc);
  }
  // Deliberately NOT gated behind the `if (dir)` above (code-review round 4 Major): `state` is a
  // fresh local, so it is zero-residue on every call regardless of `dir`, but g_axis_overrides is
  // a persistent global across MakeNewDocumentState()'s repeated production calls (startup,
  // every DoNew(), every DoOpen() .json import). GetUserConfigDir() can flip from available to
  // unavailable within one process (its own doc comment: the directory can become uncreatable at
  // runtime) — reusing the same `if (dir)` to guard this assignment would leave a prior call's
  // overrides live on exactly that flip, the one case this function exists to degrade from.
  g_axis_overrides = dir ? ParseAxisPresetOverrides(doc) : AxisPresetOverrides{};

  // The override file is user-editable, so the read path — not just the write path — has to
  // enforce eligibility. Otherwise "which fields may be defaults" would be advisory metadata
  // that anyone can step around with a text editor.
  ResetIneligibleScalarFields(state);

  // Namespace 4 (collections): a key path into these carries a document-local index, so they
  // are never written as defaults — but clear them anyway so a hand-edited file cannot make a
  // new document start with someone else's crystals/layers/filters/raypath_color. Must track
  // kCollectionFields (user_defaults.hpp) exactly — that list, not this line count, is the
  // single source of truth for which containers are namespace 4.
  state.crystals.clear();
  state.layers.clear();
  state.filters.clear();
  state.raypath_color.clear();

  SeedDefaultDocumentContents(state);
  return state;
}

}  // namespace lumice::gui
