#include "gui/user_defaults.hpp"

#include <cmath>
#include <cstdlib>
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
std::vector<std::string> g_clamp_notices;

// Loaded preset-library overrides, indexed by AxisPreset. Only the four presets that have an
// adjustable face (see kAxisPresetJsonNames) can ever be populated.
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

struct AxisPresetJsonName {
  const char* name;
  AxisPreset preset;
};

// Lowercase JSON names, matching the convention of the other enum name tables in file_io.cpp
// (kLensTypeJsonNames / kAspectPresetJsonNames). Random has no adjustable face (it is defined
// as three uniform-360 axes), so it has no key.
constexpr AxisPresetJsonName kAxisPresetJsonNames[] = {
  { "column", AxisPreset::kColumn },
  { "plate", AxisPreset::kPlate },
  { "parry", AxisPreset::kParry },
  { "lowitz", AxisPreset::kLowitz },
};

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

std::vector<std::string> TakeUserDefaultsClampNotices() {
  std::vector<std::string> notices;
  notices.swap(g_clamp_notices);
  return notices;
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

namespace {

// The directory MakeNewDocumentState()'s no-arg path reads from, per the process-wide source.
// Deliberately private: callers state their intent once through SetUserConfigSourceForProcess()
// rather than each resolving a directory of their own.
std::optional<std::filesystem::path> ResolveActiveUserConfigDir() {
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

}  // namespace

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
// is recorded in TakeUserDefaultsClampNotices(): a silent clamp is a silent data loss.
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

  for (const auto& entry : kAxisPresetJsonNames) {
    if (!axis.contains(entry.name)) {
      continue;
    }
    const nlohmann::json& node = axis[entry.name];
    if (!node.is_object() || !node.contains("zenith_std")) {
      continue;
    }
    const nlohmann::json& value_node = node["zenith_std"];
    if (!value_node.is_number()) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: presets.axis.{}.zenith_std is not a number; ignoring it", entry.name);
      continue;
    }
    float value = value_node.get<float>();
    if (!std::isfinite(value)) {
      ++g_downgrade_count;
      GUI_LOG_WARNING("[GUI] User defaults: presets.axis.{}.zenith_std is not finite; ignoring it", entry.name);
      continue;
    }

    const float original = value;
    if (ClampZenithStdToPresetDomain(entry.preset, value)) {
      // Never silent: at load time the user is not looking at the preset panel, so a clamp with
      // no trace would be indistinguishable from the value having been dropped.
      std::string notice = std::string("Preset '") + entry.name + "': zenith std " + std::to_string(original) +
                           " is outside the allowed range and was adjusted to " + std::to_string(value);
      GUI_LOG_WARNING("[GUI] User defaults: {}", notice);
      g_clamp_notices.push_back(std::move(notice));
    }

    const auto slot = static_cast<std::size_t>(entry.preset);
    result.slots[slot] = { true, value };
  }
  return result;
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
  std::optional<std::filesystem::path> dir = override_dir ? std::move(override_dir) : ResolveActiveUserConfigDir();
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
