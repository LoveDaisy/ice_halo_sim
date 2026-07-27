#ifndef LUMICE_GUI_USER_DEFAULTS_HPP
#define LUMICE_GUI_USER_DEFAULTS_HPP

// User-level default values: field eligibility + the sparse override-file contract.
//
// Two orthogonal halves live here:
//
//   1. Eligibility (header-only, pure). "May this GuiState top-level field be stored as a
//      personal default?" is DERIVED from the field-tier registry in gui_state_tiers.hpp —
//      the single source that scripts/check_policies.py already forces every GuiState field
//      into. There is deliberately NO hand-written "fields that may be defaults" list here:
//      a second schema would drift the first time someone adds a field and forgets it.
//
//   2. The override store (declared here, defined in user_defaults.cpp). The on-disk artifact
//      is a PARTIAL GuiState JSON plus a sibling "presets" section, living in the OS user
//      config directory. The GuiState half needs no new merge logic: the existing
//      deserializer is already "missing key = factory value" (file_io.cpp
//      ParseRendererFromGuiJson et al.), so a sparse document read through it yields exactly
//      "factory defaults with the user's keys applied".
//
// The split is not cosmetic. Half 1 has no dependency on file_io.cpp, so it is unit-testable
// from unit_correctness_test (which links lumice_obj only); half 2 calls into file_io.cpp and
// is therefore exercised from gui_test (which links lumice_gui_obj).

#include <cstddef>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "gui/axis_presets.hpp"
#include "gui/gui_state.hpp"
#include "gui/gui_state_tiers.hpp"

namespace lumice::gui {

// ==================================================================================================
// Part 1 — eligibility (pure, header-only)
// ==================================================================================================

enum class DefaultEligibility {
  kEligible,     // may be stored as a personal default
  kIneligible,   // may NOT — see IneligibleReason
  kUnregistered  // name is in neither governance table; only reachable via a typo or a field that
                 // escaped scripts/check_policies.py. Never a legitimate verdict.
};

enum class IneligibleReason {
  kNone,            // paired with kEligible / kUnregistered
  kDerivedRuntime,  // kDerivedFieldsExcludeList: outputs / runtime-derived state
  kSessionOnly,     // FieldTier::kSession: explicitly not persisted
  kCollection,      // namespace 4: crystals / layers / filters / raypath_color — a key path into
                    // these contains a document-local index, which is meaningless as a default
  kNotSerialized,   // in no persisted schema at all (FieldTier::kDisplay)
  kAppPreference,   // an application preference (log levels, backend choice) rather than
                    // anything the document expresses; a defaults system for these is a
                    // separate feature and none is offered yet
};

// Fields the user override file must never be able to fill in, keyed by the mechanism that
// makes them ineligible rather than by an enumerated name list (the list IS kFieldTierTable).
inline constexpr const char* kCollectionFields[] = {
  "crystals",
  "layers",
  "filters",
  "raypath_color",
};

// FieldTier::kView members that SerializeGuiStateJson does NOT emit a root key for, i.e.
// `kView \ serialized`. These are app preferences (namespace 3), not document view state.
//
// This list is a one-hand recomputation against SerializeGuiStateJson, NOT a transcription of
// the design doc: test_user_defaults_ac2_kview_difference_set (test/gui/functional/
// test_gui_user_defaults.cpp) re-derives it at test time by serializing a factory GuiState and
// probing for each kView field's expected root key, so a future change that starts (or stops)
// serializing one of these turns the test red instead of silently widening eligibility.
inline constexpr const char* kUnserializedViewFields[] = {
  "left_panel_collapsed", "gui_log_level", "core_log_level", "log_to_file", "log_panel_open",
};

struct EligibilityVerdict {
  DefaultEligibility eligibility = DefaultEligibility::kUnregistered;
  IneligibleReason reason = IneligibleReason::kNone;
};

namespace user_defaults_detail {

template <std::size_t N>
constexpr bool Contains(const char* const (&list)[N], std::string_view name) {
  for (std::size_t i = 0; i < N; ++i) {
    if (name == list[i]) {
      return true;
    }
  }
  return false;
}

}  // namespace user_defaults_detail

// Resolve whether a GuiState top-level field may be stored as a personal default.
// Total over the governance union (kFieldTierTable + kDerivedFieldsExcludeList); anything else
// returns kUnregistered, which the AC1 test treats as a failure.
inline EligibilityVerdict ResolveDefaultEligibility(std::string_view field_name) {
  if (user_defaults_detail::Contains(kDerivedFieldsExcludeList, field_name)) {
    return { DefaultEligibility::kIneligible, IneligibleReason::kDerivedRuntime };
  }
  for (const auto& entry : kFieldTierTable) {
    if (field_name != entry.name) {
      continue;
    }
    switch (entry.tier) {
      case FieldTier::kSession:
        return { DefaultEligibility::kIneligible, IneligibleReason::kSessionOnly };
      case FieldTier::kStructHard:
        // Both current members (filters / raypath_color) are collections.
        return { DefaultEligibility::kIneligible, IneligibleReason::kCollection };
      case FieldTier::kStructSoft:
        if (user_defaults_detail::Contains(kCollectionFields, field_name)) {
          return { DefaultEligibility::kIneligible, IneligibleReason::kCollection };
        }
        if (entry.auto_diff_excluded) {
          // Currently only use_gpu_backend: registered kStructSoft for governance coverage but
          // deliberately outside the Revert baseline. It is an app preference (namespace 3).
          return { DefaultEligibility::kIneligible, IneligibleReason::kAppPreference };
        }
        // sun / sim / renderer — the singleton document config (namespace 1).
        return { DefaultEligibility::kEligible, IneligibleReason::kNone };
      case FieldTier::kDisplay:
        // Currently only raypath_color_mode. Neither SerializeGuiStateJson nor
        // DeserializeGuiStateJson touches it, so it is not in the persisted schema this
        // override file reuses. Making it default-able would first require extending the .lmc
        // format itself to persist it — an orthogonal, larger decision.
        return { DefaultEligibility::kIneligible, IneligibleReason::kNotSerialized };
      case FieldTier::kView:
        if (user_defaults_detail::Contains(kUnserializedViewFields, field_name)) {
          return { DefaultEligibility::kIneligible, IneligibleReason::kAppPreference };
        }
        return { DefaultEligibility::kEligible, IneligibleReason::kNone };
    }
  }
  return { DefaultEligibility::kUnregistered, IneligibleReason::kNone };
}

inline const char* IneligibleReasonLabel(IneligibleReason reason) {
  switch (reason) {
    case IneligibleReason::kNone:
      return "none";
    case IneligibleReason::kDerivedRuntime:
      return "derived-runtime";
    case IneligibleReason::kSessionOnly:
      return "session-only";
    case IneligibleReason::kCollection:
      return "collection";
    case IneligibleReason::kNotSerialized:
      return "not-serialized";
    case IneligibleReason::kAppPreference:
      return "app-preference";
  }
  return "unknown";
}

// Number of fields ResetIneligibleScalarFields() (user_defaults.cpp) explicitly restores to
// their factory value after an overlay is applied. Those are the ineligible fields that are
// nonetheless structurally ordinary serializable scalars, i.e. a hand-edited override file
// COULD smuggle them in. Predicate: tier == kStructSoft && auto_diff_excluded.
// A coverage assertion in the AC1 test ties this constant to that predicate's population, so
// adding another such field turns the test red instead of silently leaving a hole.
inline constexpr std::size_t kIneligibleScalarResetFieldCount = 1;

// ==================================================================================================
// Part 2 — the override store (defined in user_defaults.cpp)
// ==================================================================================================

inline constexpr const char* kUserDefaultsFileName = "user_defaults.json";

// Pure OS config-directory computation — no environment reads, no filesystem IO, so all three
// are testable on any host regardless of which platform it actually is. An absent OR EMPTY
// input string counts as "unset" (the XDG spec says so explicitly, and an empty %APPDATA% /
// $HOME would otherwise produce a relative path next to the CWD).
inline std::optional<std::filesystem::path> ComputeWindowsConfigDir(const std::optional<std::string>& appdata) {
  if (!appdata || appdata->empty()) {
    return std::nullopt;
  }
  return std::filesystem::path(*appdata) / "Lumice";
}

inline std::optional<std::filesystem::path> ComputeMacConfigDir(const std::optional<std::string>& home) {
  if (!home || home->empty()) {
    return std::nullopt;
  }
  return std::filesystem::path(*home) / "Library" / "Application Support" / "Lumice";
}

inline std::optional<std::filesystem::path> ComputeLinuxConfigDir(const std::optional<std::string>& xdg_config_home,
                                                                  const std::optional<std::string>& home) {
  if (xdg_config_home && !xdg_config_home->empty()) {
    return std::filesystem::path(*xdg_config_home) / "lumice";
  }
  if (home && !home->empty()) {
    return std::filesystem::path(*home) / ".config" / "lumice";
  }
  return std::nullopt;
}

// The real per-user config directory for this platform, created on demand. Returns nullopt
// (after a GUI_LOG_WARNING) when the environment gives us nothing to anchor to or the
// directory cannot be created — callers must degrade, never abort.
std::optional<std::filesystem::path> GetUserConfigDir();

// Read `<dir>/user_defaults.json`. Three distinct outcomes, deliberately not collapsed:
//   - file absent      → empty object, downgrade counter untouched (this is the FIRST-RUN path
//                        and by far the most common one; counting it would make the notice
//                        fire for every user who has never saved a default)
//   - file unreadable / not valid JSON / not a JSON object → empty object, ONE downgrade counted
//   - file parses      → the parsed object
nlohmann::json ReadOverlayJsonIfPresent(const std::filesystem::path& dir);

// Apply the GuiState half of an override document.
//
// Contract note: this REPLACES `state` with "factory defaults + the document's keys"; it does
// not merge into whatever `state` happened to hold. That is what the reuse of
// DeserializeGuiStateJson buys us (it resets to GuiState{} then fills), and the sole
// production caller passes a freshly constructed GuiState, so the two are equivalent there.
// On a field-level type error the whole overlay is discarded (`state` untouched) and one
// downgrade is counted — a partially applied overlay would be worse than none.
void ApplyUserDefaultsOverlay(GuiState& state, const nlohmann::json& doc);

// Apply the preset-library half: root["presets"]["axis"]["<preset>"]["zenith_std"].
// A user may retune a built-in preset, but only within the domain ClassifyAxisPreset still
// recognizes as that preset — otherwise the preset library would be able to define a "Column"
// the classifier calls Custom. Values outside the domain are therefore clamped, and each clamp
// is recorded in TakeUserDefaultsClampNotices(): a silent clamp is a silent data loss.
void ApplyAxisPresetOverridesFromJson(const nlohmann::json& root);

// The user's zenith-std override for a built-in axis preset, if any. Consumed by the preset
// library UI (a later task); nullopt means "use the factory value".
std::optional<float> GetUserAxisPresetZenithStdOverride(AxisPreset preset);

// Drop every loaded preset override. Called at the start of every
// ApplyAxisPresetOverridesFromJson so each load reflects only its own doc, not an accumulation
// across MakeNewDocumentState()'s repeated production call sites (startup / DoNew / DoOpen
// .json import). Also called directly by tests between cases in the same process, for the same
// reason: without it, one test's overrides would leak into the next.
void ResetUserAxisPresetOverrides();

// Write `doc` to `<dir>/user_defaults.json`, creating `dir` if needed. WHAT goes in `doc` is
// the caller's business (the defaults panel decides which keys the user adopted); this only
// owns "given a document, put it on disk". Returns false + GUI_LOG_WARNING on failure.
bool WriteUserDefaultsFile(const std::filesystem::path& dir, const nlohmann::json& doc);

// Number of override-file degradations since the last call, then resets — same shape as
// TakeShapeDistDowngradeCount(), so the load-time notice can be surfaced through the same
// one-shot import-warning path. Counts: unreadable/invalid file, field-level type error,
// non-numeric or non-finite preset value.
int TakeUserDefaultsDowngradeCount();

// Human-readable descriptions of preset values clamped into the tolerance domain during the
// last load, then resets. A plain counter would not satisfy the requirement that the user can
// tell WHICH key was clamped and to what.
std::vector<std::string> TakeUserDefaultsClampNotices();

// Build the GuiState for a NEW document: factory defaults, plus the user's personal defaults,
// plus the standard seed contents. This is the ONLY place personal defaults enter a GuiState
// (invariant I1: they apply to "New", never to a value present in a file being loaded).
//
// `override_dir` exists so tests can inject a temporary directory; production call sites pass
// nothing and fall back to GetUserConfigDir().
GuiState MakeNewDocumentState(std::optional<std::filesystem::path> override_dir = std::nullopt);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_USER_DEFAULTS_HPP
