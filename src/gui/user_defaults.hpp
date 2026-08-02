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
// is therefore exercised from a target that links lumice_gui_obj — gui_unit_test for the cases
// that need no frame, gui_test for the ones that do.

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

// --------------------------------------------------------------------------------------------------
// Where a process reads personal defaults from — the CLI surface (pure, header-only)
// --------------------------------------------------------------------------------------------------
//
// Everything down to ResolveUserConfigSource() is pure argv/enum arithmetic with no dependency on
// GetUserConfigDir(), so it stays in Part 1 and is unit-testable from unit_correctness_test. The
// process-wide setter that consumes the result lives in Part 2 (it has to call GetUserConfigDir()).

enum class UserConfigSource {
  kAutoDetect,   // GetUserConfigDir() — the interactive application's default
  kDisabled,     // --no-user-config: skip the directory lookup entirely, factory values only
  kExplicitDir,  // --user-config <path>: use the caller-supplied directory
};

enum class UserConfigArgPresence {
  kAbsent,        // neither --user-config nor --no-user-config appeared
  kDisableFlag,   // --no-user-config
  kExplicitFlag,  // --user-config <path>
};

struct ParsedUserConfigArg {
  UserConfigArgPresence presence = UserConfigArgPresence::kAbsent;
  std::filesystem::path explicit_dir;  // meaningful only when presence == kExplicitFlag
  // A bare trailing `--user-config` with no path after it. Presence stays kAbsent (the flag
  // cannot be honored), but the caller must be able to SAY so: silently starting in auto-detect
  // while the user believes an explicit directory took effect is the expensive failure here —
  // the startup log would even print "auto-detect", actively pointing away from the typo.
  bool missing_value = false;
};

// Shared by LumiceGUI's main() and gui_test's main() so the two binaries cannot drift on CLI
// syntax. Deliberately does NOT decide what "no flag at all" means — that differs per binary and
// belongs to ResolveUserConfigSource() below. When both flags appear the last one wins, matching
// the tolerant style of the other argv loops in those two files (no hard error).
inline ParsedUserConfigArg ParseUserConfigArg(int argc, char** argv) {
  ParsedUserConfigArg result;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if (arg == "--no-user-config") {
      result.presence = UserConfigArgPresence::kDisableFlag;
      result.missing_value = false;
    } else if (arg == "--user-config") {
      if (i + 1 < argc) {
        result.presence = UserConfigArgPresence::kExplicitFlag;
        result.explicit_dir = argv[++i];
        result.missing_value = false;
      } else {
        result.missing_value = true;
      }
    }
  }
  return result;
}

// The default each binary falls back to when neither flag was passed. These are named constants
// rather than literals inlined in the two main() bodies because the second one is this whole
// task's only behavior change, and a literal there would have no automated regression signal:
// CI machines carry no user_defaults.json, so silently reverting the test harness to kAutoDetect
// would stay green on CI and only surface as drifted reference images on a developer's machine —
// exactly the failure mode the isolation switch exists to remove. As constants they are asserted
// in test/unit-correctness/gui/test_user_defaults_eligibility.cpp.
inline constexpr UserConfigSource kInteractiveAppUserConfigDefault = UserConfigSource::kAutoDetect;
inline constexpr UserConfigSource kTestHarnessUserConfigDefault = UserConfigSource::kDisabled;

// Map a parsed flag onto the source a binary should install, given that binary's own default for
// "no flag passed". Pure, so both mains share one mapping and it can be asserted directly.
inline UserConfigSource ResolveUserConfigSource(UserConfigArgPresence presence, UserConfigSource default_source) {
  switch (presence) {
    case UserConfigArgPresence::kDisableFlag:
      return UserConfigSource::kDisabled;
    case UserConfigArgPresence::kExplicitFlag:
      return UserConfigSource::kExplicitDir;
    case UserConfigArgPresence::kAbsent:
      return default_source;
  }
  return default_source;
}

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

// The directory the CURRENT process reads and writes personal defaults in, per the source
// installed by SetUserConfigSourceForProcess (nullopt under --no-user-config, or when the OS gave
// us no config directory to anchor to).
//
// Public so that the defaults panel's diff/write layer (defaults_diff.cpp) resolves the SAME
// directory MakeNewDocumentState() reads from. Re-deriving the directory there instead would let
// "the defaults the panel shows" and "the defaults a New document gets" drift apart silently,
// which is exactly the split invariant I1 exists to prevent.
std::optional<std::filesystem::path> GetActiveUserConfigDir();

// Install the process-wide source that MakeNewDocumentState()'s no-arg path resolves against.
// All three production call sites (main.cpp startup, DoNew(), DoOpen()'s JSON import) call it
// with no argument, so this one setting covers them without threading a directory through each.
//
// Call it once, in main(), before the first MakeNewDocumentState(). It is not built for repeated
// runtime switching: a test that needs a different source for its own scope must save and restore
// (see ScopedUserConfigSource in test/gui/functional/test_gui_user_defaults.cpp). Left unset, the
// source is kAutoDetect — i.e. a binary that never calls this behaves exactly as before the
// switch existed.
void SetUserConfigSourceForProcess(UserConfigSource source, std::filesystem::path explicit_dir = {});

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

// The user's zenith-std override for a built-in axis preset, if any. nullopt means "use the
// factory value". Prefer EffectiveAxisPresetZenith() below at consumption sites — this raw
// accessor exists for the tests and for the panel's "is anything saved for this preset" cell.
std::optional<float> GetUserAxisPresetZenithStdOverride(AxisPreset preset);

// --------------------------------------------------------------------------------------------------
// Preset-library write side.
//
// The read side above (405.2) could already resolve an override; nothing could PRODUCE one. These
// three close that loop. All of them are surgical edits of the existing override document — the
// GuiState half of the file and every other preset survive by construction, not by each call site
// remembering to preserve them.
//
// Scope of what is storable, and why it is one float rather than a preset's nine fields: the
// classifier (ClassifyAxisPreset) pins mean to within kEpsilon and requires a full-uniform-360
// azimuth, so zenith.std is the only field a user can move and still have the preset keep its
// identity. Storing anything else would let the library define a "Column" the classifier calls
// Custom, which is the failure this whole design exists to avoid.
// --------------------------------------------------------------------------------------------------

// What a write did, in the terms the UI has to report. `message` is a ready-to-show sentence
// (empty when there is nothing to say) rather than a code the two call sites would each render:
// the panel's warning cell and the modal gesture's status line must not drift into describing the
// same clamp differently, and the wording is constrained (it must not imply the bound is physical
// — see the note on the domain in axis_presets.hpp).
struct AxisPresetWriteResult {
  bool written = false;       // the value reached disk (false ⇒ nothing changed, in memory or on it)
  bool clamped = false;       // stored_value differs from the requested one
  float stored_value = 0.0f;  // what is now in effect; meaningful only when written
  std::string message;
};

// What a write WOULD do, with no IO: the clamp decision on its own.
//
// `accepted` is the counterpart of AxisPresetWriteResult::written for a caller that has not
// committed anything yet — false means the value was refused outright (an unadjustable preset, a
// non-finite number) and there is nothing to store; `message` then says why. It is a distinct
// struct from AxisPresetWriteResult rather than a reuse of it precisely because `written` cannot
// be answered here, and a struct whose fields are only sometimes meaningful invites reading the
// one that is not.
struct AxisPresetClampResult {
  bool accepted = false;      // the value can be stored (false ⇒ refused; `message` says why)
  bool clamped = false;       // stored_value differs from the requested one
  float stored_value = 0.0f;  // what would be stored; meaningful only when accepted
  std::string message;
};

// The clamp WITHOUT the write: what storing `raw_value` for `preset` would produce, in the same
// terms AxisPresetWriteResult reports (`written` is meaningless here and stays false).
//
// Exists because there are now two commit timings for one rule. The axis modal's "Save as <preset>"
// gesture writes immediately, while the defaults panel is a pure editor that keeps its edits in an
// in-memory copy until Save. Both must clamp identically and word the clamp identically, so both go
// through this function — SaveAxisPresetZenithStdOverride below is this function plus the IO. A
// second clamp implementation in the panel would drift from this one the first time a domain moves.
AxisPresetClampResult ClampAxisPresetZenithStdForSave(AxisPreset preset, float raw_value);

// The preset library's half of an override DOCUMENT, with no IO: read / write / erase
// presets.axis.<preset>.zenith_std.
//
// One owner for that document shape, shared by the two commit timings — the axis modal's gesture
// writes the file immediately, the defaults panel edits an in-memory copy and writes it once on
// Save. The erase prunes the parents it empties, so a file someone opens by hand does not
// accumulate `"presets": {"axis": {}}` skeletons. A preset with no adjustable face has no key at
// all, so all three are no-ops (read: nullopt) for it.
//
// The read is deliberately RAW: it reports what the document says, not what the classifier would
// accept. Clamping is ClampAxisPresetZenithStdForSave's job, and folding it in here would collapse
// "what is stored" and "what is in effect" into one call — they are not the same thing, because a
// hand-edited file can hold a value the loader clamps.
std::optional<float> ReadAxisPresetZenithStdFromDoc(const nlohmann::json& doc, AxisPreset preset);
void WriteAxisPresetZenithStdToDoc(nlohmann::json& doc, AxisPreset preset, float stored_value);
void EraseAxisPresetZenithStdFromDoc(nlohmann::json& doc, AxisPreset preset);

// Point the in-memory preset cache at `stored_value` (nullopt = "nothing stored, use the factory
// value") without touching any file.
//
// For a caller that has ALREADY committed the document itself — the defaults panel writes its
// whole working copy in one go, so by the time it calls this the value is on disk. Nothing else
// should: memory that leads disk is exactly how "what this session resolves" and "what the next
// launch reads" drift apart, which is why the two write functions above do the file first.
void AdoptAxisPresetZenithStdOverrideInMemory(AxisPreset preset, std::optional<float> stored_value);

// Store `raw_value` as the user's zenith std for `preset`, clamped into the classifier's tolerance
// domain. A preset with no adjustable face (Random / Custom) is REFUSED — with a warning, not an
// assert: this is the second of two defenses (the first being that the UI draws it no input), and
// an assert would be compiled out of the release build, i.e. absent from exactly the build the
// requirement is about.
//
// On a failed disk write the in-memory override is left untouched, so "what this session resolves"
// never disagrees with "what the next launch will read".
AxisPresetWriteResult SaveAxisPresetZenithStdOverride(AxisPreset preset, float raw_value);

// Drop the user's override for one preset, returning it to the factory value. Disk first, memory
// second — the reverse order would leave a failed write reporting success in this session and
// resurrecting the old value on the next launch. Other presets and the GuiState half are untouched.
bool RevertOneAxisPresetOverride(AxisPreset preset);

// The zenith distribution a preset button actually writes: the factory row, with the user's std
// substituted when one is stored. Single source for that resolution — the axis modal's preset
// buttons and the library panel both call it, so "what Column gives you" cannot differ between the
// place you press it and the place you configure it.
AxisDist EffectiveAxisPresetZenith(const AxisPresetEntry& entry);

// Human-readable form of the domain a preset's zenith std must stay inside, e.g.
// "greater than 0 and less than 10". For UI copy only.
std::string DescribeAxisPresetZenithStdDomain(AxisPreset preset);

// The shortest %g precision (6..9) that still reads back as exactly this float via strtof. Shared
// by FormatAxisPresetStd below and by the §1 panel's InputFloat display format so both places pick
// precision the same way: a fixed digit count anywhere renders the tuned values a user types
// correctly (0.3 stays "0.3") but collapses a clamp target that happens to need one more digit —
// nextafter(10, 0) needs 7 ("9.999999"), nextafter(15, +inf) needs 8 ("15.000001") — and a value
// that reads as the boundary itself contradicts the "must stay less/greater than N" text next to
// it. Escalating only when the shorter form is lossy keeps the common case short everywhere.
int RoundTripPrecisionForAxisPresetStd(float value);

// Display form of a std value, via RoundTripPrecisionForAxisPresetStd. Shared so the panel, the
// modal gesture and the load-time notice print "0.3" and "9.99999" the same way; std::to_string
// would render both as a run of trailing zeros or digits that reads like precision the value does
// not carry.
std::string FormatAxisPresetStd(float value);

// Drop every loaded preset override. MakeNewDocumentState() no longer needs this itself (it
// replaces the whole override set with one unconditional assignment on every call — see its
// definition in user_defaults.cpp), but tests still call it directly between cases in the same
// process: without it, one test's overrides would leak into the next.
void ResetUserAxisPresetOverrides();

// Write `doc` to `<dir>/user_defaults.json`, creating `dir` if needed. WHAT goes in `doc` is
// the caller's business (the defaults panel decides which keys the user adopted); this only
// owns "given a document, put it on disk". Returns false + GUI_LOG_WARNING on failure.
bool WriteUserDefaultsFile(const std::filesystem::path& dir, const nlohmann::json& doc);

// Number of override-file degradations since the last call, then resets — same shape as
// TakeShapeDistDowngradeCount(), so the load-time notice can be surfaced through the same
// one-shot import-warning path. Counts: unreadable/invalid file, field-level type error,
// non-numeric or non-finite preset value, a preset value clamped into the tolerance domain, and
// a hand-edited path node that had to be replaced to honor a write.
int TakeUserDefaultsDowngradeCount();

// Human-readable descriptions of the degradations counted above, then resets. A plain counter
// would not satisfy the requirement that the user can tell WHICH key degraded and how.
//
// Named "downgrade" rather than "clamp" because clamping is only one of the things that lands
// here; the channel is deliberately singular (scrum invariant I3: a degradation leaves a trace,
// and one trace is one place to look).
std::vector<std::string> TakeUserDefaultsDowngradeNotices();

// Record one degradation: bumps the counter above and files `notice` for the same load's report.
// THE way another translation unit contributes to this channel — defaults_diff.cpp writes here
// rather than reaching for a counter of its own, so a user sees one list, not two.
void NoteUserDefaultsDowngrade(std::string notice);

// Build the GuiState for a NEW document: factory defaults, plus the user's personal defaults,
// plus the standard seed contents. This is the ONLY place personal defaults enter a GuiState
// (invariant I1: they apply to "New", never to a value present in a file being loaded).
//
// `override_dir` exists so tests can inject a temporary directory; production call sites pass
// nothing and fall back to GetUserConfigDir().
GuiState MakeNewDocumentState(std::optional<std::filesystem::path> override_dir = std::nullopt);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_USER_DEFAULTS_HPP
