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
  kAppPreference,   // an application preference (the log-level trio, the log panel's open state,
                    // the left panel's collapsed state) rather than anything the document
                    // expresses, and with no storage channel of its own yet. NOT the verdict for
                    // an app preference that HAS one: use_gpu_backend is stored under the `app`
                    // root key and is therefore kEligible — see kFieldTierTable's
                    // app_preference_eligible bit.
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
        // sun / sim / renderer — the singleton document config (namespace 1) — and
        // use_gpu_backend, whose channel is the `app` root key (namespace 3) instead. This
        // verdict says WHETHER the field may be a personal default, not THROUGH WHICH half of
        // the override file: nothing consumes it to route a write, and a field's channel is
        // registered on its kFieldTierTable row (app_preference_eligible). It used to read
        // `auto_diff_excluded` here to hold use_gpu_backend back, which was a proxy for a
        // question that bit does not answer — that is the coupling this branch no longer has.
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
// their factory value after the DOCUMENT half of an overlay is applied. Those are the fields the
// document half must never decide, but which are nonetheless structurally ordinary serializable
// scalars — i.e. a hand-edited override file could put one at the top level and have the
// deserializer read it. Predicate: app_preference_eligible, the bit that says a field's only
// legitimate default channel is the `app` root key.
// A coverage assertion in the AC1 test ties this constant to that predicate's population, so
// adding another such field turns the test red instead of silently leaving a hole.
// The predicate used to be `tier == kStructSoft && auto_diff_excluded`, which named the same one
// field for a reason unrelated to defaults; a second app preference registered under any other
// tier would have slipped straight past it.
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

// Provenance stamp for the override file: which generation of this program's key space wrote it.
// Stamped by WriteUserDefaultsFile (the single write owner), so every writer gets it without
// knowing it exists.
//
// It RECORDS, it does not gate and it does not migrate. Nothing in the load path branches on it:
// a file from a newer build still has every one of its other keys applied, because an overlay is
// a bag of independent hints that each already fall back to the factory value when absent, so
// rejecting the bag over one key from the future is strictly worse than today's "ignore the keys
// you don't know". (That is the opposite of the `.lmc` policy, where a document read only in part
// must not be displayed as if it were whole.) The one thing a stamp mismatch produces is a notice
// through NoteUserDefaultsDowngrade.
//
// Its point is future-facing: a key whose NAME and VALUE stay the same while its MEANING moves is
// invisible to the key-existence/value-shape probing every migration in this codebase actually
// uses. `presets.axis.*` stores a bare zenith-std whose meaning depends on ClassifyAxisPreset's
// threshold constants — move those and a stored value silently maps to a different preset. The
// discriminator has to be in the stored files BEFORE that change, which is why it is written now
// while nothing reads it.
//
// ⚠️ Coupling with kGuiStateSchemaVersion (file_io.hpp) — the `.lmc`/GuiState payload version.
// TWO INDEPENDENT COUNTERS, deliberately NOT aligned; do not assume they move together, and do
// not renumber one to match the other. They answer different questions over overlapping but
// unequal key spaces (`presets.*` is overlay-only; `layers` is `.lmc`-only and is excluded from
// the overlay wholesale). A semantic change to a field BOTH can carry must be evaluated against
// both counters.
inline constexpr const char* kUserDefaultsOverlaySchemaVersionKey = "defaults_schema_version";
inline constexpr int kUserDefaultsOverlaySchemaVersion = 1;

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
//
// IT CANNOT BE FOLDED INTO EffectiveAxisPresetZenith, and has been mistaken for dead code twice
// on the assumption that it can. That function returns the COMPOSED AxisDist, in which "nothing
// stored" and "stored a number that happens to equal the factory value" are indistinguishable —
// the two produce the same struct. Only this accessor answers the question the panel and the
// tests actually ask, which is about the OVERRIDE's existence and not about the resulting
// distribution: the same presence-vs-value distinction DefaultDiffRow::has_saved_override draws
// for the settings half of the file.
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

// What storing a value WOULD do, with no IO: the clamp decision on its own.
//
// `accepted` false means the value was refused outright (an unadjustable preset, a non-finite
// number) and there is nothing to store; `message` then says why. `message` is a ready-to-show
// sentence (empty when there is nothing to say) rather than a code each call site would render:
// the panel's warning cell and any other surface reporting the same clamp must not drift into
// describing it differently, and the wording is constrained (it must not imply the bound is
// physical — see the note on the domain in axis_presets.hpp).
//
// Deliberately says nothing about whether anything reached disk: committing is the caller's own
// step (the panel writes its whole working copy in one go), and a struct whose fields are only
// sometimes meaningful invites reading the one that is not.
struct AxisPresetClampResult {
  bool accepted = false;      // the value can be stored (false ⇒ refused; `message` says why)
  bool clamped = false;       // stored_value differs from the requested one
  float stored_value = 0.0f;  // what would be stored; meaningful only when accepted
  std::string message;
};

// The clamp decision on its own: what storing `raw_value` for `preset` would produce, with no IO.
//
// The single owner of that rule and of its wording. Its caller is the defaults panel, a pure editor
// that keeps edits in an in-memory copy until Save, and it goes through this function twice: once
// when the user finishes typing a value (clamp, warn, then store the clamped result in the copy),
// and once when resolving what the copy currently means for display — a hand-edited file can hold
// an out-of-domain value the copy keeps verbatim, and showing it raw would name a number the button
// does not actually give. A second clamp implementation next to either call site would drift from
// this one the first time a domain moves.
AxisPresetClampResult ClampAxisPresetZenithStdForSave(AxisPreset preset, float raw_value);

// The preset library's half of an override DOCUMENT, with no IO: read / write / erase
// presets.axis.<preset>.zenith_std.
//
// One owner for that document shape, so the panel's in-memory copy and the file it is eventually
// written back as cannot spell the same setting two ways. The erase prunes the parents it empties,
// so a file someone opens by hand does not accumulate `"presets": {"axis": {}}` skeletons. A preset
// with no adjustable face has no key at all, so all three are no-ops (read: nullopt) for it.
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
// launch reads" drift apart. That ordering is the panel's own to keep, and it says so at the call
// site (defaults_panel.cpp's CommitCopy, "ORDER IS PART OF THE CONTRACT"); there is deliberately
// no store-side revert wrapper composing the two here, because a second writer of this file is a
// second answer to "what did the user save".
void AdoptAxisPresetZenithStdOverrideInMemory(AxisPreset preset, std::optional<float> stored_value);

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

namespace detail {

// Layer a sparse override document over the factory document and hand back the merged JSON, one
// step short of deserializing it. Pure: no IO, no globals.
//
// Exposed in detail:: for unit testing; not part of the stable public surface. Intended callers:
// ApplyUserDefaultsOverlay's own body, and gui_unit_test. Production code must NOT call this to
// assemble an overlay of its own — ApplyUserDefaultsOverlay orchestrates a fixed sequence around
// it (classify the stamp, then the effectively-empty early return, THEN merge), and reaching
// straight for the merge step skips the first two. The detail:: namespace makes that boundary
// structural rather than comment-only: it keeps the function out of lumice::gui's public surface
// (and IDE autocomplete there) while still leaving it reachable for gui_unit_test.
//
// It exists as a separate, header-visible function only because the invariant below is otherwise
// untestable. DeserializeGuiStateJson does not read `schema_version` at all, so a polluted value
// in the merged document produces NO observable difference in the resulting GuiState — a black-box
// test through ApplyUserDefaultsOverlay stays green with the protection deleted. Testing it
// requires looking at the merged document itself, before it is consumed.
//
// The invariant: the `schema_version` in the returned document is ALWAYS this build's
// kGuiStateSchemaVersion, whatever the disk file said. A hand-edited overlay may carry a root
// `schema_version` of its own (nothing stops a user adding one; the diff engine merely never
// writes it), and merge_patch would then overwrite the factory value with it — or, for a JSON
// `null`, delete the key outright. Harmless today because nothing downstream reads it; it is
// precisely the thing that starts biting on the day something does, which is the day a version
// number exists to serve.
nlohmann::json BuildMergedOverlayDocument(const nlohmann::json& doc);

}  // namespace detail

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

// --------------------------------------------------------------------------------------------------
// The app-preferences half of an override DOCUMENT (namespace 3), with no IO.
//
// A THIRD root key, `app`, beside the GuiState half and `presets`. Structurally a sibling of the
// preset library and for the same reason: SerializeGuiStateJson / DeserializeGuiStateJson never
// emit or read this key, BuildMergedOverlayDocument's merge_patch is inert on it, and
// BuildDefaultDiffRows' generative walk — which enumerates the keys SerializeGuiStateJson produces
// — cannot see it. So a field stored here is reachable ONLY through the functions below, which is
// what makes "the document half must never decide use_gpu_backend" a structural fact rather than a
// rule someone has to keep.
//
// One field today, deliberately: `use_gpu_backend`. The other namespace-3 members
// (kUnserializedViewFields) stay excluded, and this is not a general per-field registry — the shape
// leaves room for a second member without pre-building a mechanism for one that does not exist.
//
// The read is tolerant because the file is user-editable: a missing key, a non-object `app`, or a
// non-bool value all read as nullopt, i.e. "nothing stored", and NO downgrade is counted. That is
// the one place this half deliberately diverges from the document half's "one field-level type
// error discards the whole overlay": `app` is independent of the document half, so a typo in it
// must not cost the user their document defaults, nor vice versa.
std::optional<bool> ReadUseGpuBackendFromDoc(const nlohmann::json& doc);
void WriteUseGpuBackendToDoc(nlohmann::json& doc, bool value);
// Prunes the parents it empties, like EraseAxisPresetZenithStdFromDoc, so a file opened by hand
// does not accumulate an empty `"app": {}` skeleton.
void EraseUseGpuBackendFromDoc(nlohmann::json& doc);

// Apply the app-preferences half of `doc` to `state`: each stored field is assigned, each absent
// one leaves `state` untouched (so an overlay with no `app` key yields the factory value).
void ApplyAppPreferencesOverride(GuiState& state, const nlohmann::json& doc);

// The two steps that decide this field, in the one order that is correct, in one function body.
//
// It exists so the order is enforced by control flow instead of by two call sites agreeing to
// remember it: ResetIneligibleScalarFields re-asserts unconditionally that namespace 1 (the
// document half) may not decide these fields, and ApplyAppPreferencesOverride then lets namespace
// 3 — their only legitimate source — override the just-reset factory value. Reversed, the reset
// would wipe the personal default that had just been applied, and the feature would be silently
// dead while every other assertion about the file stayed green.
//
// `has_dir` false means there is no readable config directory: the reset still runs (it is not
// optional), and the app half is left alone — `state` keeps its factory value.
//
// This is the ONLY call site of ResetIneligibleScalarFields, verified rather than assumed
// (`grep -rn ResetIneligibleScalarFields src/ test/`; every other hit is prose, not a call). A new
// caller that needs the reset must go through this function rather than reaching for it directly,
// even one that has no interest in the app-preferences half — that is what keeps "namespace 1
// cannot decide this field" a property of the code rather than of the current call graph.
void ResetIneligibleScalarsThenApplyAppOverride(GuiState& state, const nlohmann::json& doc, bool has_dir);

// Build the GuiState for a NEW document: factory defaults, plus the user's personal defaults,
// plus the standard seed contents. This is the ONLY place personal defaults enter a GuiState
// (invariant I1: they apply to "New", never to a value present in a file being loaded).
//
// `override_dir` exists so tests can inject a temporary directory; production call sites pass
// nothing and fall back to GetUserConfigDir().
GuiState MakeNewDocumentState(std::optional<std::filesystem::path> override_dir = std::nullopt);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_USER_DEFAULTS_HPP
