// Unit tests for the pure half of the user-defaults system (src/gui/user_defaults.hpp):
//   AC1 — every governed GuiState top-level field resolves to exactly one eligibility verdict,
//         derived from the field-tier registry rather than a second hand-written list.
//   AC6 — the three platform config-directory functions, including their env-missing fallbacks.
//
// Both halves are header-only and depend on nothing in lumice_gui_obj, which is why they live
// here rather than in gui_test: unit_correctness_test links lumice_obj only. The parts of the
// system that DO call file_io.cpp (overlay read/write, AC2-AC5) are covered by
// test/gui/functional/test_gui_user_defaults.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gui/gui_state_tiers.hpp"
#include "gui/user_defaults.hpp"

namespace lumice::gui {
namespace {

// The governance union's size, asserted below as a deliberate tripwire.
//
// C++ has no reflection, so this test cannot walk GuiState's members the way
// scripts/check_policies.py::check_gui_state_field_tier_registration does. The division of
// labor is: the policy gate proves "every GuiState field is registered in exactly one table"
// (that is where the red/green for an unregistered field lives); this test proves "every
// registered name resolves to a determinate eligibility". The count below ties the two
// together — adding a GuiState field necessarily changes it, which forces whoever adds the
// field to come here and state the field's eligibility on purpose instead of inheriting a
// default by accident.
constexpr std::size_t kExpectedGovernedFieldCount = 63;

std::vector<std::string> AllGovernedFieldNames() {
  std::vector<std::string> names;
  for (const auto& entry : kFieldTierTable) {
    names.emplace_back(entry.name);
  }
  for (const char* name : kDerivedFieldsExcludeList) {
    names.emplace_back(name);
  }
  return names;
}

TEST(UserDefaultsEligibility, GovernanceUnionIsDisjointAndComplete) {
  std::set<std::string> tiered;
  for (const auto& entry : kFieldTierTable) {
    EXPECT_TRUE(tiered.insert(entry.name).second) << "duplicate entry in kFieldTierTable: " << entry.name;
  }
  std::set<std::string> derived;
  for (const char* name : kDerivedFieldsExcludeList) {
    EXPECT_TRUE(derived.insert(name).second) << "duplicate entry in kDerivedFieldsExcludeList: " << name;
  }

  // "Registered in EXACTLY one of the two tables" — the C++-side mirror of the policy gate's
  // invariant. A name in both would make ResolveDefaultEligibility's answer depend on lookup
  // order rather than on the design.
  for (const auto& name : tiered) {
    EXPECT_EQ(derived.count(name), 0u) << name << " is registered in BOTH governance tables";
  }

  EXPECT_EQ(tiered.size() + derived.size(), kExpectedGovernedFieldCount)
      << "GuiState's governed field count changed. Classify the new field's default-eligibility "
         "deliberately (see ResolveDefaultEligibility), then update kExpectedGovernedFieldCount.";
}

TEST(UserDefaultsEligibility, EveryGovernedFieldResolvesToADeterminateVerdict) {
  for (const auto& name : AllGovernedFieldNames()) {
    const EligibilityVerdict verdict = ResolveDefaultEligibility(name);
    ASSERT_NE(verdict.eligibility, DefaultEligibility::kUnregistered) << name << " resolved to kUnregistered";
    if (verdict.eligibility == DefaultEligibility::kEligible) {
      EXPECT_EQ(verdict.reason, IneligibleReason::kNone) << name << " is eligible but carries a reason";
    } else {
      EXPECT_NE(verdict.reason, IneligibleReason::kNone)
          << name << " is ineligible but carries no reason — the reason is what the UI shows";
    }
  }
}

TEST(UserDefaultsEligibility, UnknownNameIsUnregisteredRatherThanSilentlyEligible) {
  // The failure mode this guards: a typo'd or renamed field quietly becoming "eligible" and so
  // silently enterable through the override file.
  const EligibilityVerdict verdict = ResolveDefaultEligibility("no_such_gui_state_field");
  EXPECT_EQ(verdict.eligibility, DefaultEligibility::kUnregistered);
}

TEST(UserDefaultsEligibility, RepresentativeFieldsMapToTheDesignedVerdicts) {
  struct Expectation {
    const char* field;
    DefaultEligibility eligibility;
    IneligibleReason reason;
  };
  // A representative sample per branch — full coverage is guaranteed by the two tests above;
  // this one pins the ACTUAL classification of the fields the design argued about.
  constexpr Expectation kExpectations[] = {
    // namespace 1 — singleton document config (kStructSoft, not a collection).
    { "sun", DefaultEligibility::kEligible, IneligibleReason::kNone },
    { "sim", DefaultEligibility::kEligible, IneligibleReason::kNone },
    { "renderer", DefaultEligibility::kEligible, IneligibleReason::kNone },
    // namespace 1 — kView fields that ARE serialized (default lens / overlays / background).
    { "bg_alpha", DefaultEligibility::kEligible, IneligibleReason::kNone },
    { "aspect_preset", DefaultEligibility::kEligible, IneligibleReason::kNone },
    { "right_panel_collapsed", DefaultEligibility::kEligible, IneligibleReason::kNone },
    { "show_zenith_nadir_line", DefaultEligibility::kEligible, IneligibleReason::kNone },
    // namespace 4 — collections. A key path into these carries a document-local index.
    { "crystals", DefaultEligibility::kIneligible, IneligibleReason::kCollection },
    { "layers", DefaultEligibility::kIneligible, IneligibleReason::kCollection },
    { "filters", DefaultEligibility::kIneligible, IneligibleReason::kCollection },
    { "raypath_color", DefaultEligibility::kIneligible, IneligibleReason::kCollection },
    // namespace 3 — app preference, out of scope for phase one.
    { "use_gpu_backend", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    { "gui_log_level", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    { "core_log_level", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    { "log_to_file", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    { "log_panel_open", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    { "left_panel_collapsed", DefaultEligibility::kIneligible, IneligibleReason::kAppPreference },
    // Not in any persisted schema.
    { "raypath_color_mode", DefaultEligibility::kIneligible, IneligibleReason::kNotSerialized },
    // Session / derived.
    { "current_file_path", DefaultEligibility::kIneligible, IneligibleReason::kSessionOnly },
    { "save_texture", DefaultEligibility::kIneligible, IneligibleReason::kSessionOnly },
    { "sim_state", DefaultEligibility::kIneligible, IneligibleReason::kDerivedRuntime },
    { "dirty", DefaultEligibility::kIneligible, IneligibleReason::kDerivedRuntime },
  };

  for (const auto& e : kExpectations) {
    const EligibilityVerdict verdict = ResolveDefaultEligibility(e.field);
    EXPECT_EQ(verdict.eligibility, e.eligibility) << e.field;
    EXPECT_EQ(verdict.reason, e.reason) << e.field << " reason=" << IneligibleReasonLabel(verdict.reason);
  }
}

TEST(UserDefaultsEligibility, IneligibleScalarResetCoversEverySmugglableField) {
  // Coverage gate for ResetIneligibleScalarFields (user_defaults.cpp). Most ineligible fields
  // are structurally unreachable from the override file — no JSON key at all, or a collection
  // the read path clears wholesale. The exception is a field registered kStructSoft with
  // auto_diff_excluded: an ordinary serializable scalar the deserializer will happily read,
  // which therefore needs an explicit reset. If someone adds another such field without
  // extending that reset, this assertion fails instead of leaving a silent hole.
  //
  // Scope note (deliberate narrowing): the predicate below is exactly the shape of the current
  // exception. A structurally DIFFERENT future category of "serializable but ineligible" would
  // not be caught here — extending the predicate is part of introducing such a category.
  std::size_t smugglable = 0;
  for (const auto& entry : kFieldTierTable) {
    if (entry.tier == FieldTier::kStructSoft && entry.auto_diff_excluded) {
      ++smugglable;
      EXPECT_EQ(ResolveDefaultEligibility(entry.name).eligibility, DefaultEligibility::kIneligible) << entry.name;
    }
  }
  EXPECT_EQ(smugglable, kIneligibleScalarResetFieldCount)
      << "A new kStructSoft/auto_diff_excluded field appeared. Add it to "
         "ResetIneligibleScalarFields() in src/gui/user_defaults.cpp and bump "
         "kIneligibleScalarResetFieldCount.";
}

TEST(UserDefaultsEligibility, UnserializedViewFieldsAreAllRegisteredAsView) {
  // Guards the constant against a stale name (a rename would otherwise silently turn the
  // renamed field eligible). Whether the SET is right is checked against the real serializer
  // in the AC2 test over in gui_test.
  for (const char* name : kUnserializedViewFields) {
    const auto* found = std::find_if(std::begin(kFieldTierTable), std::end(kFieldTierTable),
                                     [name](const FieldTierEntry& e) { return std::string_view(e.name) == name; });
    ASSERT_NE(found, std::end(kFieldTierTable)) << name << " is not a registered GuiState field";
    EXPECT_EQ(found->tier, FieldTier::kView) << name;
  }
}

// ==================================================================================================
// AC6 — platform config directories
// ==================================================================================================

TEST(UserConfigDir, WindowsUsesAppDataSubdirectory) {
  EXPECT_EQ(*ComputeWindowsConfigDir(std::string("C:\\Users\\me\\AppData\\Roaming")),
            std::filesystem::path("C:\\Users\\me\\AppData\\Roaming") / "Lumice");
}

TEST(UserConfigDir, WindowsWithoutAppDataYieldsNullopt) {
  EXPECT_FALSE(ComputeWindowsConfigDir(std::nullopt).has_value());
  // An empty %APPDATA% must count as unset, not produce a CWD-relative "Lumice".
  EXPECT_FALSE(ComputeWindowsConfigDir(std::string("")).has_value());
}

TEST(UserConfigDir, MacUsesApplicationSupport) {
  EXPECT_EQ(*ComputeMacConfigDir(std::string("/Users/me")),
            std::filesystem::path("/Users/me") / "Library" / "Application Support" / "Lumice");
}

TEST(UserConfigDir, MacWithoutHomeYieldsNullopt) {
  EXPECT_FALSE(ComputeMacConfigDir(std::nullopt).has_value());
  EXPECT_FALSE(ComputeMacConfigDir(std::string("")).has_value());
}

TEST(UserConfigDir, LinuxPrefersXdgConfigHome) {
  EXPECT_EQ(*ComputeLinuxConfigDir(std::string("/xdg"), std::string("/home/me")),
            std::filesystem::path("/xdg") / "lumice");
}

TEST(UserConfigDir, LinuxFallsBackToDotConfigWhenXdgMissingOrEmpty) {
  EXPECT_EQ(*ComputeLinuxConfigDir(std::nullopt, std::string("/home/me")),
            std::filesystem::path("/home/me") / ".config" / "lumice");
  // XDG spec: an empty XDG_CONFIG_HOME is treated exactly as unset.
  EXPECT_EQ(*ComputeLinuxConfigDir(std::string(""), std::string("/home/me")),
            std::filesystem::path("/home/me") / ".config" / "lumice");
}

TEST(UserConfigDir, LinuxWithNeitherVariableYieldsNullopt) {
  EXPECT_FALSE(ComputeLinuxConfigDir(std::nullopt, std::nullopt).has_value());
  EXPECT_FALSE(ComputeLinuxConfigDir(std::string(""), std::string("")).has_value());
}

// ==================================================================================================
// The --user-config / --no-user-config CLI surface (pure argv + enum arithmetic)
// ==================================================================================================

// ParseUserConfigArg takes char**, so the literals have to be writable storage.
class ArgvBuilder {
 public:
  explicit ArgvBuilder(std::vector<std::string> args) : storage_(std::move(args)) {
    pointers_.reserve(storage_.size());
    for (auto& arg : storage_) {
      pointers_.push_back(arg.data());
    }
  }

  int argc() const { return static_cast<int>(pointers_.size()); }
  char** argv() { return pointers_.data(); }

 private:
  std::vector<std::string> storage_;
  std::vector<char*> pointers_;
};

ParsedUserConfigArg ParseArgs(std::vector<std::string> args) {
  args.insert(args.begin(), "binary");  // argv[0] is skipped by the parser
  ArgvBuilder builder(std::move(args));
  return ParseUserConfigArg(builder.argc(), builder.argv());
}

TEST(UserConfigArg, NoFlagIsAbsent) {
  const auto parsed = ParseArgs({ "--fixed-dt", "--filter", "foo" });
  EXPECT_EQ(parsed.presence, UserConfigArgPresence::kAbsent);
  EXPECT_FALSE(parsed.missing_value);
}

TEST(UserConfigArg, DisableFlagRecognized) {
  const auto parsed = ParseArgs({ "--no-user-config" });
  EXPECT_EQ(parsed.presence, UserConfigArgPresence::kDisableFlag);
}

TEST(UserConfigArg, ExplicitFlagCapturesPath) {
  const auto parsed = ParseArgs({ "--user-config", "/tmp/x" });
  EXPECT_EQ(parsed.presence, UserConfigArgPresence::kExplicitFlag);
  EXPECT_EQ(parsed.explicit_dir, std::filesystem::path("/tmp/x"));
}

// A trailing `--user-config` with nothing after it must not read past argv's end, and must not
// be mistaken for "the user did not ask for anything" — main() warns on missing_value.
TEST(UserConfigArg, DanglingExplicitFlagIsReportedNotHonored) {
  const auto parsed = ParseArgs({ "--fixed-dt", "--user-config" });
  EXPECT_EQ(parsed.presence, UserConfigArgPresence::kAbsent);
  EXPECT_TRUE(parsed.missing_value);
}

TEST(UserConfigArg, LastFlagWins) {
  const auto disable_last = ParseArgs({ "--user-config", "/tmp/x", "--no-user-config" });
  EXPECT_EQ(disable_last.presence, UserConfigArgPresence::kDisableFlag);

  const auto explicit_last = ParseArgs({ "--no-user-config", "--user-config", "/tmp/x" });
  EXPECT_EQ(explicit_last.presence, UserConfigArgPresence::kExplicitFlag);
  EXPECT_EQ(explicit_last.explicit_dir, std::filesystem::path("/tmp/x"));

  // A dangling flag after a good one leaves the honored value in place but still reports.
  const auto dangling_last = ParseArgs({ "--no-user-config", "--user-config" });
  EXPECT_EQ(dangling_last.presence, UserConfigArgPresence::kDisableFlag);
  EXPECT_TRUE(dangling_last.missing_value);
}

TEST(UserConfigArg, FlagsOutrankTheBinaryDefault) {
  for (const auto default_source :
       { UserConfigSource::kAutoDetect, UserConfigSource::kDisabled, UserConfigSource::kExplicitDir }) {
    EXPECT_EQ(ResolveUserConfigSource(UserConfigArgPresence::kDisableFlag, default_source),
              UserConfigSource::kDisabled);
    EXPECT_EQ(ResolveUserConfigSource(UserConfigArgPresence::kExplicitFlag, default_source),
              UserConfigSource::kExplicitDir);
    EXPECT_EQ(ResolveUserConfigSource(UserConfigArgPresence::kAbsent, default_source), default_source);
  }
}

// The whole point of this task: gui_test must isolate when nobody passes a flag. Reverting that
// default would stay green on CI (no CI machine has a user_defaults.json to leak) and would only
// surface as drifted reference images on a developer's machine, so it needs an explicit assertion
// rather than a build-time-only presence. LumiceGUI's default is asserted alongside it because
// the two being DIFFERENT is the deliberate design decision, not an oversight.
TEST(UserConfigArg, BinaryDefaultsAreTheDocumentedOnes) {
  EXPECT_EQ(kTestHarnessUserConfigDefault, UserConfigSource::kDisabled);
  EXPECT_EQ(kInteractiveAppUserConfigDefault, UserConfigSource::kAutoDetect);

  // No flag passed → each binary lands on its own default.
  EXPECT_EQ(ResolveUserConfigSource(UserConfigArgPresence::kAbsent, kTestHarnessUserConfigDefault),
            UserConfigSource::kDisabled);
  EXPECT_EQ(ResolveUserConfigSource(UserConfigArgPresence::kAbsent, kInteractiveAppUserConfigDefault),
            UserConfigSource::kAutoDetect);
}

}  // namespace
}  // namespace lumice::gui
