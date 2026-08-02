// gui_unit_test's process-level personal-defaults baseline.
//
// gui_test installs kTestHarnessUserConfigDefault from its own main() before any case runs, so a
// case reaching the no-arg MakeNewDocumentState() sees "no personal defaults" instead of whatever
// user_defaults.json happens to exist on the machine. gui_unit_test shares test_main.cpp with the
// non-GUI targets, which cannot call into lumice_gui_obj — so the install lives here instead, in a
// gtest global environment only this target compiles.
//
// It is not belt-and-braces. Without it the process source stays kAutoDetect (the unset value, see
// user_defaults.cpp), which resolves to the real OS config directory: pointing HOME at a directory
// holding a saved bg_alpha turns switch_harness_baseline_is_isolated red, measured. Every future
// case in this binary that calls the no-arg path inherits the same exposure, which is why the fix
// belongs to the harness rather than to the one case that happened to surface it.

#include <gtest/gtest.h>

#include "gui/user_defaults.hpp"

namespace {

class UserConfigBaselineEnv : public ::testing::Environment {
 public:
  void SetUp() override { lumice::gui::SetUserConfigSourceForProcess(lumice::gui::kTestHarnessUserConfigDefault); }
};

// Registered during static initialization: gtest keeps its environment list in a function-local
// static inside UnitTest::GetInstance(), so registering before main() is well defined.
[[maybe_unused]] const ::testing::Environment* kUserConfigBaseline =
    ::testing::AddGlobalTestEnvironment(new UserConfigBaselineEnv);

}  // namespace
