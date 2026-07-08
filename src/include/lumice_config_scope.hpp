// C++ RAII helpers for LUMICE_Config lifetime management. Header-only; sits next to
// lumice.h and only calls the public C API. GUI / tests should include this instead of
// hand-writing LUMICE_ConfigReleaseColorClasses cleanup on every early return.
//
// Design (task-344, BREAKING v4.8): `LUMICE_Config::raypath_color` is a heap-allocated
// LUMICE_ColorClass* owned by the struct. Anything that populates the array
// (LUMICE_ConfigCreateColorClasses, LUMICE_ParseConfigString / _File, GUI's
// FillLumiceConfig) leaves the caller holding a config that must Release before scope
// exit — otherwise the allocation leaks. ImGui-style IM_CHECK / gtest EXPECT_EQ /
// early-return code paths make manual cleanup easy to skip; ConfigColorGuard makes it
// automatic.

#ifndef LUMICE_LUMICE_CONFIG_SCOPE_HPP_
#define LUMICE_LUMICE_CONFIG_SCOPE_HPP_

#include "lumice.h"

namespace lumice {

// Non-owning of the LUMICE_Config struct itself; owns only the lifetime of its
// raypath_color allocation. Attach it to a stack-local config immediately after
// declaration, BEFORE any call that may populate raypath_color[]:
//
//     LUMICE_Config cfg{};
//     lumice::ConfigColorGuard color_guard(cfg);
//     LUMICE_ParseConfigString(json_str, &cfg);   // may allocate raypath_color
//     // ... use cfg ...
//     // color_guard destructor calls LUMICE_ConfigReleaseColorClasses on scope exit,
//     // regardless of IM_CHECK / EXPECT_EQ / exception early return.
//
// Non-copyable and non-movable to avoid two guards ever pointing at the same config
// (which would double-Release).
class ConfigColorGuard {
 public:
  explicit ConfigColorGuard(LUMICE_Config& cfg) noexcept : cfg_(&cfg) {}
  ~ConfigColorGuard() { LUMICE_ConfigReleaseColorClasses(cfg_); }
  ConfigColorGuard(const ConfigColorGuard&) = delete;
  ConfigColorGuard& operator=(const ConfigColorGuard&) = delete;
  ConfigColorGuard(ConfigColorGuard&&) = delete;
  ConfigColorGuard& operator=(ConfigColorGuard&&) = delete;

 private:
  LUMICE_Config* cfg_;
};

}  // namespace lumice

#endif  // LUMICE_LUMICE_CONFIG_SCOPE_HPP_
