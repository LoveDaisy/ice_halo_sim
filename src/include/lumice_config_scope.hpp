// C++ RAII helpers for LUMICE_Config lifetime management. Header-only; sits next to
// lumice.h and only calls the public C API. GUI / tests should include this instead of
// hand-writing per-owning-field cleanup on every early return.
//
// Design (task-344, BREAKING v4.8): `LUMICE_Config::raypath_color` is a heap-allocated
// LUMICE_ColorClass* owned by the struct. Anything that populates the array
// (LUMICE_ConfigCreateColorClasses, LUMICE_ParseConfigString / _File, GUI's
// FillLumiceConfig) leaves the caller holding a config that must Release before scope
// exit — otherwise the allocation leaks. ImGui-style IM_CHECK / gtest EXPECT_EQ /
// early-return code paths make manual cleanup easy to skip; the guard makes it
// automatic.
//
// Extended (task-host-abi-cpu-caps, BREAKING v4.9): every
// `LUMICE_Config::compositions[i]` record now owns two heap pointers (term_ids /
// term_counts) via LUMICE_CompositionSetClauses; the guard also invokes
// LUMICE_ConfigReleaseCompositions on scope exit so callers don't need a second guard.
// The class was renamed `ConfigColorGuard` → `ConfigOwningGuard` to make the extended
// responsibility explicit (a "Color"-scoped name would be misleading for the
// composition release).

#ifndef LUMICE_LUMICE_CONFIG_SCOPE_HPP_
#define LUMICE_LUMICE_CONFIG_SCOPE_HPP_

#include "lumice.h"

namespace lumice {

// Non-owning of the LUMICE_Config struct itself; owns only the lifetime of its heap-
// backed owning fields (raypath_color allocation + each compositions[i].term_ids /
// term_counts allocation). Attach it to a stack-local config immediately after
// declaration, BEFORE any call that may populate those fields:
//
//     LUMICE_Config cfg{};
//     lumice::ConfigOwningGuard cfg_guard(cfg);
//     LUMICE_ParseConfigString(json_str, &cfg);   // may allocate raypath_color and compositions
//     // ... use cfg ...
//     // cfg_guard destructor calls LUMICE_ConfigReleaseColorClasses AND
//     // LUMICE_ConfigReleaseCompositions on scope exit, regardless of IM_CHECK /
//     // EXPECT_EQ / exception early return.
//
// Non-copyable and non-movable to avoid two guards ever pointing at the same config
// (which would double-Release both owning fields).
class ConfigOwningGuard {
 public:
  explicit ConfigOwningGuard(LUMICE_Config& cfg) noexcept : cfg_(&cfg) {}
  ~ConfigOwningGuard() {
    LUMICE_ConfigReleaseColorClasses(cfg_);
    LUMICE_ConfigReleaseCompositions(cfg_);
  }
  ConfigOwningGuard(const ConfigOwningGuard&) = delete;
  ConfigOwningGuard& operator=(const ConfigOwningGuard&) = delete;
  ConfigOwningGuard(ConfigOwningGuard&&) = delete;
  ConfigOwningGuard& operator=(ConfigOwningGuard&&) = delete;

 private:
  LUMICE_Config* cfg_;
};

}  // namespace lumice

#endif  // LUMICE_LUMICE_CONFIG_SCOPE_HPP_
