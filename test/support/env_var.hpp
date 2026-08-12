#ifndef LUMICE_TEST_SUPPORT_ENV_VAR_HPP_
#define LUMICE_TEST_SUPPORT_ENV_VAR_HPP_

// Cross-platform environment-variable set/clear for test code — the single place in the tree that
// owns the `_WIN32` branch behind it.
//
// Why a named helper rather than one more `#ifdef` per call site: `setenv` / `unsetenv` are POSIX,
// not C or C++, and MSVC does not declare them. A bare `::setenv(...)` therefore compiles on mac and
// Linux and fails to compile on Windows — but only in a build configuration that actually reaches
// the file. Two CUDA test sources sat behind `#if LUMICE_CUDA_ENABLED` and the Metal ones behind
// `#if defined(__APPLE__)`, so their bare calls were invisible until a Windows host built with CUDA
// on and BUILD_TEST on for the first time, three weeks after they landed. Copies of the portability
// workaround are what let that happen; one owner is what stops it recurring. (Same conclusion as the
// `no-bare-print` logger exemptions: make the exception a symbol with a name and an owner.)
//
// It lives in test/support/ because its consumers already span four CMake targets
// (unit_correctness_test, parity_test, gui_test, gui_unit_test), all of which carry
// ${PROJ_TEST_DIR} on their include path, so `#include "support/env_var.hpp"` is one spelling valid
// in all of them. Nothing here is production code: `src/` reads env vars through
// src/util/env_knobs.cpp and never writes them.
//
// Deliberately two free functions rather than an RAII guard. Most call sites are function-scoped
// set/clear pairs written inline, and the two places that do want scope semantics want *different*
// ones — test_c_api.cpp's EnvGuard only has to clear on exit, while
// user_defaults_test_env.hpp's ScopedNoUserConfigDirEnv has to restore each variable's prior value
// (or its prior absence). Those two lifetimes stay where they are and compose these calls; folding
// them into one guard here would be a second, unrelated refactor.

#include <cstdlib>

namespace lumice::test {

// Sets `name` to `value`, overwriting any existing value (POSIX `overwrite=1` semantics).
inline void SetEnvVar(const char* name, const char* value) {
#if defined(_WIN32)
  ::_putenv_s(name, value);
#else
  ::setenv(name, value, /*overwrite=*/1);
#endif
}

// Removes `name` from the environment, so a subsequent std::getenv(name) returns nullptr.
//
// On Windows that is spelled as an assignment of the empty string: the CRT treats `_putenv_s(name,
// "")` as a deletion, and correspondingly has no way to express "present but empty". Every knob
// these tests toggle (LUMICE_GPU_GEOM_CLOCK, LUMICE_DISABLE_DEVICE_GEN, LUMICE_TRACE_BACKEND, HOME
// / XDG_CONFIG_HOME / APPDATA) is read with a presence-or-non-empty test, so the two states are
// interchangeable for them — but a future caller whose variable must distinguish empty from absent
// on Windows cannot get that from this function.
inline void UnsetEnvVar(const char* name) {
#if defined(_WIN32)
  ::_putenv_s(name, "");
#else
  ::unsetenv(name);
#endif
}

}  // namespace lumice::test

#endif  // LUMICE_TEST_SUPPORT_ENV_VAR_HPP_
