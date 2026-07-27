#ifndef LUMICE_TEST_GUI_USER_DEFAULTS_TEST_ENV_HPP
#define LUMICE_TEST_GUI_USER_DEFAULTS_TEST_ENV_HPP

// Test-side environment control for the personal-defaults store, shared by every gui_test source
// that touches it (the store tests, the diff-engine tests and the panel tests).
//
// It lives in a header rather than being copy-pasted per file because these helpers encode
// isolation POLICY, not convenience: "each case gets a fresh directory" and "the scope guard
// restores the harness baseline rather than what it found" are the two rules that keep one case's
// override file from reaching the next in this single-process binary. Three divergent copies of
// that policy would be three chances to weaken it silently.

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "gui/user_defaults.hpp"
#include "test_gui_shared.hpp"

namespace lumice::test_user_defaults {

// A fresh, empty override directory under the system temp dir — never the developer's real OS
// config directory. `tag` must be unique per case; reusing one silently shares state.
inline std::filesystem::path FreshOverlayDir(const char* tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path() / (std::string("lumice_user_defaults_") + tag);
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  return dir;
}

// Reset every consumable channel so a case starts from a known state regardless of what ran
// before it in this single-process test binary.
inline void ResetUserDefaultsChannels() {
  gui::ResetUserAxisPresetOverrides();
  gui::TakeUserDefaultsDowngradeCount();
  gui::TakeUserDefaultsClampNotices();
}

inline void WriteRawOverlay(const std::filesystem::path& dir, std::string_view text) {
  std::ofstream out(dir / gui::kUserDefaultsFileName, std::ios::trunc);
  out << text;
}

// Temporarily installs a different process-wide user-config source, then restores gui_test's
// baseline. The destructor goes back to kDisabled — the harness's own default (see
// kTestHarnessUserConfigDefault) — rather than to whatever was set on entry: in a single-process
// suite "restore what I found" propagates a leak from an earlier test instead of ending it, and
// every test in this binary is entitled to start from the harness baseline.
class ScopedUserConfigSource {
 public:
  explicit ScopedUserConfigSource(gui::UserConfigSource source, std::filesystem::path dir = {}) {
    gui::SetUserConfigSourceForProcess(source, std::move(dir));
  }

  ScopedUserConfigSource(const ScopedUserConfigSource&) = delete;
  ScopedUserConfigSource& operator=(const ScopedUserConfigSource&) = delete;

  ~ScopedUserConfigSource() { gui::SetUserConfigSourceForProcess(gui::kTestHarnessUserConfigDefault); }
};

// Forces GetUserConfigDir() to resolve to nullopt for the scope of this object, regardless of
// which platform this binary is running on: it clears every env var one of the three
// Compute*ConfigDir helpers reads (HOME / XDG_CONFIG_HOME / APPDATA), then restores each to its
// original value (or absence) on destruction. Needed to exercise MakeNewDocumentState(nullopt) —
// the exact no-arg call main.cpp / DoNew() / DoOpen() make in production — without leaving this
// single-process test binary's environment changed for every test that runs after this one.
class ScopedNoUserConfigDirEnv {
 public:
  ScopedNoUserConfigDirEnv() {
    Clear("HOME");
    Clear("XDG_CONFIG_HOME");
    Clear("APPDATA");
  }

  ScopedNoUserConfigDirEnv(const ScopedNoUserConfigDirEnv&) = delete;
  ScopedNoUserConfigDirEnv& operator=(const ScopedNoUserConfigDirEnv&) = delete;

  ~ScopedNoUserConfigDirEnv() {
    for (const auto& [name, value] : saved_) {
      Apply(name, value);
    }
  }

 private:
  void Clear(const char* name) {
    const char* current = std::getenv(name);
    saved_.emplace_back(name, current ? std::optional<std::string>(current) : std::nullopt);
    Apply(name, std::nullopt);
  }

  static void Apply(const std::string& name, const std::optional<std::string>& value) {
#if defined(_WIN32)
    _putenv_s(name.c_str(), value ? value->c_str() : "");
#else
    if (value) {
      setenv(name.c_str(), value->c_str(), 1);
    } else {
      unsetenv(name.c_str());
    }
#endif
  }

  std::vector<std::pair<std::string, std::optional<std::string>>> saved_;
};

}  // namespace lumice::test_user_defaults

#endif  // LUMICE_TEST_GUI_USER_DEFAULTS_TEST_ENV_HPP
