#ifndef LUMICE_GUI_LOGGER_HPP
#define LUMICE_GUI_LOGGER_HPP

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

#include "util/spdlog_levels.hpp"

namespace lumice::gui {

// GUI-side log pattern. Use '%*' (our custom flag) instead of '%L' for level tag.
constexpr const char* kGuiLogPattern = "%Y-%m-%d %H:%M:%S.%e [%*] %v";

// GUI-side logger singleton — completely independent from Core's logger.
// Core logs are received via LUMICE_SetLogCallback(), not through shared spdlog instances.
inline spdlog::logger& GetGuiLogger() {
  static auto logger = []() {
    auto l = spdlog::default_logger()->clone("GUI");
    l->set_formatter(lumice::CreateLumiceFormatter(kGuiLogPattern));
    l->set_level(spdlog::level::warn);  // = our info (see spdlog_levels.hpp)
    // Register so spdlog::flush_every() can flush this logger's file sink.
    spdlog::register_logger(l);
    return l;
  }();
  return *logger;
}

// Replace the GUI logger's sinks (call during initialization, before multi-threaded use).
inline void SetGuiLoggerSinks(std::vector<spdlog::sink_ptr> sinks) {
  GetGuiLogger().sinks() = std::move(sinks);
}

// ---- Sink assembly (two stages, deliberately not one) --------------------------------------
//
// The GUI's sinks are installed at two different points in main(), and the split is load-bearing
// rather than incidental — which is why both halves live here, behind names, instead of as two
// inline blocks in main() that a reader has to reconstruct the ordering rule from.
//
// InstallEarlyGuiSinks() must run FIRST, before GLFW / GL / ImGui init, because every failure
// those stages can report is itself logged: a sink installed after them silently drops exactly
// the startup diagnostics a user needs. Nothing it installs depends on a window, a GL context or
// an ImGui context. On Windows it must still come after the FreeConsole() block, so the stdout
// sink is built against the final console state.
//
// AttachGuiFileSink() must run LATER, and specifically NOT from the early call, because
// constructing the file sink TRUNCATES the log file. The sink defaults to level=off (the GUI
// checkbox enables it), so hoisting it would buy no extra coverage while wiping the previous
// run's log on every launch that dies before this point — the launch a user is most likely to be
// diagnosing.
//
// Splitting them has one cost worth stating: the formatter is applied per sink (set_formatter
// clones into each), so the second stage must re-apply it. Both stages do that here, so adding a
// sink or changing kGuiLogPattern is a single-file edit rather than two edits in main() that can
// drift apart.

// Stage 1: the side-effect-free sinks (ImGui ring buffer + stdout). Publishes g_imgui_log_sink,
// sets the formatter, and installs the flush strategy.
void InstallEarlyGuiSinks();

// Stage 2: the file sink. Resolves the log path (beside the other per-user Lumice artifacts,
// falling back to a CWD-relative path when no config directory is available — degrade, never
// fail to start), publishes g_file_log_sink / g_log_file_path, appends the sink and re-applies
// the formatter.
void AttachGuiFileSink();

inline void SetGuiLogLevel(spdlog::level::level_enum level) {
  GetGuiLogger().set_level(level);
}

}  // namespace lumice::gui

// GUI-side log macros. Note: spdlog macro names don't match our level names
// due to the remapping. See spdlog_levels.hpp for the mapping.
#define GUI_LOG_TRACE(...) SPDLOG_LOGGER_TRACE(&lumice::gui::GetGuiLogger(), __VA_ARGS__)
#define GUI_LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(&lumice::gui::GetGuiLogger(), __VA_ARGS__)
#define GUI_LOG_VERBOSE(...) SPDLOG_LOGGER_INFO(&lumice::gui::GetGuiLogger(), __VA_ARGS__)
#define GUI_LOG_INFO(...) SPDLOG_LOGGER_WARN(&lumice::gui::GetGuiLogger(), __VA_ARGS__)
#define GUI_LOG_WARNING(...) SPDLOG_LOGGER_ERROR(&lumice::gui::GetGuiLogger(), __VA_ARGS__)
#define GUI_LOG_ERROR(...) SPDLOG_LOGGER_CRITICAL(&lumice::gui::GetGuiLogger(), __VA_ARGS__)

#endif  // LUMICE_GUI_LOGGER_HPP
