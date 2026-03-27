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
