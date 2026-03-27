#ifndef SRC_UTIL_LOGGER_H_
#define SRC_UTIL_LOGGER_H_

#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

#include "util/spdlog_levels.hpp"

namespace lumice {

// Log output pattern. Use '%*' (our custom flag) instead of '%L' for level tag.
// See spdlog_levels.hpp for the T/D/V/I/W/E mapping.
constexpr const char* kLogPattern = "%Y-%m-%d %H:%M:%S.%e [%*] %v";


// Application log levels, mapped to spdlog's 6 numeric levels:
//   trace(0) < debug(1) < verbose(2) < info(3) < warning(4) < error(5) < off(6)
enum class LogLevel {
  kTrace,
  kDebug,
  kVerbose,
  kInfo,
  kWarning,
  kError,
  kOff,
};


// Shared dist_sink singleton. All Logger instances use this as their sole sink,
// so adding a sink here (e.g., callback sink, file sink) is immediately visible
// to all loggers. No initialization order dependency.
inline std::shared_ptr<spdlog::sinks::dist_sink_mt>& GetSharedSink() {
  static auto sink = []() {
    auto s = std::make_shared<spdlog::sinks::dist_sink_mt>();
    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    stdout_sink->set_formatter(CreateLumiceFormatter(kLogPattern));
    s->add_sink(stdout_sink);
    return s;
  }();
  return sink;
}


class Logger {
 public:
  explicit Logger(const std::string& name) : impl_(std::make_shared<spdlog::logger>(name, GetSharedSink())) {
    impl_->set_formatter(CreateLumiceFormatter(kLogPattern));
  }

  void SetLevel(LogLevel level) { impl_->set_level(ToSpdLevel(level)); }

  spdlog::logger* GetSpdLogger() const { return impl_.get(); }

 private:
  // Maps our LogLevel to spdlog's numeric levels.
  // Our verbose→spdlog::info, our info→spdlog::warn, etc.
  // See spdlog_levels.hpp for the full mapping rationale.
  static spdlog::level::level_enum ToSpdLevel(LogLevel level) {
    switch (level) {
      case LogLevel::kTrace:
        return spdlog::level::trace;
      case LogLevel::kDebug:
        return spdlog::level::debug;
      case LogLevel::kVerbose:
        return spdlog::level::info;
      case LogLevel::kInfo:
        return spdlog::level::warn;
      case LogLevel::kWarning:
        return spdlog::level::err;
      case LogLevel::kError:
        return spdlog::level::critical;
      case LogLevel::kOff:
        return spdlog::level::off;
      default:
        return spdlog::level::warn;
    }
  }

  std::shared_ptr<spdlog::logger> impl_;
};


// Global Logger singleton (used by LOG_* macros).
inline Logger& GetGlobalLogger() {
  static Logger instance("global");
  return instance;
}

// Initialize the global logger and spdlog default logger pattern.
// Call once at program startup.
inline void InitGlobalLogger() {
  GetGlobalLogger().SetLevel(LogLevel::kInfo);
}

}  // namespace lumice


// Instance logger macros (for classes that own a Logger member).
// Note: spdlog macro names don't match our level names due to the remapping.
// Our verbose→SPDLOG_INFO, our info→SPDLOG_WARN, etc. See spdlog_levels.hpp.
#define ILOG_TRACE(logger, ...) SPDLOG_LOGGER_TRACE((logger).GetSpdLogger(), __VA_ARGS__)
#define ILOG_DEBUG(logger, ...) SPDLOG_LOGGER_DEBUG((logger).GetSpdLogger(), __VA_ARGS__)
#define ILOG_VERBOSE(logger, ...) SPDLOG_LOGGER_INFO((logger).GetSpdLogger(), __VA_ARGS__)
#define ILOG_INFO(logger, ...) SPDLOG_LOGGER_WARN((logger).GetSpdLogger(), __VA_ARGS__)
#define ILOG_WARN(logger, ...) SPDLOG_LOGGER_ERROR((logger).GetSpdLogger(), __VA_ARGS__)
#define ILOG_ERROR(logger, ...) SPDLOG_LOGGER_CRITICAL((logger).GetSpdLogger(), __VA_ARGS__)

// Global logger macros (for config/core/util utility code)
#define LOG_TRACE(...) ILOG_TRACE(lumice::GetGlobalLogger(), __VA_ARGS__)
#define LOG_VERBOSE(...) ILOG_VERBOSE(lumice::GetGlobalLogger(), __VA_ARGS__)
#define LOG_DEBUG(...) ILOG_DEBUG(lumice::GetGlobalLogger(), __VA_ARGS__)
#define LOG_INFO(...) ILOG_INFO(lumice::GetGlobalLogger(), __VA_ARGS__)
#define LOG_WARNING(...) ILOG_WARN(lumice::GetGlobalLogger(), __VA_ARGS__)
#define LOG_ERROR(...) ILOG_ERROR(lumice::GetGlobalLogger(), __VA_ARGS__)

#endif  // SRC_UTIL_LOGGER_H_
