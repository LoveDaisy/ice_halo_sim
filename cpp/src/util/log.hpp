#ifndef SRC_UTIL_LOG_H_
#define SRC_UTIL_LOG_H_

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace icehalo {

// Log output pattern: "2026-01-02 10:11:12.345 [I] this is a message"
constexpr const char* kLogPattern = "%Y-%m-%d %H:%M:%S.%e [%L] %v";

// Initialize the default logger with the project pattern.
// Call this once at program startup (e.g., in main()).
inline void InitLogger() {
  spdlog::set_pattern(kLogPattern);
  spdlog::set_level(spdlog::level::info);
}

// Set the global log level at runtime (e.g., from -v / -d flags).
inline void SetLogLevel(spdlog::level::level_enum level) {
  spdlog::set_level(level);
}

// Create a named logger for class-level use.
// Returns an existing logger if one with the same name already exists.
// Usage: static auto logger = icehalo::GetLogger("ClassName");
inline std::shared_ptr<spdlog::logger> GetLogger(const std::string& name) {
  auto logger = spdlog::get(name);
  if (!logger) {
    logger = spdlog::default_logger()->clone(name);
    spdlog::register_logger(logger);
  }
  return logger;
}

}  // namespace icehalo

// Convenience macros — map to spdlog default logger
#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_VERBOSE(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARNING(...) SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_FATAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#endif  // SRC_UTIL_LOG_H_
