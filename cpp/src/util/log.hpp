#ifndef SRC_UTIL_LOG_H_
#define SRC_UTIL_LOG_H_

#include <chrono>
#include <initializer_list>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace icehalo {

enum class LogLevel {
  kDebug,
  kVerbose,
  kInfo,
  kWarning,
  kError,
  kFatal,
};


struct LogMessage {
  LogLevel level;
  std::chrono::system_clock::time_point time;
  int line;
  std::string source_file;
  std::string tag;
  std::string text;
  std::thread::id thread_id;
};


class LogFormatter;
using LogFormatterPtr = std::shared_ptr<LogFormatter>;

class LogFormatter {
 public:
  LogFormatter();
  virtual ~LogFormatter() = default;

  virtual const char* Format(const LogMessage& msg) = 0;

  void EnableSeverity(bool enable);
  void EnableThreadId(bool enable);

 protected:
  static constexpr size_t kBufSize = 1024;

  bool enable_severity_;
  bool enable_thread_id_;
  char buf_[kBufSize];
};


class SimpleLogFormatter : public LogFormatter {
 public:
  const char* Format(const LogMessage& msg) override;
};


class LogDestination;
using LogDestPtrU = std::unique_ptr<LogDestination>;
using LogDestPtr = std::shared_ptr<LogDestination>;

class LogDestination {
 public:
  virtual ~LogDestination() = default;
  virtual void Write(const LogMessage& message, LogFormatterPtr& formatter) = 0;
};


class LogStdOutDest : public LogDestination {
 public:
  void Write(const LogMessage& message, LogFormatterPtr& formatter) override;

  static LogDestPtr GetInstance();

 private:
  LogStdOutDest() = default;
};


class LogStdErrDest : public LogDestination {
 public:
  void Write(const LogMessage& message, LogFormatterPtr& formatter) override;

  static LogDestPtr GetInstance();

 private:
  LogStdErrDest() = default;
};


class LogFilter;
using LogFilterPtrU = std::unique_ptr<LogFilter>;
using LogFilterPtr = std::shared_ptr<LogFilter>;

class LogFilter {
 public:
  virtual ~LogFilter() = default;
  virtual bool Filter(const LogMessage& message) = 0;

  static LogFilterPtrU MakeTagFilter(const char* tag);
  static LogFilterPtrU MakeLevelFilter(std::initializer_list<LogLevel> levels);
};

LogFilterPtr operator&&(const LogFilterPtr& a, const LogFilterPtr& b);
LogFilterPtr operator||(const LogFilterPtr& a, const LogFilterPtr& b);


class LogTagFilter : public LogFilter {
 public:
  explicit LogTagFilter(const char* tag) : tag_(tag) {}

  bool Filter(const LogMessage& message) override;

 private:
  std::string tag_;
};


class LogLevelFilter : public LogFilter {
 public:
  LogLevelFilter(std::initializer_list<LogLevel> levels) : levels_{ levels } {}

  bool Filter(const LogMessage& message) override;

 private:
  std::vector<LogLevel> levels_;
};


class LogComplexFilter : public LogFilter {
 public:
  LogComplexFilter() = default;
  explicit LogComplexFilter(LogFilterPtr filter);

  void Reset();
  void And(LogFilterPtr other);
  void Or(LogFilterPtr other);

  bool Filter(const LogMessage& message) override;

 private:
  std::vector<std::vector<LogFilterPtr>> filters_;
};


class Logger {
 public:
  void EmitLog(const char* file, int line, LogLevel level, const char* fmt, ...) {
    char buf[kMaxMessageLength];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, kMaxMessageLength, fmt, args);
    va_end(args);

    LogMessage msg{ level, std::chrono::system_clock::now(), line, file, "", buf, std::this_thread::get_id() };
    std::unordered_set<LogDestPtr> dest_set;
    for (auto& kv : filter_dest_list_) {
      if (kv.first->Filter(msg) && !dest_set.count(kv.second)) {
        kv.second->Write(msg, default_formatter_);
        dest_set.emplace(kv.second);
      }
    }
  }

  void EmitTagLog(const char* file, int line, LogLevel level, const char* tag, const char* fmt, ...) {
    char buf[kMaxMessageLength];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, kMaxMessageLength, fmt, args);
    va_end(args);

    LogMessage msg{ level, std::chrono::system_clock::now(), line, file, tag, buf, std::this_thread::get_id() };
    std::unordered_set<LogDestPtr> dest_set;
    for (auto& kv : filter_dest_list_) {
      if (kv.first->Filter(msg) && !dest_set.count(kv.second)) {
        kv.second->Write(msg, default_formatter_);
        dest_set.emplace(kv.second);
      }
    }
  }

  void AddDestination(LogFilterPtr filter, LogDestPtr dest);
  void EnableSeverity(bool enable);
  void EnableThreadId(bool enable);

  static Logger* GetInstance();

  static constexpr size_t kMaxMessageLength = 1024;

 private:
  Logger();

  LogFormatterPtr default_formatter_;
  std::vector<std::pair<LogFilterPtr, LogDestPtr>> filter_dest_list_;
};

}  // namespace icehalo

// Convenience macros for log
#define LOG_DEBUG(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kDebug, (fmt), ##__VA_ARGS__)
#define LOG_VERBOSE(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kVerbose, (fmt), ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kInfo, (fmt), ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kWarning, (fmt), ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kError, (fmt), ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) \
  icehalo::Logger::GetInstance()->EmitLog(__FILE__, __LINE__, icehalo::LogLevel::kFatal, (fmt), ##__VA_ARGS__)

#define LOG_TAG_DEBUG(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kDebugTag, (tag), (fmt), \
                                             ##__VA_ARGS__)
#define LOG_TAG_VERBOSE(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kVerboseTag, (tag), (fmt), \
                                             ##__VA_ARGS__)
#define LOG_TAG_INFO(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kInfoTag, (tag), (fmt), \
                                             ##__VA_ARGS__)
#define LOG_TAG_WARNING(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kWarningTag, (tag), (fmt), \
                                             ##__VA_ARGS__)
#define LOG_TAG_ERROR(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kErrorTag, (tag), (fmt), \
                                             ##__VA_ARGS__)
#define LOG_TAG_FATAL(tag, fmt, ...)                                                                         \
  icehalo::Logger::GetInstance()->EmitTagLog(__FILE__, __LINE__, icehalo::LogLevel::kFatalTag, (tag), (fmt), \
                                             ##__VA_ARGS__)

#define ENABLE_LOG_SEVERITY icehalo::Logger::GetInstance()->EnableSeverity(true)
#define DISABLE_LOG_SEVERITY icehalo::Logger::GetInstance()->EnableSeverity(false)
#define ENABLE_LOG_THREAD_ID icehalo::Logger::GetInstance()->EnableThreadId(true)
#define DISABLE_LOG_THREAD_ID icehalo::Logger::GetInstance()->EnableThreadId(false)

#endif  // SRC_UTIL_LOG_H_
