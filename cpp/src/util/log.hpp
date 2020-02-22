#ifndef SRC_UTIL_LOG_H_
#define SRC_UTIL_LOG_H_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
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
  std::string message;
  std::thread::id thread_id;
};


class LogDestination;
using LogDestPtrU = std::unique_ptr<LogDestination>;
using LogDestPtr = std::shared_ptr<LogDestination>;

class LogDestination {
 public:
  virtual ~LogDestination() = default;
  virtual void Write(const LogMessage& message) = 0;
};


class LogStdOutDest : public LogDestination {
 public:
  void Write(const LogMessage& message) override;

  static LogDestPtr GetInstance();

 private:
  LogStdOutDest() = default;
};


class LogStdErrDest : public LogDestination {
 public:
  void Write(const LogMessage& message) override;

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
  template <typename... LogLevel>
  static LogFilterPtrU MakeLevelFilter(LogLevel... levels);
};

LogFilterPtr operator&&(LogFilterPtr a, LogFilterPtr b);
LogFilterPtr operator||(LogFilterPtr a, LogFilterPtr b);


class LogTagFilter : public LogFilter {
 public:
  explicit LogTagFilter(const char* tag) : tag_(tag) {}

  bool Filter(const LogMessage& message) override;

 private:
  std::string tag_;
};


class LogLevelFilter : public LogFilter {
 public:
  template <typename... LogLevel>
  explicit LogLevelFilter(LogLevel... levels) : levels_{ levels... } {}

  bool Filter(const LogMessage& message) override;

 private:
  std::vector<LogLevel> levels_;
};

template <typename... LogLevel>
LogFilterPtrU LogFilter::MakeLevelFilter(LogLevel... levels) {
  LogFilterPtrU filter{ new LogLevelFilter(levels...) };
  return filter;
}


class LogComplexFilter : public LogFilter {
 public:
  LogComplexFilter() = default;

  void Reset();
  void And(LogFilterPtr other);
  void Or(LogFilterPtr other);

  bool Filter(const LogMessage& message) override;

 private:
  std::vector<std::vector<LogFilterPtr>> filters_;
};


#define DECL_LOG_WRAPPER(name, level)                              \
  template <typename... Args>                                      \
  void name(Args&&... args) {                                      \
    EmitLog<LogLevel::level>(std::forward<Args>(args)...);         \
  }                                                                \
  template <typename... Args>                                      \
  void name##Tag(const char* tag, Args&&... args) {                \
    EmitTagLog<LogLevel::level>(tag, std::forward<Args>(args)...); \
  }

class Logger {
 public:
  template <LogLevel level>
  void EmitLog(const char* file, int line, const char* fmt, ...) {
    char buf[kMaxMessageLength];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, kMaxMessageLength, fmt, args);
    va_end(args);

    LogMessage msg{ level, std::chrono::system_clock::now(), line, file, "", buf, std::this_thread::get_id() };
    for (auto& kv : filter_dest_list_) {
      if (kv.first->Filter(msg)) {
        kv.second->Write(msg);
      }
    }
  }

  template <LogLevel level>
  void EmitTagLog(const char* file, int line, const char* tag, const char* fmt, ...) {
    char buf[kMaxMessageLength];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, kMaxMessageLength, fmt, args);
    va_end(args);

    LogMessage msg{ level, std::chrono::system_clock::now(), line, file, tag, buf, std::this_thread::get_id() };
    for (auto& kv : filter_dest_list_) {
      if (kv.first->Filter(msg)) {
        kv.second->Write(msg);
      }
    }
  }

  // Convenience wrappers for log.
  DECL_LOG_WRAPPER(Verbose, kVerbose)
  DECL_LOG_WRAPPER(Debug, kDebug)
  DECL_LOG_WRAPPER(Info, kInfo)
  DECL_LOG_WRAPPER(Warning, kWarning)
  DECL_LOG_WRAPPER(Error, kError)
  DECL_LOG_WRAPPER(Fatal, kFatal)

  static Logger* GetInstance();

  static constexpr size_t kMaxMessageLength = 1024;

 private:
  Logger();

  std::vector<std::pair<LogFilterPtr, LogDestPtr>> filter_dest_list_;
};

#undef DECL_LOG_WRAPPER
#undef DECL_LOG_TAG_WRAPPER

}  // namespace icehalo

// Convenience macros for log
#define LOG_DEBUG(fmt, ...) icehalo::Logger::GetInstance()->Debug(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_VERBOSE(fmt, ...) icehalo::Logger::GetInstance()->Verbose(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) icehalo::Logger::GetInstance()->Info(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) icehalo::Logger::GetInstance()->Warning(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) icehalo::Logger::GetInstance()->Error(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) icehalo::Logger::GetInstance()->Fatal(__FILE__, __LINE__, (fmt), ##__VA_ARGS__)
#define LOG_TAG_DEBUG(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->DebugTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)
#define LOG_TAG_VERBOSE(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->VerboseTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)
#define LOG_TAG_INFO(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->InfoTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)
#define LOG_TAG_WARNING(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->WarningTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)
#define LOG_TAG_ERROR(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->ErrorTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)
#define LOG_TAG_FATAL(tag, fmt, ...) \
  icehalo::Logger::GetInstance()->FatalTag(__FILE__, __LINE__, (tag), (fmt), ##__VA_ARGS__)

#endif  // SRC_UTIL_LOG_H_
