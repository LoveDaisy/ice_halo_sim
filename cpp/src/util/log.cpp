#include "include/log.hpp"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <unordered_set>
#include <utility>

namespace icehalo {

void LogStdOutDest::Write(const LogMessage& message, LogFormatterPtr& formatter) {
  if (!formatter) {
    const char* level_str = "";
    switch (message.level) {
      case LogLevel::kVerbose:
        level_str = "[VERBOSE]";
        break;
      case LogLevel::kDebug:
        level_str = "[DEBUG]";
        break;
      case LogLevel::kInfo:
        level_str = "[INFO]";
        break;
      default:
        break;
    }
    std::printf("%s(%016zx) %s\n", level_str, std::hash<std::thread::id>{}(message.thread_id), message.text.c_str());
  } else {
    std::printf("%s\n", formatter->Format(message));
  }
}


LogDestPtr LogStdOutDest::GetInstance() {
  LogDestPtr dest{ new LogStdOutDest };
  return dest;
}


void LogStdErrDest::Write(const LogMessage& message, LogFormatterPtr& formatter) {
  if (!formatter) {
    const char* level_str = "";
    switch (message.level) {
      case LogLevel::kWarning:
        level_str = "[WARNING]";
        break;
      case LogLevel::kError:
        level_str = "[ERROR]";
        break;
      case LogLevel::kFatal:
        level_str = "[FATAL]";
        break;
      default:
        break;
    }
    std::fprintf(stderr, "%s(%016zx) %s\n", level_str, std::hash<std::thread::id>{}(message.thread_id),
                 message.text.c_str());
  } else {
    std::fprintf(stderr, "%s\n", formatter->Format(message));
  }
}


LogDestPtr LogStdErrDest::GetInstance() {
  LogDestPtr dest{ new LogStdErrDest };
  return dest;
}


LogFilterPtrU LogFilter::MakeTagFilter(const char* tag) {
  LogFilterPtrU filter{ new LogTagFilter(tag) };
  return filter;
}


LogFilterPtrU LogFilter::MakeLevelFilter(std::initializer_list<LogLevel> levels) {
  LogFilterPtrU filter{ new LogLevelFilter(levels) };
  return filter;
}


LogFilterPtr operator&&(const LogFilterPtr& a, const LogFilterPtr& b) {
  auto* filter = new LogComplexFilter(a);
  filter->And(b);
  return LogFilterPtr{ filter };
}


LogFilterPtr operator||(const LogFilterPtr& a, const LogFilterPtr& b) {
  auto* filter = new LogComplexFilter(a);
  filter->Or(b);
  return LogFilterPtr{ filter };
}


bool LogTagFilter::Filter(const LogMessage& message) {
  if (tag_.empty()) {
    return true;
  } else {
    return tag_ == message.tag;
  }
}


bool LogLevelFilter::Filter(const LogMessage& message) {
  if (levels_.empty()) {
    return true;
  } else {
    for (const auto& lvl : levels_) {
      if (message.level == lvl) {
        return true;
      }
    }
    return false;
  }
}


LogComplexFilter::LogComplexFilter(LogFilterPtr filter) {
  auto complex_filter = std::dynamic_pointer_cast<LogComplexFilter>(filter);
  if (complex_filter) {
    filters_ = std::move(complex_filter->filters_);
  } else {
    filters_ = { { std::move(filter) } };
  }
}


void LogComplexFilter::Reset() {
  filters_.clear();
}


void LogComplexFilter::And(LogFilterPtr other) {
  auto complex_filter = std::dynamic_pointer_cast<LogComplexFilter>(other);
  if (complex_filter) {
    std::vector<std::vector<LogFilterPtr>> new_filters;
    for (const auto& filter_group1 : complex_filter->filters_) {
      std::vector<std::vector<LogFilterPtr>> curr_filters{ filters_ };
      for (const auto& f : filter_group1) {
        for (auto& filter_group0 : curr_filters) {
          filter_group0.emplace_back(f);
        }
      }
      for (auto& fg : curr_filters) {
        new_filters.emplace_back(std::move(fg));
      }
    }
    filters_.swap(new_filters);
  } else {
    for (auto& filter_group : filters_) {
      filter_group.emplace_back(std::move(other));
    }
  }
}


void LogComplexFilter::Or(LogFilterPtr other) {
  auto complex_filter = std::dynamic_pointer_cast<LogComplexFilter>(other);
  if (complex_filter) {
    for (const auto& filter_group : complex_filter->filters_) {
      filters_.emplace_back(filter_group);
    }
  } else {
    filters_.emplace_back(std::vector<LogFilterPtr>{ std::move(other) });
  }
}


bool LogComplexFilter::Filter(const LogMessage& message) {
  bool result = false;
  for (auto& filter_group : filters_) {
    bool curr_result = true;
    for (auto& f : filter_group) {
      if (!f->Filter(message)) {
        curr_result = false;
        break;
      }
    }
    if (curr_result) {
      result = true;
      break;
    }
  }
  return result;
}


Logger::Logger() : default_formatter_{ std::make_shared<SimpleLogFormatter>() } {
#if defined(DEBUG) || defined(FOR_TEST)
  LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kVerbose, LogLevel::kDebug, LogLevel::kInfo });
#else
  LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kInfo });
#endif
  LogFilterPtr stderr_filter = LogFilter::MakeLevelFilter({ LogLevel::kWarning, LogLevel::kError, LogLevel::kFatal });
  LogDestPtr stdout_dest = LogStdOutDest::GetInstance();
  LogDestPtr stderr_dest = LogStdErrDest::GetInstance();

  filter_dest_list_.emplace_back(std::make_pair(stdout_filter, stdout_dest));
  filter_dest_list_.emplace_back(std::make_pair(stderr_filter, stderr_dest));
}


void Logger::EmitLog(const char* file, int line, LogLevel level, const char* fmt, ...) {
  char buf[kMaxMessageLength];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, kMaxMessageLength, fmt, args);
  va_end(args);

  EmitLog(file, line, level, std::string(buf));
}


void Logger::EmitLog(const char* file, int line, LogLevel level, std::string msg_str) {
  LogMessage msg{
    level, std::chrono::system_clock::now(), line, file, "", std::move(msg_str), std::this_thread::get_id()
  };
  std::unordered_set<LogDestPtr> dest_set;
  for (auto& [filter, dest] : filter_dest_list_) {
    if (filter->Filter(msg) && !dest_set.count(dest)) {
      dest->Write(msg, default_formatter_);
      dest_set.emplace(dest);
    }
  }
}


void Logger::EmitTagLog(const char* file, int line, LogLevel level, const char* tag, const char* fmt, ...) {
  char buf[kMaxMessageLength];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, kMaxMessageLength, fmt, args);
  va_end(args);

  EmitTagLog(file, line, level, tag, std::string(buf));
}


void Logger::EmitTagLog(const char* file, int line, LogLevel level, const char* tag, std::string msg_str) {
  LogMessage msg{
    level, std::chrono::system_clock::now(), line, file, tag, std::move(msg_str), std::this_thread::get_id()
  };
  std::unordered_set<LogDestPtr> dest_set;
  for (auto& [filter, dest] : filter_dest_list_) {
    if (filter->Filter(msg) && !dest_set.count(dest)) {
      dest->Write(msg, default_formatter_);
      dest_set.emplace(dest);
    }
  }
}


void Logger::AddDestination(LogFilterPtr filter, LogDestPtr dest) {
  filter_dest_list_.emplace_back(std::make_pair(std::move(filter), std::move(dest)));
}


void Logger::EnableSeverity(bool enable) {
  default_formatter_->EnableSeverity(enable);
}


void Logger::EnableThreadId(bool enable) {
  default_formatter_->EnableThreadId(enable);
}


Logger* Logger::GetInstance() {
  static Logger logger;
  return &logger;
}


LogFormatter::LogFormatter() : enable_severity_(true), enable_thread_id_(true), buf_{} {}


void LogFormatter::EnableSeverity(bool enable) {
  enable_severity_ = enable;
}


void LogFormatter::EnableThreadId(bool enable) {
  enable_thread_id_ = enable;
}


const char* SimpleLogFormatter::Format(const LogMessage& message) {
  size_t offset = 0;
  const auto tm = std::chrono::system_clock::to_time_t(message.time);
  offset += std::strftime(buf_ + offset, kBufSize - offset, "%T.", std::localtime(&tm));

  auto t_sec = std::chrono::floor<std::chrono::seconds>(message.time);
  std::chrono::duration<double, std::milli> d_ms = message.time - t_sec;
  offset += std::snprintf(buf_ + offset, kBufSize - offset, "%03d", static_cast<int>(d_ms.count()));

  const char* level_str = "";
  if (enable_severity_) {
    switch (message.level) {
      case LogLevel::kDebug:
        level_str = "[DEBUG]";
        break;
      case LogLevel::kVerbose:
        level_str = "[VERBOSE]";
        break;
      case LogLevel::kInfo:
        level_str = "[INFO]";
        break;
      case LogLevel::kWarning:
        level_str = "[WARNING]";
        break;
      case LogLevel::kError:
        level_str = "[ERROR]";
        break;
      case LogLevel::kFatal:
        level_str = "[FATAL]";
        break;
    }
  }
  offset += std::snprintf(buf_ + offset, kBufSize - offset, "%s", level_str);
  if (enable_thread_id_) {
    offset +=
        std::snprintf(buf_ + offset, kBufSize - offset, "(%016zx)", std::hash<std::thread::id>{}(message.thread_id));
  }
  std::snprintf(buf_ + offset, kBufSize - offset, " %s", message.text.c_str());
  return buf_;
}

}  // namespace icehalo
