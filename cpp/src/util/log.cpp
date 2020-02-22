#include "util/log.hpp"

#include <cstdio>
#include <utility>

namespace icehalo {

void LogStdOutDest::Write(const LogMessage& message) {
  switch (message.level) {
    case LogLevel::kDebug:
      std::printf("[DEBUG] ");
      break;
    case LogLevel::kInfo:
      std::printf("[INFO] ");
      break;
    case LogLevel::kVerbose:
    default:
      break;
  }
  std::printf("%s\n", message.message.c_str());
}


LogDestPtr LogStdOutDest::GetInstance() {
  LogDestPtr dest{ new LogStdOutDest };
  return dest;
}


void LogStdErrDest::Write(const LogMessage& message) {
  switch (message.level) {
    case LogLevel::kWarning:
      std::fprintf(stderr, "[WARNING] ");
      break;
    case LogLevel::kError:
      std::fprintf(stderr, "[ERROR] ");
      break;
    case LogLevel::kFatal:
      std::fprintf(stderr, "[FATAL] ");
      break;
    default:
      break;
  }
  std::fprintf(stderr, "%s\n", message.message.c_str());
}


LogDestPtr LogStdErrDest::GetInstance() {
  LogDestPtr dest{ new LogStdErrDest };
  return dest;
}


LogFilterPtrU LogFilter::MakeTagFilter(const char* tag) {
  LogFilterPtrU filter{ new LogTagFilter(tag) };
  return filter;
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


void LogComplexFilter::Reset() {
  filters_.clear();
}


void LogComplexFilter::And(LogFilterPtr other) {
  for (auto& and_filters : filters_) {
    and_filters.emplace_back(std::move(other));
  }
}


void LogComplexFilter::Or(LogFilterPtr other) {
  filters_.emplace_back(std::vector<LogFilterPtr>{ std::move(other) });
}


bool LogComplexFilter::Filter(const LogMessage& message) {
  bool result = false;
  for (auto& and_filters : filters_) {
    bool curr_result = true;
    for (auto& f : and_filters) {
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


Logger::Logger() {
  LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter(LogLevel::kVerbose, LogLevel::kDebug, LogLevel::kInfo);
  LogFilterPtr stderr_filter = LogFilter::MakeLevelFilter(LogLevel::kWarning, LogLevel::kError, LogLevel::kFatal);
  LogDestPtr stdout_dest = LogStdOutDest::GetInstance();
  LogDestPtr stderr_dest = LogStdErrDest::GetInstance();

  filter_dest_list_.emplace_back(std::make_pair(stdout_filter, stdout_dest));
  filter_dest_list_.emplace_back(std::make_pair(stderr_filter, stderr_dest));
}


Logger* Logger::GetInstance() {
  static Logger logger;
  return &logger;
}

}  // namespace icehalo
