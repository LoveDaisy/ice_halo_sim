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
  std::printf("%s\n", message.text.c_str());
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
  std::fprintf(stderr, "%s\n", message.text.c_str());
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


Logger::Logger() {
  LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kVerbose, LogLevel::kDebug, LogLevel::kInfo });
  LogFilterPtr stderr_filter = LogFilter::MakeLevelFilter({ LogLevel::kWarning, LogLevel::kError, LogLevel::kFatal });
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
