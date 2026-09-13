#pragma once

// A capture of the shared logger's output for a test to count lines in, shared by the
// ray-allocation binding tests on both scene publishers (CommitConfig in
// test_ray_allocation_online_commit.cpp, StartRaypathAnalysis in
// test_ray_allocation_online_analysis.cpp). Those tests assert on the OUTCOME of the
// server's bind decision — cold start / kept / none — read off the log line each
// branch emits, which is the one channel the live server exposes it on without a
// test reaching into a private member.
//
// The sink hands its text out under the SAME mutex the writes take. An
// ostream_sink_mt read through `oss.str()` from the test thread races the worker
// threads that keep logging while the run is live (the server's own lines land on
// those threads), and that race read as an EMPTY capture once in CI — a truncated
// copy of a string being reallocated underneath it.

#include <spdlog/sinks/base_sink.h>

#include <memory>
#include <mutex>
#include <string>

#include "util/logger.hpp"

namespace lumice::test {

class CaptureSink : public spdlog::sinks::base_sink<std::mutex> {
 public:
  std::string Text() {
    std::lock_guard<std::mutex> lock(mutex_);
    return text_;
  }
  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    text_.clear();
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    text_.append(formatted.data(), formatted.size());
  }
  void flush_() override {}

 private:
  std::string text_;
};

// RAII: attached to the shared sink for the capture's lifetime.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<CaptureSink>()) { GetSharedSink()->add_sink(sink_); }
  ~LogCapture() { GetSharedSink()->remove_sink(sink_); }
  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return sink_->Text(); }
  void Clear() { sink_->Clear(); }

 private:
  std::shared_ptr<CaptureSink> sink_;
};

// Occurrences of `needle` in `haystack`.
inline size_t CountOccurrences(const std::string& haystack, const char* needle) {
  size_t n = 0;
  for (size_t pos = haystack.find(needle); pos != std::string::npos; pos = haystack.find(needle, pos + 1)) {
    n++;
  }
  return n;
}

}  // namespace lumice::test
