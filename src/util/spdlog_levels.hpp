#ifndef LUMICE_UTIL_SPDLOG_LEVELS_HPP
#define LUMICE_UTIL_SPDLOG_LEVELS_HPP

#include <spdlog/pattern_formatter.h>

#include <memory>

namespace lumice {

// Custom spdlog flag formatter for our 6-level scheme.
//
// We map 6 application log levels to spdlog's 6 built-in numeric levels:
//   trace(0) < debug(1) < verbose(2) < info(3) < warning(4) < error(5) < off(6)
//
// Use '%*' in the pattern instead of '%L' to get our short level tags:
//   T, D, V, I, W, E
class LumiceLevelFlag : public spdlog::custom_flag_formatter {
 public:
  void format(const spdlog::details::log_msg& msg, const std::tm&, spdlog::memory_buf_t& dest) override {
    static constexpr char kShort[] = {'T', 'D', 'V', 'I', 'W', 'E', 'O'};
    dest.push_back(kShort[static_cast<int>(msg.level)]);
  }

  std::unique_ptr<custom_flag_formatter> clone() const override {
    return std::make_unique<LumiceLevelFlag>();
  }
};

// Create a spdlog formatter with our custom '%*' flag registered.
inline std::unique_ptr<spdlog::formatter> CreateLumiceFormatter(const std::string& pattern) {
  auto f = std::make_unique<spdlog::pattern_formatter>();
  f->add_flag<LumiceLevelFlag>('*').set_pattern(pattern);
  return f;
}

}  // namespace lumice

#endif  // LUMICE_UTIL_SPDLOG_LEVELS_HPP
