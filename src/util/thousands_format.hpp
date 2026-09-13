#pragma once

#include <cstdint>
#include <string>

namespace lumice {

// An unsigned integer as decimal text with a comma every three digits from the right:
// 0 -> "0", 999 -> "999", 1000 -> "1,000", 1234567 -> "1,234,567".
//
// Written by hand rather than through std::locale / imbue on purpose. The locale route reads the
// process's or the OS's number formatting, which differs across platforms and across machines of
// one platform (some locales group by two, some use a space or a point, some define no grouping
// at all), so the same count would print three ways in three places. A ray counter is a progress
// indicator: the user watches the low digits move, so the digits are kept in full — no k / M / B
// abbreviation — and the grouping is only there to make a nine-digit number readable at a glance.
//
// Lives in src/util/ because it carries no simulation or configuration semantics — the shape
// AGENTS.md admits there, so both the GUI (its only consumer today) and the CLI can read it
// without a second copy.
inline std::string FormatThousands(std::uint64_t value) {
  std::string digits = std::to_string(value);
  std::string out;
  out.reserve(digits.size() + digits.size() / 3);
  const std::size_t n = digits.size();
  for (std::size_t i = 0; i < n; ++i) {
    // A comma before every digit whose distance to the end is a positive multiple of three.
    if (i > 0 && (n - i) % 3 == 0) {
      out.push_back(',');
    }
    out.push_back(digits[i]);
  }
  return out;
}

}  // namespace lumice
