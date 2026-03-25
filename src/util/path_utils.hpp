#pragma once

#include <filesystem>
#include <string>

namespace lumice {

// Construct a filesystem::path from a UTF-8 encoded string.
// On Windows, this correctly converts to the internal UTF-16 representation.
// Use this instead of path(string), which interprets string as ANSI codepage on Windows.
// Note: u8path is deprecated in C++20; if migrating, replace with path(u8string).
inline std::filesystem::path PathFromU8(const std::string& utf8) {
  return std::filesystem::u8path(utf8);  // NOLINT(clang-diagnostic-deprecated-declarations)
}

// Convert a filesystem::path to a UTF-8 encoded string.
// On Windows, path::string() returns ANSI codepage; always use this for cross-platform UTF-8.
inline std::string PathToU8(const std::filesystem::path& p) {
  return p.u8string();
}

}  // namespace lumice
