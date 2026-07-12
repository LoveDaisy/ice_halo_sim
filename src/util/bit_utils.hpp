#pragma once

#include <cstdint>

namespace lumice {

// Portable constexpr population count for a 64-bit unsigned value.
//
// Why SWAR (and not __builtin_popcountll / MSVC __popcnt64):
//   - __builtin_popcountll is a GCC/Clang extension; MSVC reports C3861. This
//     helper exists specifically to prevent that class of regression (PR#182).
//   - MSVC __popcnt64 requires SSE4.2 (POPCNT) at runtime and is not constexpr,
//     so it would need per-compiler #ifdef and would still be unusable in
//     constant-evaluated contexts.
//   - Standard 64-bit SWAR (4 mask-shift-add groups) has no CPU-feature
//     dependency, is constexpr-friendly under C++17, and is fast enough for
//     the cold call sites where PopCount is used.
//
// Return type is `int` to match the C++20 std::popcount signature; callers
// comparing against unsigned counters (size_t, etc.) must cast at the call
// site. On C++20 migration, this whole file collapses to a one-line
// #include <bit> + `constexpr int PopCount(uint64_t x) { return std::popcount(x); }`.
constexpr int PopCount(uint64_t x) {
  x = x - ((x >> 1) & 0x5555555555555555ULL);
  x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
  x = (x + (x >> 4)) & 0x0f0f0f0f0f0f0f0fULL;
  return static_cast<int>((x * 0x0101010101010101ULL) >> 56);
}

}  // namespace lumice
