#ifndef LUMICE_UTIL_FATAL_HPP
#define LUMICE_UTIL_FATAL_HPP

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace lumice {

// Unrecoverable-invariant trap: write a diagnostic to stderr, then abort.
//
// POLICY: this header is the single owner of the "print and die" idiom, and one
// of only two places under src/ allowed to write to a stdio stream directly (the
// other is main.cpp, whose stdout IS the CLI's product output). Enforced by the
// no-bare-print rule in scripts/check_policies.py. Everything that is not a
// pre-abort trap must go through the logger (ILOG_* / GUI_LOG_*).
//
// Deliberately NOT routed through spdlog, for two reasons:
//   - stderr is unbuffered, so the message is guaranteed to land before
//     std::abort() tears the process down. The shared dist_sink's file sink is
//     not flushed per message, so a log line emitted here could be lost exactly
//     when it matters most.
//   - the caller is on a corrupt-state path by definition. Depending on the
//     logger's allocation, formatting and locking to report that corruption is
//     how a diagnostic turns into a second, more confusing crash.
//
// `fmt` takes a printf-style format string; a "FATAL: " prefix and a trailing
// newline are added here, so call sites pass the bare message.
#if defined(__GNUC__) || defined(__clang__)
#define LUMICE_PRINTF_FORMAT(fmt_idx, first_arg_idx) __attribute__((format(printf, fmt_idx, first_arg_idx)))
#else
#define LUMICE_PRINTF_FORMAT(fmt_idx, first_arg_idx)
#endif

[[noreturn]] inline void FatalAbort(const char* fmt, ...) LUMICE_PRINTF_FORMAT(1, 2);

[[noreturn]] inline void FatalAbort(const char* fmt, ...) {
  std::va_list args;  // NOLINT(cppcoreguidelines-init-variables) — initialized by va_start
  va_start(args, fmt);
  std::fprintf(stderr, "FATAL: ");
  std::vfprintf(stderr, fmt, args);
  std::fprintf(stderr, "\n");
  va_end(args);
  std::fflush(stderr);
  std::abort();
}

}  // namespace lumice

#endif  // LUMICE_UTIL_FATAL_HPP
