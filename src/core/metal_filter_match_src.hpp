// MSL device filter match — helper source string retired (task-#283).
//
// Historical context: this header used to declare
// `extern const char* const kFilterMatchHelperSrc`, a verbatim MSL string
// that was prepended to `kKernelSrc` and handed to `newLibraryWithSource` at
// runtime. As of task-metal-build-time-metallib the entire MSL body (helper
// + trace kernel) lives in a single source-of-truth file
// (`src/core/metal/lumice_trace.metal`) which CMake precompiles into a
// .metallib, embedded into the binary and loaded via `newLibraryWithData`.
// See `metal_trace_backend.mm` for the load path.
//
// What remains here is the host-side mirror of the MSL hop-count budget —
// `kDevFilterMatchRecCap` — kept so the static_assert in
// `metal_trace_backend.mm` can still trap a future drift between the trace
// kernel's `kRecCap` and the filter-match helper's `kDevRecCap` (both MSL
// constants live inside lumice_trace.metal; the host-visible mirror is the
// only cross-language tie-point).

#ifndef SRC_CORE_METAL_FILTER_MATCH_SRC_H_
#define SRC_CORE_METAL_FILTER_MATCH_SRC_H_

#if defined(__APPLE__)

namespace lumice {

// Trace-kernel splice contract: kFilterMatchHelperSrc declares
// `constant uint kDevRecCap = 64` and the production trace kernel declares
// `constant uint kRecCap = 64`. Both names refer to the same hop-count budget
// — the host-visible mirror below is checked against the trace kernel's
// `kTraceKernelRecCap` (in metal_trace_backend.mm) via static_assert so a
// future bump cannot drift the two MSL constants apart silently.
constexpr int kDevFilterMatchRecCap = 64;

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // SRC_CORE_METAL_FILTER_MATCH_SRC_H_
