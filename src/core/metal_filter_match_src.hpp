// MSL helper + test-kernel source strings for device filter MATCH.
//
// scrum-267 task-msl-filter-match-port (plan D3). These strings are the
// "device counterpart" of FilterSpec::Check / RaypathOrbit::Contains; the
// parity harness compiles `kFilterMatchHelperSrc + kFilterMatchTestKernelSrc`
// into a Metal library and validates bit-exactness vs the host. Sub-task 2
// will additionally splice `kFilterMatchHelperSrc` in front of `kKernelSrc`
// to wire the gate into the production trace kernel — `kFilterMatchHelperSrc`
// declares everything `static inline` so the symbols do not collide with the
// production kernel's namespace.

#ifndef SRC_CORE_METAL_FILTER_MATCH_SRC_H_
#define SRC_CORE_METAL_FILTER_MATCH_SRC_H_

#if defined(__APPLE__)

namespace lumice {

// `static inline` helpers shared by the parity test kernel AND the future
// production trace kernel splice. No I/O / kernel entry points here.
extern const char* const kFilterMatchHelperSrc;

// Self-contained kernel entry that exercises `DeviceFilterMatch` once per
// thread. Used only by the parity harness — not spliced into production.
extern const char* const kFilterMatchTestKernelSrc;

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // SRC_CORE_METAL_FILTER_MATCH_SRC_H_
