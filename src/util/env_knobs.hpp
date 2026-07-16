#ifndef LUMICE_UTIL_ENV_KNOBS_HPP
#define LUMICE_UTIL_ENV_KNOBS_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace lumice {
class Logger;
}  // namespace lumice

// Central registry for all LUMICE_* environment-variable tuning knobs.
//
// POLICY (doc/env-var-policy.md): this is the ONLY translation unit allowed to
// call std::getenv() for a LUMICE_* name — enforced by scripts/check_policies.py.
// Every knob defined here is registered in doc/env-var-policy.md and logs once
// when it is set to a non-default value, so a knob silently overriding behavior
// on some machine always leaves a greppable line in the log.
//
// Do NOT add a user-facing behavior switch here. If a knob changes the output a
// user sees, it must go through CLI / config / API instead — see the decision
// gate in doc/env-var-policy.md. The only env-driven behavior switch that
// survives here is LUMICE_TRACE_BACKEND, kept strictly as a loud debug/CI
// override on top of the first-class --backend flag / API preference.
namespace lumice::env {

// LUMICE_TRACE_BACKEND — debug/CI override of the trace backend. Returns the
// raw value when the variable is set (including "" and "legacy", which the
// callers treat as "defer to preference"); std::nullopt when unset. Logs a WARN
// exactly once on the first in-effect read (non-empty, non-"legacy" value):
// it bypasses the --backend flag and the API backend preference and is not a
// normal-run configuration path.
std::optional<std::string> TraceBackendOverride(Logger& logger);

// LUMICE_DISPATCH_RAY_NUM — per-SimBatch dispatch granularity (GPU). Returns the
// override when set to a positive integer, else default_val. INFO once if applied.
std::size_t DispatchRayNum(Logger& logger, std::size_t default_val);

// LUMICE_GEOM_CLOCK — experiment knob: how many rays share one
// sampled crystal shape on the legacy CPU path (its geometry clock). This is the
// D/K ratio directly: 1 = a fresh shape per ray (the unconstrained optimum, used
// to build the calibration oracle), 32 = the shipped default. Returns the
// override when positive, else default_val. INFO once if applied.
//
// KNOWN UPPER BOUND — safe range is [1, 64]; >= 128 CORRUPTS THE HEAP.
// Measured (3 reps each): 32 -> 0 0 0, 64 -> 0 0 0,
// 128 -> 139 134 139, 256 -> 139 0 134 (139=SIGSEGV, 134=SIGABRT, intermittent).
// Mechanism: the threshold is exactly the SimBatch size (server.cpp
// kDefaultRayNum=128). SimulateOneWavelength receives that as ray_num and sizes
// its ray buffers to ray_num*2 = 256, while a sub-batch's working set is
// ~curr_ray_num*2 == geom_clock*2 -- so 64 needs 128 and fits, 128 needs 256 and
// sits exactly at capacity, spilling intermittently. This is deliberately NOT
// clamped: a silent clamp would make a sweep quietly not do what was asked. The
// geometry clock is entangled with the ray-buffer capacity model and that must be
// fixed before it can become a tunable quantity; unreachable in production since
// the default is the hard-coded constant.
std::size_t GeomClock(Logger& logger, std::size_t default_val);

// LUMICE_COMMIT_RAY_NUM (with legacy LUMICE_BATCH_RAY_NUM fallback) — consumer
// commit granularity. COMMIT takes precedence; BATCH is honored as a fallback.
// Returns the override when positive, else default_val. INFO once if applied; a
// deprecation WARN once if the legacy LUMICE_BATCH_RAY_NUM name is set at all.
std::size_t CommitRayNum(Logger& logger, std::size_t default_val);

// LUMICE_XYZ_DRAIN_BATCHES — scrum-312 third-clock drain cadence cap. Max number
// of device-fused XYZ batches accumulated on-device before a forced drain
// (readback+reset), bounding within-window float32 accumulation + latency during
// continuous bursts (CLI). GUI display cadence is driven mainly by producer-pause
// flush; this is the burst-internal upper bound. Returns the override when set to
// a positive integer, else default_val. INFO once if applied.
std::uint32_t XyzDrainBatches(Logger& logger, std::uint32_t default_val);

// LUMICE_WL_POOL_SIZE — Metal per-ray wavelength pool size. Clamped to
// [1, max_val]; returns default_val when unset/invalid. INFO once if applied.
std::uint32_t WlPoolSize(Logger& logger, std::uint32_t default_val, std::uint32_t max_val);

// LUMICE_DISABLE_METAL_SOURCE_COMPILE — suppress the runtime MSL source-compile
// fallback (metallib-only path). True when set to a non-empty, non-"0" value.
// INFO once when enabled.
bool DisableMetalSourceCompile(Logger& logger);

// LUMICE_DISABLE_DEVICE_GEN — force host root generation instead of device-gen.
// True when set to a non-empty, non-"0" value. INFO once when enabled.
bool DisableDeviceGen(Logger& logger);

}  // namespace lumice::env

#endif  // LUMICE_UTIL_ENV_KNOBS_HPP
