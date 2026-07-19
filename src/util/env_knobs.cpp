#include "util/env_knobs.hpp"

#include <cstdlib>
#include <mutex>

#include "util/logger.hpp"

// Implementation note: each knob owns a std::once_flag so its "applied" log line
// fires exactly once per process even under concurrent first reads (the GPU
// route reads several of these from worker/encode threads). The parse + return
// value are recomputed on every call; only the log is gated.
namespace lumice::env {

namespace {

// Shared "set to a non-empty, non-zero string" predicate for the boolean
// disable_* knobs, matching the historical `c[0] != '\0' && c[0] != '0'` test.
bool IsTruthy(const char* v) {
  return v != nullptr && v[0] != '\0' && v[0] != '0';
}

}  // namespace

std::optional<std::string> TraceBackendOverride(Logger& logger) {
  const char* raw = std::getenv("LUMICE_TRACE_BACKEND");
  if (raw == nullptr) {
    return std::nullopt;
  }
  std::string value(raw);
  // "" / "legacy" defer to the preference and are not actually overriding, so
  // they do not deserve the loud warning.
  if (!value.empty() && value != "legacy") {
    static std::once_flag warned;
    std::call_once(warned, [&logger, &value]() {
      ILOG_WARN(logger,
                "LUMICE_TRACE_BACKEND={} is a debug/CI override; it bypasses the "
                "--backend flag and the API backend preference. Unset it for normal runs.",
                value);
    });
  }
  return value;
}

std::size_t DispatchRayNum(Logger& logger, std::size_t default_val) {
  if (const char* env = std::getenv("LUMICE_DISPATCH_RAY_NUM")) {
    long b = std::atol(env);
    if (b > 0) {
      static std::once_flag logged;
      std::call_once(logged, [&logger, b, default_val]() {
        ILOG_INFO(logger, "env override: LUMICE_DISPATCH_RAY_NUM={} (default {})", b, default_val);
      });
      return static_cast<std::size_t>(b);
    }
  }
  return default_val;
}

std::size_t GeomClock(Logger& logger, std::size_t default_val) {
  if (const char* env = std::getenv("LUMICE_GEOM_CLOCK")) {
    long b = std::atol(env);
    if (b > 0) {
      static std::once_flag logged;
      std::call_once(logged, [&logger, b, default_val]() {
        ILOG_INFO(logger, "env override: LUMICE_GEOM_CLOCK={} rays/shape (default {})", b, default_val);
      });
      return static_cast<std::size_t>(b);
    }
  }
  return default_val;
}

std::size_t GpuGeomClock(Logger& logger, std::size_t default_val) {
  if (const char* env = std::getenv("LUMICE_GPU_GEOM_CLOCK")) {
    long b = std::atol(env);
    if (b > 0) {
      static std::once_flag logged;
      std::call_once(logged, [&logger, b, default_val]() {
        ILOG_INFO(logger, "env override: LUMICE_GPU_GEOM_CLOCK={} rays/shape (default {})", b, default_val);
      });
      return static_cast<std::size_t>(b);
    }
  }
  return default_val;
}

std::uint32_t XyzDrainBatches(Logger& logger, std::uint32_t default_val) {
  if (const char* env = std::getenv("LUMICE_XYZ_DRAIN_BATCHES")) {
    long b = std::atol(env);
    if (b > 0) {
      static std::once_flag logged;
      std::call_once(logged, [&logger, b, default_val]() {
        ILOG_INFO(logger, "env override: LUMICE_XYZ_DRAIN_BATCHES={} (default {})", b, default_val);
      });
      return static_cast<std::uint32_t>(b);
    }
  }
  return default_val;
}

std::size_t CommitRayNum(Logger& logger, std::size_t default_val) {
  std::size_t result = default_val;
  bool overridden = false;
  const char* applied_name = nullptr;
  long applied_value = 0;

  if (const char* env = std::getenv("LUMICE_COMMIT_RAY_NUM")) {
    long b = std::atol(env);
    if (b > 0) {
      result = static_cast<std::size_t>(b);
      overridden = true;
      applied_name = "LUMICE_COMMIT_RAY_NUM";
      applied_value = b;
    }
  }
  if (!overridden) {
    if (const char* env = std::getenv("LUMICE_BATCH_RAY_NUM")) {
      long b = std::atol(env);
      if (b > 0) {
        result = static_cast<std::size_t>(b);
        overridden = true;
        applied_name = "LUMICE_BATCH_RAY_NUM";
        applied_value = b;
      }
    }
  }

  // Deprecation WARN fires once whenever the legacy name is present at all —
  // even when LUMICE_COMMIT_RAY_NUM also wins — so migrating users always get
  // guidance (preserves the pre-refactor server.cpp behavior).
  if (std::getenv("LUMICE_BATCH_RAY_NUM") != nullptr) {
    static std::once_flag deprecation_warned;
    std::call_once(deprecation_warned, [&logger]() {
      ILOG_WARN(logger,
                "LUMICE_BATCH_RAY_NUM is deprecated; migrate to LUMICE_COMMIT_RAY_NUM. "
                "LUMICE_COMMIT_RAY_NUM takes precedence when both are set; otherwise the "
                "legacy value is applied as commit granularity.");
    });
  }
  if (overridden) {
    static std::once_flag logged;
    std::call_once(logged, [&logger, applied_name, applied_value, default_val]() {
      ILOG_INFO(logger, "env override: {}={} (commit granularity, default {})", applied_name, applied_value,
                default_val);
    });
  }
  return result;
}

std::uint32_t WlPoolSize(Logger& logger, std::uint32_t default_val, std::uint32_t max_val) {
  const char* env = std::getenv("LUMICE_WL_POOL_SIZE");
  if (env == nullptr || env[0] == '\0') {
    return default_val;
  }
  long v = std::strtol(env, nullptr, 10);
  if (v <= 0) {
    // Set but unparseable / non-positive: warn once so a typo'd value is visible
    // rather than silently behaving like "unset" (consistency with the other
    // knobs' "non-default value is observable" principle).
    static std::once_flag invalid_warned;
    std::call_once(invalid_warned, [&logger, env, default_val]() {
      ILOG_WARN(logger, "LUMICE_WL_POOL_SIZE='{}' is invalid; using default {}", env, default_val);
    });
    return default_val;
  }
  std::uint32_t resolved = (v > static_cast<long>(max_val)) ? max_val : static_cast<std::uint32_t>(v);
  static std::once_flag logged;
  std::call_once(logged, [&logger, resolved, default_val]() {
    ILOG_INFO(logger, "env override: LUMICE_WL_POOL_SIZE={} (default {})", resolved, default_val);
  });
  return resolved;
}

bool DisableMetalSourceCompile(Logger& logger) {
  bool on = IsTruthy(std::getenv("LUMICE_DISABLE_METAL_SOURCE_COMPILE"));
  if (on) {
    static std::once_flag logged;
    std::call_once(logged, [&logger]() {
      ILOG_INFO(logger, "env override: LUMICE_DISABLE_METAL_SOURCE_COMPILE=1 (metallib-only path)");
    });
  }
  return on;
}

bool DisableDeviceGen(Logger& logger) {
  bool on = IsTruthy(std::getenv("LUMICE_DISABLE_DEVICE_GEN"));
  if (on) {
    static std::once_flag logged;
    std::call_once(
        logged, [&logger]() { ILOG_INFO(logger, "env override: LUMICE_DISABLE_DEVICE_GEN=1 (host root-gen forced)"); });
  }
  return on;
}

}  // namespace lumice::env
