#ifndef INCLUDE_SERVER_H_
#define INCLUDE_SERVER_H_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "core/backend/backend_kind.hpp"
#include "util/logger.hpp"


namespace lumice {

// =============== Error ===============
/**
 * @brief Error code enumeration
 */
enum class ErrorCode {
  kSuccess,         ///< Success (no error)
  kInvalidJson,     ///< JSON format error
  kInvalidConfig,   ///< Configuration content error
  kMissingField,    ///< Missing required field
  kInvalidValue,    ///< Invalid field value
  kServerNotReady,  ///< Server not ready
  kServerError,     ///< Server internal error
};

/**
 * @brief Error structure for error handling
 * @details Contains error code, message, and optional field name
 */
struct Error {
  ErrorCode code;       ///< Error code
  std::string message;  ///< Error message
  std::string field;    ///< Field name where error occurred (optional)

  /**
   * @brief Default constructor: success state
   */
  Error() : code(ErrorCode::kSuccess) {}

  /**
   * @brief Constructor with error code and message
   * @param c Error code
   * @param msg Error message
   * @param f Field name (optional)
   */
  Error(ErrorCode c, const std::string& msg, const std::string& f = "") : code(c), message(msg), field(f) {}

  /**
   * @brief Check if operation was successful
   * @return true if success, false if error
   */
  bool IsSuccess() const { return code == ErrorCode::kSuccess; }

  /**
   * @brief Check if there was an error
   * @return true if error, false if success
   */
  bool IsError() const { return code != ErrorCode::kSuccess; }

  /**
   * @brief Boolean conversion operator
   * @return true if error, false if success
   * @note Allows usage: if (err) { ... }
   */
  explicit operator bool() const { return IsError(); }

  // Factory methods for common error types
  static Error Success() { return Error(); }

  static Error InvalidJson(const std::string& msg) { return Error(ErrorCode::kInvalidJson, msg); }

  static Error InvalidConfig(const std::string& msg) { return Error(ErrorCode::kInvalidConfig, msg); }

  static Error MissingField(const std::string& field) {
    return Error(ErrorCode::kMissingField, "Missing required field: " + field, field);
  }

  static Error InvalidValue(const std::string& field, const std::string& msg) {
    return Error(ErrorCode::kInvalidValue, msg, field);
  }

  static Error ServerNotReady(const std::string& msg = "Server is not ready") {
    return Error(ErrorCode::kServerNotReady, msg);
  }

  static Error ServerError(const std::string& msg) { return Error(ErrorCode::kServerError, msg); }
};

// =============== Result ===============
struct NoneResult {};

/**
 * @brief Render result containing rendered image data
 * @details Contains metadata and pixel data for a rendered image
 */
struct RenderResult {
  int renderer_id_;  ///< Renderer ID
  int img_width_;    ///< Image width in pixels
  int img_height_;   ///< Image height in pixels

  /**
   * @brief Image data buffer (read-only)
   * @details
   * - Format: RGB, 3 bytes per pixel, row-major order
   * - Size: img_width_ * img_height_ * 3 bytes
   * - Ownership: Managed internally by Server, user should not free
   * - Lifetime: Valid until next call to GetRenderResults() or CommitConfig()
   * - Usage: For long-term storage, use CopyBuffer() to copy the data
   *
   * @warning Do not access this pointer outside its lifetime, undefined behavior may occur
   */
  const uint8_t* img_buffer_;

  /**
   * @brief Copy image data to a new vector
   * @return Vector containing image data (RGB format)
   * @details Use this method when you need to store the image data for an extended period
   */
  std::vector<uint8_t> CopyBuffer() const {
    if (!img_buffer_ || img_width_ <= 0 || img_height_ <= 0) {
      return std::vector<uint8_t>();
    }
    size_t size = static_cast<size_t>(img_width_) * static_cast<size_t>(img_height_) * 3;
    return std::vector<uint8_t>(img_buffer_, img_buffer_ + size);
  }
};

struct RawXyzResult {
  int renderer_id_;
  int img_width_;
  int img_height_;
  const float* xyz_buffer_;  // Points to snapshot_xyz_ (not color-converted)
  float snapshot_intensity_;
  float intensity_factor_;
  bool has_valid_data_ = false;       // True after first ConsumeData; reset on Stop
  uint64_t snapshot_generation_ = 0;  // Increments on each new snapshot
  int effective_pixels_ = 0;          // Non-zero pixel count for adaptive normalization
  // Lifecycle epoch (committed_epoch_ at snapshot time). Stamped by
  // GetRawXyzResults; consumed by the GUI display-keying in 1.5. See
  // doc/gui-preview-lifecycle-architecture.md §4/§5.
  uint64_t epoch_ = 0;
};

struct StatsResult {
  size_t ray_seg_num_;
  size_t sim_ray_num_;
  size_t crystal_num_;
};

using Result = std::variant<NoneResult, RenderResult, StatsResult>;


// =============== GPU route query ===============
/**
 * @brief Would a server built with @p preferred_backend take the GPU single-engine route?
 * @details Single source of truth for the routing decision (env `LUMICE_TRACE_BACKEND`
 *          override wins over @p preferred_backend, CUDA gated on device availability),
 *          mirroring CreateBackend. `ServerImpl`'s constructor uses the same function to
 *          decide `worker_count` (GPU route => 1). Callers outside the server (e.g. the CLI
 *          `--benchmark` dual-pass, which must skip the meaningless "single" warmup pass for
 *          the single-engine GPU route) should query this rather than re-deriving the logic.
 */
bool ResolveGpuRoute(BackendKind preferred_backend, Logger& logger);

// =============== Server Status ===============
/**
 * @brief Server status enumeration
 */
enum class ServerStatus {
  kIdle,     ///< Idle (completed or not started)
  kRunning,  ///< Running (processing)
  kError     ///< Error state
};

// =============== Simulation Lifecycle ===============
/**
 * @brief Explicit single-source simulation lifecycle truth.
 * @details Replaces the historical side-signal disambiguation (bare kIdle +
 *          has_ever_consumed_ + stats>0). Derived in one place
 *          (ServerImpl::GetSimLifecycle); ServerStatus / QueryServerState become
 *          projections of it. See doc/gui-preview-lifecycle-architecture.md §4/§5.
 *          - kCompleted = a finite run drained clean (includes the zero-output /
 *            all-filter-rejected convergence, capi-lifecycle §3.6).
 *          - kIdle      = never run, or reset (post-Stop) with no data consumed.
 *          - kRunning   = pending work / workers active. Infinite runs stay here
 *            forever (never kCompleted); Stop returns them to kIdle.
 */
enum class SimLifecycle {
  kIdle,       ///< Not run, or reset and not yet consumed (incl. after Stop)
  kRunning,    ///< Pending work / workers active
  kCompleted,  ///< Finite run drained clean (incl. zero-output convergence)
};

// =============== Server ===============
class ServerImpl;

/**
 * @brief Server interface for ice halo simulation
 * @details The Server class provides a high-level interface for running ice halo simulations.
 *          It uses a server-consumer architecture with multi-threaded processing.
 *          The server starts running immediately after construction.
 */
class Server {
 public:
  /**
   * @brief Construct a new Server with default worker count
   * @note The server starts running immediately after construction
   */
  Server();

  /**
   * @brief Construct a new Server
   * @param num_workers CPU route only: worker count (0 = PhysicalCoreCount()).
   *        Ignored on the GPU/Metal route, which is always a single engine
   *        (task-268.7). The route is fixed at construction (see preferred_backend).
   * @param sim_seed Deterministic RNG seed. 0 = random; non-zero clamps CPU route to 1 worker.
   * @param preferred_backend BackendKind::kCpu (multi-worker), kMetal (single engine)
   *        or kCuda (future). An env LUMICE_TRACE_BACKEND override takes precedence.
   *        The GUI reconstructs the server when the backend selection changes.
   * @note The server starts running immediately after construction
   */
  explicit Server(int num_workers, uint32_t sim_seed = 0, BackendKind preferred_backend = BackendKind::kCpu);

  /**
   * @brief Commit configuration from string
   * @param config_str JSON configuration string
   * @return Error object indicating success or failure
   * @note The configuration format should follow V3 configuration schema
   * @see configuration.md for configuration format details
   * @example
   *   auto err = server.CommitConfig(config_str);
   *   if (err) {  // Check if error occurred
   *     std::cerr << "Error: " << err.message << std::endl;
   *     return;
   *   }
   */
  Error CommitConfig(const std::string& config_str);

  /**
   * @brief Commit configuration from parsed JSON object (skips string parse overhead)
   * @param config_json Parsed JSON object
   * @return Error object indicating success or failure
   */
  Error CommitConfig(const nlohmann::json& config_json, bool* out_reused = nullptr);

  /**
   * @brief Commit configuration from file
   * @param filename Path to JSON configuration file
   * @return Error object indicating success or failure
   * @note The configuration format should follow V3 configuration schema
   * @see configuration.md for configuration format details
   * @example
   *   auto err = server.CommitConfigFromFile("config.json");
   *   if (err) {
   *     std::cerr << "Error: " << err.message << std::endl;
   *     return;
   *   }
   */
  Error CommitConfigFromFile(const std::filesystem::path& filename);

  /**
   * @brief Get all render results
   * @return Vector of RenderResult objects. Returns empty vector if no render results available.
   * @note This is a non-blocking call. It returns immediately even if processing is ongoing.
   * @note Only returns RenderResult entries, filters out other result types
   */
  std::vector<RenderResult> GetRenderResults();

  /**
   * @brief Get composited per-raypath colored results (task-336.4)
   * @return Vector of RenderResult objects, one per colored renderer. Empty when
   *         no `raypath_color` is configured (mono-only path). Reuses RenderResult
   *         since a composite is just an RGB uint8 W*H*3 image; img_buffer_ points
   *         into server-owned storage with the same lifetime contract as
   *         GetRenderResults (valid until the next GetCompositeResults/CommitConfig).
   */
  std::vector<RenderResult> GetCompositeResults();

  /**
   * @brief Get raw XYZ results (unconverted float data for GPU-side processing)
   * @return Vector of RawXyzResult. Empty if no render consumers.
   */
  std::vector<RawXyzResult> GetRawXyzResults();

  /**
   * @brief Get statistics result
   * @return Optional StatsResult. Returns std::nullopt if no statistics result available.
   * @note This is a non-blocking call. It returns immediately even if processing is ongoing.
   * @note Only returns StatsResult if available, otherwise returns std::nullopt
   */
  std::optional<StatsResult> GetStatsResult();

  /**
   * @brief Get cached statistics without triggering DoSnapshot
   * @return Optional StatsResult from the most recent snapshot. Returns std::nullopt if no data yet.
   * @note Unlike GetStatsResult(), this does NOT call DoSnapshot/PostSnapshot.
   *       The cache is updated by GetRawXyzResults() when snapshot_dirty_ is true.
   */
  std::optional<StatsResult> GetCachedStatsResult();

  /**
   * @brief Cheap O(1) live accumulated sim ray count (no snapshot / no render).
   * @return Running StatsConsumer ray count; 0 if none. For progress polling
   *         (e.g. the --benchmark drain loop) that needs sim_ray_num every
   *         iteration but not a rendered image. See task-317.
   */
  size_t GetLiveSimRayCount();

  /**
   * @brief Stop the server
   * @note Stops processing but keeps the server alive. Can be restarted by committing new config.
   * @note No Run() method exists because server starts running immediately after construction.
   */
  void Stop();

  /**
   * @brief Terminate the server
   * @note Stops processing and prepares for destruction. Server cannot be used after termination.
   */
  void Terminate();

  void SetLogLevel(LogLevel level);

  /**
   * @brief Set preferred trace backend for this server.
   * @param backend 0 = CPU (default), 1 = Metal (Apple-only; silent CPU
   *                fallback elsewhere or when the active config is not
   *                backend-compatible). Mirrors the public C API constants
   *                LUMICE_BACKEND_CPU / LUMICE_BACKEND_METAL.
   * @note Takes effect on the next Simulator::Run() entry (after CommitConfig
   *       restart). env-var LUMICE_TRACE_BACKEND, when set, overrides this.
   */
  void SetPreferredBackend(BackendKind backend);

  /**
   * @brief Get server status
   * @return Current server status
   */
  ServerStatus GetStatus() const;

  /**
   * @brief Get the explicit simulation lifecycle (single-source truth).
   * @return kRunning / kCompleted / kIdle (see SimLifecycle).
   * @note Authoritative derivation; GetStatus()/QueryServerState are projections.
   */
  SimLifecycle GetSimLifecycle() const;

  /**
   * @brief Current lifecycle epoch (monotonic generation counter).
   * @return committed_epoch_; ++ on each reset-causing CommitConfig. 0 before any
   *         successful commit. Read back after a synchronous commit to learn the
   *         just-minted epoch (no commit-signature change; see plan §2 decision 3).
   */
  uint64_t CommittedEpoch() const;

  /**
   * @brief Check if server is idle
   * @return true if server is idle (no processing), false if processing
   * @note Convenience method, equivalent to GetStatus() == ServerStatus::kIdle
   */
  bool IsIdle() const;

 private:
  std::shared_ptr<ServerImpl> impl_;
};

}  // namespace lumice

#endif  // INCLUDE_SERVER_H_
