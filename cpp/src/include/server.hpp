#ifndef INCLUDE_SERVER_H_
#define INCLUDE_SERVER_H_

#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>


namespace icehalo {
namespace v3 {

// =============== Error ===============
/**
 * @brief Error code enumeration
 */
enum class ErrorCode {
  kSuccess,        ///< Success (no error)
  kInvalidJson,    ///< JSON format error
  kInvalidConfig,  ///< Configuration content error
  kMissingField,   ///< Missing required field
  kInvalidValue,   ///< Invalid field value
  kServerNotReady, ///< Server not ready
  kServerError,    ///< Server internal error
};

/**
 * @brief Error structure for error handling
 * @details Contains error code, message, and optional field name
 */
struct Error {
  ErrorCode code;           ///< Error code
  std::string message;      ///< Error message
  std::string field;        ///< Field name where error occurred (optional)

  /**
   * @brief Default constructor: success state
   */
  Error() : code(ErrorCode::kSuccess), message(""), field("") {}

  /**
   * @brief Constructor with error code and message
   * @param c Error code
   * @param msg Error message
   * @param f Field name (optional)
   */
  Error(ErrorCode c, const std::string& msg, const std::string& f = "")
      : code(c), message(msg), field(f) {}

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
  operator bool() const { return IsError(); }

  // Factory methods for common error types
  static Error Success() { return Error(); }

  static Error InvalidJson(const std::string& msg) {
    return Error(ErrorCode::kInvalidJson, msg);
  }

  static Error InvalidConfig(const std::string& msg) {
    return Error(ErrorCode::kInvalidConfig, msg);
  }

  static Error MissingField(const std::string& field) {
    return Error(ErrorCode::kMissingField, "Missing required field: " + field, field);
  }

  static Error InvalidValue(const std::string& field, const std::string& msg) {
    return Error(ErrorCode::kInvalidValue, msg, field);
  }

  static Error ServerNotReady(const std::string& msg = "Server is not ready") {
    return Error(ErrorCode::kServerNotReady, msg);
  }

  static Error ServerError(const std::string& msg) {
    return Error(ErrorCode::kServerError, msg);
  }
};

// =============== Result ===============
struct NoneResult {};

struct RenderResult {
  int renderer_id_;
  int img_width_;
  int img_height_;
  uint8_t* img_buffer_;
};

struct StatsResult {
  size_t ray_seg_num_;
  size_t sim_ray_num_;
  size_t crystal_num_;
};

using Result = std::variant<NoneResult, RenderResult, StatsResult>;


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
   * @brief Construct a new Server
   * @note The server starts running immediately after construction
   */
  Server();

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
  Error CommitConfigFromFile(const std::string& filename);

  /**
   * @brief Get all available results
   * @return Vector of Result objects. Returns empty vector if no results available.
   * @note This is a non-blocking call. It returns immediately even if processing is ongoing.
   * @note If there are no results, the vector may contain NoneResult entries
   */
  std::vector<Result> GetResults();

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

  /**
   * @brief Check if server is idle
   * @return true if server is idle (no processing), false if processing
   */
  bool IsIdle() const;

 private:
  std::shared_ptr<ServerImpl> impl_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_SERVER_H_
