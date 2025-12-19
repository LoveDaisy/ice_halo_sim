#ifndef INCLUDE_SERVER_H_
#define INCLUDE_SERVER_H_

#include <fstream>
#include <memory>
#include <variant>
#include <vector>


namespace icehalo {
namespace v3 {

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
   * @note The configuration format should follow V3 configuration schema
   * @see configuration.md for configuration format details
   */
  void CommitConfig(std::string config_str);

  /**
   * @brief Commit configuration from file
   * @param config_file Input file stream containing JSON configuration
   * @note The configuration format should follow V3 configuration schema
   * @see configuration.md for configuration format details
   */
  void CommitConfig(std::ifstream& config_file);

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
