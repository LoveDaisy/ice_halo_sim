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

class Server {
 public:
  Server();

  void CommitConfig(std::string config_str);
  void CommitConfig(std::ifstream& config_file);
  std::vector<Result> GetResults();  // if there is no result, it returns NoneResult, and does **NOT** block.

  void Stop();  // No Run() method. Because server start running immediately after construction.
  void Terminate();
  bool IsIdle() const;

 private:
  std::shared_ptr<ServerImpl> impl_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_SERVER_H_
