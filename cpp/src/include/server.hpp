#ifndef INCLUDE_SERVER_H_
#define INCLUDE_SERVER_H_

#include <fstream>
#include <memory>
#include <vector>

#include "include/result.hpp"

namespace icehalo {
namespace v3 {

class ServerImpl;

class Server {
 public:
  Server();

  void CommitConfig(std::string config_str);
  void CommitConfig(std::ifstream& config_file);
  std::vector<Result> GetResults() const;  // if there is no result, it returns NoneResult, and does **NOT** block.

  void Stop();  // No Run() method. Because server start running immediately after construction.

 private:
  std::shared_ptr<ServerImpl> impl_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // INCLUDE_SERVER_H_
