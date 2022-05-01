#include "include/server.hpp"

#include <fstream>

#include "include/result.hpp"

namespace icehalo {
namespace v3 {

class ServerImpl {};

void Server::CommitConfig(std::ifstream& /* f */) {}

Result Server::GetResult() {
  return NoneResult{};
}

}  // namespace v3
}  // namespace icehalo
