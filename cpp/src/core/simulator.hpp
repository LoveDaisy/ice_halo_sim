#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <memory>

#include "protocol/proj_config.hpp"
#include "protocol/sim_data.hpp"

namespace icehalo {
namespace v3 {

template <class T>
class Queue;

template <class T>
using QueuePtrU = std::unique_ptr<Queue<T>>;
template <class T>
using QueuePtrS = std::shared_ptr<Queue<T>>;

class Simulator {
 public:
  enum State {
    kIdle,
    kRunning,
  };

  Simulator(QueuePtrS<SceneConfig> config_queue, QueuePtrS<SimData> data_queue);

  void Run();
  void Stop();

 private:
  QueuePtrS<SceneConfig> config_queue_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CORE_SIMULATOR_H_
