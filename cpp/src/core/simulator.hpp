#ifndef CORE_SIMULATOR_H_
#define CORE_SIMULATOR_H_

#include <cstddef>
#include <memory>

#include "config/proj_config.hpp"
#include "config/sim_data.hpp"
#include "core/math.hpp"

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
  Simulator(const Simulator& other) = delete;
  Simulator(Simulator&& other);
  ~Simulator() = default;

  Simulator& operator=(const Simulator& other) = delete;
  Simulator& operator=(Simulator&& other);

  void Run();
  void Stop();
  bool IsIdle() const;

 private:
  QueuePtrS<SceneConfig> config_queue_;
  QueuePtrS<SimData> data_queue_;
  std::atomic_bool stop_;
  std::atomic_bool idle_;
  
  RandomNumberGenerator rng_;
};

}  // namespace v3
}  // namespace icehalo

#endif  // CORE_SIMULATOR_H_
