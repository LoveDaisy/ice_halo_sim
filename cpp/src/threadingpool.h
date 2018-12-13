#ifndef ICEHALOSIM_THREADINGPOOL_H
#define ICEHALOSIM_THREADINGPOOL_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>


namespace IceHalo {

class Pool {
public:
  ~Pool() = default;

  void start();
  void addJob(std::function<void()> job);
  void waitFinish();
  bool taskRunning();

  static Pool* getInstance();

private:
  Pool();

  unsigned int threadNum;
  std::vector<std::thread> pool;
  std::atomic<bool> alive;

  std::queue<std::function<void()> > queue;
  std::mutex queueMutex;
  std::condition_variable queueCondition;

  std::atomic<int> runningJobs;
  std::atomic<int> aliveThreads;
  std::mutex taskMutex;
  std::condition_variable taskCondition;

  void workingFunction();

  static Pool* instance;
  static std::mutex instanceMutex;

};


}   // namespace IceHalo


#endif
