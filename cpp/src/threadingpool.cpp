#include "threadingpool.h"

namespace IceHalo {


Pool * Pool::instance = nullptr;
std::mutex Pool::instanceMutex;


Pool * Pool::getInstance() {
  if (instance == nullptr) {
    {
      std::unique_lock<std::mutex> lock(instanceMutex);
      if (instance == nullptr) {
        instance = new Pool();
      }
    }
  }
  return instance;
}


Pool::Pool()
    : threadNum(std::thread::hardware_concurrency()), alive(false),
      runningJobs(0), aliveThreads(0) {
  start();
}


void Pool::start() {
  if (alive || runningJobs > 0 || aliveThreads > 0) {
    return;
  }

  pool.clear();
  alive = true;
  aliveThreads = 0;
  printf("Threading pool size: %u\n", threadNum);
  for (decltype(threadNum) ii = 0; ii < threadNum; ii++) {
    pool.emplace_back(std::thread(&Pool::workingFunction, this));
    aliveThreads += 1;
  }
}



void Pool::addJob(std::function<void()> job) {
  if (!alive) {
    return;
  }

  {
    std::unique_lock<std::mutex> lock(queueMutex);
    queue.emplace(job);
  }
  queueCondition.notify_one();
}


void Pool::waitFinish() {
  std::unique_lock<std::mutex> lock(taskMutex);
  taskCondition.wait(lock, [this]{ return !taskRunning(); });
}


bool Pool::taskRunning() {
  return runningJobs > 0 || !queue.empty();
}


void Pool::workingFunction() {
  std::unique_lock<std::mutex> lock(queueMutex);
  while(true) {
    if (!queue.empty()) {
      std::function<void()> job = queue.front();
      queue.pop();
      lock.unlock();
      {
        std::unique_lock<std::mutex> lk(taskMutex);
        runningJobs += 1;
      }
      job();
      {
        std::unique_lock<std::mutex> lk(taskMutex);
        runningJobs -= 1;
      }
      lock.lock();
      taskCondition.notify_one();
    } else if (!alive) {
      aliveThreads -= 1;
      taskCondition.notify_one();
      queueCondition.notify_one();
      break;
    } else {
      taskCondition.notify_one();
      queueCondition.wait(lock, [this]{ return !this->queue.empty() || !this->alive; });
    }

  }
}


}