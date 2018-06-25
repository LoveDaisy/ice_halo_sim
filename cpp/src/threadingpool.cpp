#include "threadingpool.h"


namespace IceHalo {


Pool * Pool::instance = nullptr;
std::mutex Pool::instanceMutex;


Pool * Pool::getInstance()
{
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


Pool::Pool() : 
    threadNum(std::thread::hardware_concurrency()), alive(false), 
    runningTasks(0), aliveThreads(0)
{
    start();
}


void Pool::start()
{
    if (alive || runningTasks > 0 || aliveThreads > 0) {
        return;
    }

    pool.clear();
    alive = true;
    printf("Threading pool size: %u\n", threadNum);
    for (int ii = 0; ii < threadNum; ii++) {
        pool.emplace_back(std::thread(&Pool::workingFunction, this));
        aliveThreads += 1;
    }
}


void Pool::stop()
{
    if (!alive) {
        return;
    }

    printf("stop()\n");
    queueCondition.notify_one();
    alive = false;
    {
        std::unique_lock<std::mutex> lock(taskMutex);
        taskCondition.wait(lock, [this]{ return this->aliveThreads <= 0; });
    }
    for (auto &t : pool) {
        if (t.joinable()) {
            t.join();
        }
    }
}


void Pool::addJob(std::function<void()> job)
{
    if (!alive) {
        return;
    }

    {
        std::unique_lock<std::mutex> lock(queueMutex);
        queue.push(job);
    }
    queueCondition.notify_one();
}


void Pool::waitFinish()
{
    {
        std::unique_lock<std::mutex> lock(taskMutex);
        taskCondition.wait(lock, [this]{ return !this->taskRunning(); });
    }
}


bool Pool::taskRunning()
{
    return runningTasks > 0 || !queue.empty();
}


void Pool::workingFunction()
{
    while(true)
    {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCondition.wait(lock, [this]{ return !this->queue.empty() || !this->alive; });
            if (!alive) {
                printf("stopped, break.\n");
                aliveThreads -= 1;
                taskCondition.notify_one();
                queueCondition.notify_one();
                break;
            }
            job = queue.front();
            queue.pop();
            runningTasks += 1;
        }
        job();
        runningTasks -= 1;
        taskCondition.notify_one();
    }
}


}