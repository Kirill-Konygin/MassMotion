#ifndef MASSMOTION_THREAD_POOL_H
#define MASSMOTION_THREAD_POOL_H

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace MassMotion
{

class ThreadPool
{
  public:
    using Task = std::function<void()>;

    explicit ThreadPool(std::size_t threadCount = std::thread::hardware_concurrency());
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    void Enqueue(Task task);
    // Must be called outside tasks running in this pool.
    void Wait();

  private:
    void WorkerLoop();

    std::mutex mMutex;
    std::condition_variable mWorkAvailable;
    std::condition_variable mWorkFinished;
    std::queue<Task> mTasks;
    std::size_t mPendingTasks = 0;
    bool mStopping = false;

    std::vector<std::thread> mThreads;
};

} // namespace MassMotion

#endif // MASSMOTION_THREAD_POOL_H
