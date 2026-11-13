#include "ThreadPool.h"

#include <exception>
#include <stdexcept>
#include <utility>

namespace MassMotion
{

ThreadPool::ThreadPool(std::size_t threadCount)
{
    // hardware_concurrency() may return zero when the thread count is unknown.
    if (threadCount == 0)
    {
        threadCount = 1;
    }

    try
    {
        mThreads.reserve(threadCount);
        for (std::size_t i = 0; i < threadCount; ++i)
        {
            mThreads.emplace_back(&ThreadPool::WorkerLoop, this);
        }
    }
    catch (...)
    {
        std::terminate();
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard lock(mMutex);
        mStopping = true;
    }
    mWorkAvailable.notify_all();
    for (auto& thread : mThreads)
    {
        thread.join();
    }
}

void ThreadPool::Enqueue(Task task)
{
    if (!task)
    {
        throw std::invalid_argument("Task must not be empty");
    }

    {
        std::lock_guard lock(mMutex);
        if (mStopping)
        {
            throw std::runtime_error("Cannot enqueue work after the thread pool has stopped");
        }
        mTasks.push(std::move(task));
        ++mPendingTasks;
    }
    mWorkAvailable.notify_one();
}

void ThreadPool::Wait()
{
    std::unique_lock lock(mMutex);
    mWorkFinished.wait(lock,
                       [this]
                       {
                           return mPendingTasks == 0;
                       });
}

void ThreadPool::WorkerLoop()
{
    while (true)
    {
        Task task;
        // Release the queue mutex before executing the task.
        {
            std::unique_lock lock(mMutex);
            mWorkAvailable.wait(lock,
                                [this]
                                {
                                    return mStopping || !mTasks.empty();
                                });
            if (mStopping && mTasks.empty())
            {
                return;
            }
            task = std::move(mTasks.front());
            mTasks.pop();
        }
        task();
        // Update the shared task count and wake Wait() callers when all tasks are complete.
        {
            std::lock_guard lock(mMutex);
            if (--mPendingTasks == 0)
            {
                mWorkFinished.notify_all();
            }
        }
    }
}

} // namespace MassMotion
