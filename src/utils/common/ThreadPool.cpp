#include "ThreadPool.h"

ThreadPool::ThreadPool(size_t threadCount) : stopFlag(false) {
    for (size_t i = 0; i < threadCount; ++i) {  // 创建对应数量的线程
        workers.emplace_back([this] {
            while (true) {  // 每个线程进入死循环, 直到有任务进来被唤醒 (没有任务时不占用CPU资源)
                Task task;
                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    // 使用条件变量 condition.wait(...) 挂起线程, 直到有任务被唤醒或者stopFlag为true
                    condition.wait(lock, [this] { return stopFlag || !tasks.empty(); });

                    if (stopFlag && tasks.empty()) return;

                    task = std::move(tasks.front());
                    tasks.pop();
                }
                task();  // 执行任务
            }
        });
    }
}

void ThreadPool::waitAll() {
    std::unique_lock<std::mutex> lock(queueMutex);
    waitCondition.wait(lock, [this] { return unfinishedTasks == 0 && tasks.empty(); });  // 等待所有任务完成
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        stopFlag = true;
    }
    condition.notify_all();
    for (auto& t : workers) {
        if (t.joinable()) t.join();
    }
}
