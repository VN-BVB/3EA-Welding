#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
    explicit ThreadPool(size_t threadCount = 3);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // 添加任务函数模板, 可以传入『任意函数』和『任意数量和类型的参数』, 返回 future
    template <typename Func, typename... Args>
    auto addTask(Func&& func, Args&&... args) -> std::future<decltype(func(args...))>;

    // 显式等待所有任务完成
    void waitAll();

private:
    using Task = std::function<void()>;

    std::vector<std::thread> workers;
    std::queue<Task> tasks;

    std::mutex queueMutex;
    std::condition_variable condition;
    std::atomic<bool> stopFlag;

    // 等待所有任务完成
    std::atomic<size_t> unfinishedTasks{0};
    std::condition_variable waitCondition;
};

// 模板实现
template <typename Func, typename... Args>
auto ThreadPool::addTask(Func&& func, Args&&... args) -> std::future<decltype(func(args...))> {
    using ReturnType = decltype(func(args...));  // 推导函数返回值类型

    auto boundTask = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);  // 把函数和参数绑定成一个可调用的对象
    // 将绑定后的函数包装成 packaged_task, 可以用 future 获取其返回值
    auto packaged = std::make_shared<std::packaged_task<ReturnType()>>(std::move(boundTask));
    std::future<ReturnType> future = packaged->get_future();

    {
        std::lock_guard<std::mutex> lock(queueMutex);  // 加锁并安全地将unfinishedTasks加一
        ++unfinishedTasks;

        tasks.emplace([this, packaged]() {
            (*packaged)();  // 执行传入的任务
            {
                std::lock_guard<std::mutex> lock(queueMutex);  // 加锁并安全地将unfinishedTasks减一
                --unfinishedTasks;
                if (unfinishedTasks == 0 && tasks.empty()) {
                    waitCondition.notify_all();
                }
            }
        });
    }

    condition.notify_one();  // 随机唤醒一个等待的线程
    return future;
}

#endif  // THREADPOOL_H
