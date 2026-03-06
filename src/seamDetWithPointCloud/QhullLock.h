#ifndef QHULLLOCK_H
#define QHULLLOCK_H

#include <mutex>

// 全局互斥锁，用于串行化 Qhull 调用
inline std::mutex& getQhullMutex() {
    static std::mutex qhull_mutex;
    return qhull_mutex;
}

#endif  // QHULLLOCK_H
