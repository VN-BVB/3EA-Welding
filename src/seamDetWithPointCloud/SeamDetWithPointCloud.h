#ifndef SEAMDETWITHPOINTCLOUD_H
#define SEAMDETWITHPOINTCLOUD_H

#include <plog/Log.h>

#include <QObject>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "utils/common/ThreadPool.h"

class SeamDetWithPointCloud : public QObject {
    Q_OBJECT
public:
    explicit SeamDetWithPointCloud(QObject* parent = nullptr);
};

#endif  // SEAMDETWITHPOINTCLOUD_H
