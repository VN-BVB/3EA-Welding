#ifndef BAOYUANROBOTFACTORY_H
#define BAOYUANROBOTFACTORY_H

#include <robotFactory/AbstractRobotFactory.h>

class BaoYuanRobotFactory : public AbstractRobotFactory {
public:
    BaoYuanRobotFactory(QObject *parent = nullptr);

public slots:
    std::shared_ptr<AbstractRobot> createRobot() override;
};

#endif  // BAOYUANROBOTFACTORY_H
