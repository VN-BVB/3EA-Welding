#ifndef ANCHUANROBOTFACTORY_H
#define ANCHUANROBOTFACTORY_H

#include <robotFactory/AbstractRobotFactory.h>

class AnChuanRobotFactory : public AbstractRobotFactory {
public:
    AnChuanRobotFactory(QObject *parent = nullptr);

public slots:
    std::shared_ptr<AbstractRobot> createRobot() override;
};

#endif  // ANCHUANROBOTFACTORY_H
