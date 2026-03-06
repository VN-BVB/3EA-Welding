#include "BaoYuanRobotFactory.h"

#include "BaoYuanRobot.h"

BaoYuanRobotFactory::BaoYuanRobotFactory(QObject *parent) {}

std::shared_ptr<AbstractRobot> BaoYuanRobotFactory::createRobot() { return std::make_shared<BaoYuanRobot>(nullptr); }
