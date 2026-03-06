#include "AnChuanRobotFactory.h"

#include "AnChuanRobot.h"

AnChuanRobotFactory::AnChuanRobotFactory(QObject *parent) : AbstractRobotFactory{parent} {}

std::shared_ptr<AbstractRobot> AnChuanRobotFactory::createRobot() { return std::make_shared<AnChuanRobot>(); }
