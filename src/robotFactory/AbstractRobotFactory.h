#ifndef ABSTRACTROBOTFACTORY_H
#define ABSTRACTROBOTFACTORY_H

#include <plog/Log.h>

#include <QObject>
#include <memory>

class AbstractRobot;

class AbstractRobotFactory : public QObject {
    Q_OBJECT
public:
    AbstractRobotFactory(QObject *parent = nullptr);
    ~AbstractRobotFactory();

signals:

public slots:
    virtual std::shared_ptr<AbstractRobot> createRobot() = 0;

private:
};

#endif  // ABSTRACTROBOTFACTORY_H
