#ifndef ABSTRACTPROJECTORFACTORY_H
#define ABSTRACTPROJECTORFACTORY_H

#include <plog/Log.h>

#include <QObject>
#include <memory>

class AbstractProjector;

class AbstractProjectorFactory : public QObject {
    Q_OBJECT
public:
    AbstractProjectorFactory(QObject *parent = nullptr);

signals:

public slots:
    virtual std::shared_ptr<AbstractProjector> createProjector() = 0;

private:
};

#endif  // ABSTRACTPROJECTORFACTORY_H
