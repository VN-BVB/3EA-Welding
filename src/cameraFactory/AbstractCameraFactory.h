#ifndef ABSTRACTCAMERAFACTORY_H
#define ABSTRACTCAMERAFACTORY_H

#include <plog/Log.h>

#include <QObject>

class AbstractCamera;

class AbstractCameraFactory : public QObject {
    Q_OBJECT
public:
    AbstractCameraFactory(QObject* parent = nullptr);
    ~AbstractCameraFactory();

signals:

public slots:
    virtual std::shared_ptr<AbstractCamera> createCamera() = 0;

private:
};

#endif  // ABSTRACTCAMERAFACTORY_H
