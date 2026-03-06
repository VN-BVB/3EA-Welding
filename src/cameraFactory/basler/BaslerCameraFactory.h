#ifndef BASLERCAMERAFACTORY_H
#define BASLERCAMERAFACTORY_H

#include "cameraFactory/AbstractCameraFactory.h"

class BaslerCameraFactory : public AbstractCameraFactory {
public:
    BaslerCameraFactory(QObject* parent = nullptr);

signals:

public slots:
    std::shared_ptr<AbstractCamera> createCamera() override;

private:
};

#endif  // BASLERCAMERAFACTORY_H
