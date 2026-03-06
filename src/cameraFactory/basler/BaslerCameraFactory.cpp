#include "BaslerCameraFactory.h"

#include "BaslerCamera.h"

BaslerCameraFactory::BaslerCameraFactory(QObject *parent) { (void)parent; }

std::shared_ptr<AbstractCamera> BaslerCameraFactory::createCamera() { return std::make_shared<BaslerCamera>(nullptr); }
