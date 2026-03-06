#include "TengJuProjectorFactory.h"

#include "TengJuProjector.h"

TengJuProjectorFactory::TengJuProjectorFactory(QObject *parent) { (void)parent; }

std::shared_ptr<AbstractProjector> TengJuProjectorFactory::createProjector() { return std::make_shared<TengJuProjector>(nullptr); }
