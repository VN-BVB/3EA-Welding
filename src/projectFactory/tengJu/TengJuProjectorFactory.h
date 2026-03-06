#ifndef TENGJUPROJECTORFACTORY_H
#define TENGJUPROJECTORFACTORY_H

#include <projectFactory/AbstractProjectorFactory.h>

class TengJuProjectorFactory : public AbstractProjectorFactory {
public:
    TengJuProjectorFactory(QObject *parent = nullptr);

signals:

public slots:
    std::shared_ptr<AbstractProjector> createProjector() override;

private:
};

#endif  // TENGJUPROJECTORFACTORY_H
