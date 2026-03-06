#ifndef RAILWELDINGSYSTEM_H
#define RAILWELDINGSYSTEM_H
class RailWeldingSystem : public QObject {
    Q_OBJECT
public:
    RailWeldingSystem(QObject *parent = nullptr);
    ~RailWeldingSystem();
};

#endif  // RAILWELDINGSYSTEM_H
