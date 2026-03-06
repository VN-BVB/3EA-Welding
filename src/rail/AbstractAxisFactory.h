#ifndef AXIS_FACTORY_H
#define AXIS_FACTORY_H

#include <memory>
#include <vector>

#include "AbstractAxis.h"
#include "concrete_axis/AxisManager.h"

class AbstractAxisFactory {
public:
    // 工厂方法现在接收一个 Axis 参数来创建对应的轴
    static std::shared_ptr<AbstractAxis> createAxis(Axis axisType, PLCCommunication* comm, QObject* parent = nullptr) {
        return std::make_shared<AxisManager>(axisType, comm, parent);
    }

    static std::vector<std::shared_ptr<AbstractAxis>> createAllAxes(PLCCommunication* comm, QObject* parent = nullptr) {
        return {createAxis(Axis::X, comm, parent), createAxis(Axis::Y, comm, parent), createAxis(Axis::Z, comm, parent)};
    }
};
#endif
