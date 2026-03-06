#include "StructLightConfig.h"

#include "utils/myMatrixVector/MyMatrixVector.h"

// 初始化静态成员
StructLightConfig *StructLightConfig::instance = nullptr;  // 静态实例指针
std::mutex StructLightConfig::instanceMutex;               // 保护静态实例的互斥锁

StructLightConfig &StructLightConfig::getInstance() {
    if (instance == nullptr) {
        std::lock_guard<std::mutex> lock(instanceMutex);  // 加锁, 确保线程安全
        if (instance == nullptr) {
            instance = new StructLightConfig();  // 创建实例
        }
        (void)lock;
    }

    return *instance;
}

StructLightConfig::StructLightConfig()
    : CameraHomography(3, 4),
      ProjectHomography(3, 4),
      CameraInternal(3, 3),
      ProjectInternal(3, 3),
      CameraTransformation(4, 4),
      ProjectTransformation(4, 4),
      CameraDistortion(5),
      ProjectDistortion(5) {}

// 自定义数据类型转换为库数据类型
void StructLightConfig::myDataStructure2LibDataStructure() {
    MyToolFunc::write2Mat3x4(CameraHomography, Ac);
    MyToolFunc::write2Mat3x4(ProjectHomography, Ap);
    MyToolFunc::write2Mat3x3(CameraInternal, Kc);
    MyToolFunc::write2Mat3x3(ProjectInternal, Kp);
    MyToolFunc::write2Eigen4x4d(CameraTransformation, Trans_c);
    MyToolFunc::write2Eigen4x4d(ProjectTransformation, Trans_p);
    MyToolFunc::write2Vector5(CameraDistortion, camera_distortion);
    MyToolFunc::write2Vector5(ProjectDistortion, project_distortion);
}

std::string StructLightConfig::getPrimaryCameraSerialNum() const { return primaryCameraSerialNum; }

void StructLightConfig::setPrimaryCameraSerialNum(const std::string &newPrimaryCameraSerialNum) {
    primaryCameraSerialNum = newPrimaryCameraSerialNum;
}

std::string StructLightConfig::getSecondaryCameraSerialNum() const { return secondaryCameraSerialNum; }

void StructLightConfig::setSecondaryCameraSerialNum(const std::string &newSecondaryCameraSerialNum) {
    secondaryCameraSerialNum = newSecondaryCameraSerialNum;
}

std::string StructLightConfig::getProjectorSerialNum() const { return projectorSerialNum; }

void StructLightConfig::setProjectorSerialNum(const std::string &newProjectorSerialNum) { projectorSerialNum = newProjectorSerialNum; }
