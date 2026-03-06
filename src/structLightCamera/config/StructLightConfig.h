#ifndef STRUCTLIGHTCONFIG_H
#define STRUCTLIGHTCONFIG_H

#include <plog/Log.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

#include "utils/common/CommonFunc.h"
#include "utils/myMatrixVector/MyMatrixVector.h"

class StructLightConfig {
public:
    static StructLightConfig &getInstance();  // 获取单例的方法

    StructLightConfig(const StructLightConfig &) = delete;             // 禁止拷贝构造
    StructLightConfig &operator=(const StructLightConfig &) = delete;  // 禁止赋值操作

    void myDataStructure2LibDataStructure();  // 自定义数据类型转换为库数据类型

    std::string getPrimaryCameraSerialNum() const;
    void setPrimaryCameraSerialNum(const std::string &newPrimaryCameraSerialNum);
    std::string getSecondaryCameraSerialNum() const;
    void setSecondaryCameraSerialNum(const std::string &newSecondaryCameraSerialNum);
    std::string getProjectorSerialNum() const;
    void setProjectorSerialNum(const std::string &newProjectorSerialNum);

private:
    StructLightConfig();
    ~StructLightConfig();

    static StructLightConfig *instance;  // 静态实例指针
    static std::mutex instanceMutex;     // 保护静态实例的互斥锁

    std::string robotType = MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN);             // 机器人类型
    std::string handEyeType = MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND);  // 手眼类型

    std::string primaryCameraSerialNum = "00000000";    // 主相机序列号
    std::string secondaryCameraSerialNum = "00000000";  // 次相机序列号
    std::string projectorSerialNum = "00000000";        // 投影仪序列号

    // **************************** 存入JSON的数据类型 ****************************
    MyMatrix CameraHomography;       // 相机的单应性矩阵
    MyMatrix ProjectHomography;      // 投影仪的单应性矩阵
    MyMatrix CameraInternal;         // 相机内参
    MyMatrix ProjectInternal;        // 投影仪内参
    MyMatrix CameraTransformation;   // 相机的变换矩阵
    MyMatrix ProjectTransformation;  // 投影仪的变换矩阵
    MyVector CameraDistortion;       // 相机畸变系数向量
    MyVector ProjectDistortion;      // 投影仪畸变系数

    // **************************** 程序使用的的数据类型 ****************************
    cv::Mat Ac;                              // 相机的单应性矩阵
    cv::Mat Ap;                              // 投影仪的单应性矩阵
    cv::Mat Kc;                              // 相机内参
    cv::Mat Kp;                              // 投影仪内参
    Eigen::Matrix4d Trans_c;                 // 相机的变换矩阵
    Eigen::Matrix4d Trans_p;                 // 投影仪的变换矩阵
    std::vector<double> camera_distortion;   // 相机畸变系数向量
    std::vector<double> project_distortion;  // 投影仪畸变系数

    template <class Archive>
    void serialize(Archive &ar) {
        ar(CEREAL_NVP(robotType), CEREAL_NVP(handEyeType), CEREAL_NVP(primaryCameraSerialNum), CEREAL_NVP(secondaryCameraSerialNum),
           CEREAL_NVP(projectorSerialNum), CEREAL_NVP(CameraHomography), CEREAL_NVP(ProjectHomography), CEREAL_NVP(CameraInternal),
           CEREAL_NVP(ProjectInternal), CEREAL_NVP(CameraTransformation), CEREAL_NVP(ProjectTransformation), CEREAL_NVP(CameraDistortion),
           CEREAL_NVP(ProjectDistortion));
    }

    friend class StructLightCamera;
    friend class PointCloudReconstruction;
    friend class cereal::access;
#ifdef SMART_CAMERA
    friend class SeamDetWithSeg;
#endif
};

#endif  // STRUCTLIGHTCONFIG_H
