#ifndef STRUCTLIGHTCAMERA_H
#define STRUCTLIGHTCAMERA_H

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <plog/Log.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <QImage>
#include <QObject>
#include <QString>
#include <QThread>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

#include "cameraFactory/AbstractCamera.h"

class AbstractCamera;
class AbstractProjector;
class AbstractCameraFactory;
class AbstractProjectorFactory;
class RailWeldingSystem;
class PointCloudReconstruction;
class WeldSeamInfo;

enum DEVICE {
    PRIMARY_CAMERA = 0x01,    // 0000 0001
    SECONDARY_CAMERA = 0x02,  // 0000 0010
    PROJECTOR = 0x04          // 0000 0100
};

enum RECONSTRUCTION_MODE {
    COMMON,    // 普通模式
    WORKPIECE  // 工件模式
};
enum WORKPIECE_TYPE { STEEL_ANGLE, STEEL_DEFAULT };

class StructLightCamera : public QObject {
    Q_OBJECT
public:
    StructLightCamera(QObject *parent = nullptr);
    StructLightCamera(std::shared_ptr<AbstractCameraFactory> camFac, std::shared_ptr<AbstractProjectorFactory> projFac);
    ~StructLightCamera();

    void initConfig();                    // 初始化配置信息
    void writeConfig();                   // 写配置文件
    void readConfig();                    // 读配置文件
    void printConfig();                   // 打印配置文件
    void initCamerasAndProjector();       // 初始化相机和投影仪
    void initPointCloudReconstruction();  // 初始化点云重建类

    uint8_t open();  // 打开相机和投影仪
    void close();    // 关闭相机和投影仪

signals:
    void cameraStartGrabbing();                                                          // 相机开始采图信号
    void sendStructLightStatus(std::vector<DEVICE> device, std::vector<QString> color);  // 结构光设备状态信号
    void sendTriggerProj();                                                              // 触发投影信号
    void sendPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr workbenchPointCloud);        // 发送重建结果点云
    void sendWeldAreaInfo(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);      // 发送焊缝区域点云
    void sendWeldAreaInfoSD(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);    // 发送三轴焊缝区域点云
    void sendMessage2Ui(QString message);                                                // 发送信息到UI界面
    void sendUpdataWorkbenchover();                                                      // 发送背景平面重建完成

public slots:
    // 相机相关
    void whenGetPrimaryCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode);    // 从主相机获取到图像
    void whenGetSecondaryCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode);  // 从次相机获取到图像
    void whenGetCameraExposure(int exposure);                                  // 相机曝光改变
    void whenGetCameraGain(int gain);                                          // 相机增益改变
    void whenGetCameraWorkMode(CAMERA_WORK_MODE workMode);                     // 相机工作模式改变

    // 投影仪相关
    void whenGetProjectorBrightness(int brightness);  // 投影仪亮度改变
    void whenGetProjectorFps(int fps);                // 投影仪帧率改变
    void whenGetProjectorNum(int num);                // 投影仪图片数量改变

    // 重建相关
    void whenGlobalReconstruct();                                                                   // 全局点云重建
    void whenLocalReconstruct(double normMinU, double normMaxU, double normMinV, double normMaxV);  // 局部点云重建
#ifdef SMART_CAMERA
    void whenScanWorkpiece();  // 扫描工件
#endif

private:
    double normMinU = 0, normMaxU = 1, normMinV = 0, normMaxV = 1;  // 普通模式下重建范围参数
    std::shared_ptr<AbstractCamera> primaryCamera{nullptr};         // 主相机
    std::shared_ptr<AbstractCamera> secondaryCamera{nullptr};       // 次相机
    std::shared_ptr<AbstractProjector> projector{nullptr};          // 投影仪
    QThread *primaryCameraThread = new QThread;                     // 主相机线程
    QThread *secondaryCameraThread = new QThread;                   // 次相机线程
    QThread *projectorThread = new QThread;                         // 投影仪线程
    std::shared_ptr<AbstractCameraFactory> cameraFactory{nullptr};  // 抽象相机工厂 (由外部通过依赖注入的方式注入)
    std::shared_ptr<AbstractProjectorFactory> projectorFactory{nullptr};  // 抽象投影仪工厂 (由外部通过依赖注入的方式注入)
    std::shared_ptr<PointCloudReconstruction> pointCloudReconstruction{nullptr};  // 点云重建类

    uint8_t deviceStatus = 0x00;              // 三个硬件设备的连接状态
    uint8_t cameraImagCapturedStatus = 0x00;  // 两个相机的采图数量状态 (是否采集到对应数量)
    std::shared_ptr<std::vector<cv::Mat>> primaryCameraCapturedImg =
        std::make_shared<std::vector<cv::Mat>>();  // 主相机采集到的图像
    std::shared_ptr<std::vector<cv::Mat>> secondaryCameraCapturedImg =
        std::make_shared<std::vector<cv::Mat>>();  // 主相机采集到的图像

    WORKPIECE_TYPE workpieceType = WORKPIECE_TYPE::STEEL_DEFAULT;
    RECONSTRUCTION_MODE reconstructionMode = RECONSTRUCTION_MODE::WORKPIECE;  // 重建模式
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;                           // 普通重建点云
#ifdef SMART_CAMERA
    pcl::PointCloud<pcl::PointXYZ>::Ptr workbenchPointCloud;  // 背景平面点云
    std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo;  // 焊缝区域点云
#endif

    friend class SettingWidget;
    friend class WeldingMainWindow;
#ifdef SMART_CAMERA
    friend class RailWeldiongSystem;  // 友元类
#endif
};

#endif  // STRUCTLIGHTCAMERA_H
