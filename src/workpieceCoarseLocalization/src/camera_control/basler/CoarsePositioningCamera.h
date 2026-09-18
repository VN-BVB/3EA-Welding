#ifndef COARSEPOSITIONINGCAMERA_H
#define COARSEPOSITIONINGCAMERA_H

#include <QVTKWidget.h>
#include <plog/Init.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>

#include <QColor>
#include <QDebug>
#include <QEventLoop>
#include <QMutex>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <algorithm>
#include <ctime>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>

#include "cameraFactory/AbstractCamera.h"
#include "cameraFactory/AbstractCameraFactory.h"
#include "cameraFactory/basler/BaslerCameraFactory.h"
#include "workpieceCoarseLocalization/include/CoarseLocalizationMatrix.h"
#include "workpieceCoarseLocalization/include/maskImageProcessConfig.hpp"

enum COARES_LOC_CAMERA {
    CAMERA_1 = 0x01,           // 0000 0001
    CAMERA_2 = 0x02,           // 0000 0010
    CAMERA_UNCONNECTED = 0x00  // 0000 0000
};
using namespace MaskTransformConfig;
class CoarsePositioningCamera : public QObject {
    Q_OBJECT
public:
    CoarsePositioningCamera();
    ~CoarsePositioningCamera();
    void initCameras();
    void openCamera();   // 开启相机并进行采图
    void closeCamera();  // 关闭相机
    void loadCalibConfigFromFile(const std::string& filename);
    void whenGetCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode);

    int imageSaverToInfer = 0;               // 需要保存的推理图象数
    int imageNumberToSaveInCalibration = 0;  // 需要保存的标定图象数（相机内参，畸变系数，平面，手眼，三轴方向向量）
    std::string currentS_N;                  // 当前选择相机编号
    std::vector<std::string> S_Ns;           // 相机编号群
    int saveTypeEnable;                      // 图像保存类型
    bool cameraFlag = true;                  // 相机开启使能

    void whenCameraImageInfer();
    void whenChooseCameraToShow();

public slots:
    void whenGetCameraExposure(int exposure);
signals:
    void sendImageToView(cv::Mat image);
    void sendSerialNumber(std::vector<std::string> SerialNumbers);
    void imageReady();
    void sendCvImagesToInfer(std::vector<cv::Mat> cvImages);
    void appendCameraLog(QString message);
    void sendGetCurrentWaypoint();
    void cameraStartGrabbing();
    void sendCameraStatus(std::vector<COARES_LOC_CAMERA> device, std::vector<QString> color);  // 发送粗定位相机状态

private:
    double exposure = 88888;   // 相机曝光率
    int savedCalibImages = 0;  // 已经保存的图像数
    int savedPlaneImages = 0;
    int savedTrackImages = 0;
    int savednNormalImages = 0;
    int calibCameraIndex = 0;
    int* savedImages;

    std::shared_ptr<AbstractCameraFactory> cameraFactory{nullptr};
    std::vector<std::shared_ptr<AbstractCamera>> cameras;  // pylon相机对象群
    QThread* sharedCameraThread = new QThread(this);

    std::vector<std::string> serialNum;
    std::vector<cv::Mat> imagesReceived;
    int receivedImages;

    friend class RailWeldingSystem;
    friend class WeldingMainWindow;
};
#endif  // COARSEPOSITIONINGCAMERA_H
