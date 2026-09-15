#ifndef YOLOINFERENCE_H
#define YOLOINFERENCE_H
#include <plog/Init.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

#include <QDebug>
#include <QObject>
#include <limits>

#include "workpieceCoarseLocalization/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h"

using namespace MaskTransformConfig;
#if defined(ROM_CONFIG) || defined(LI_CONFIG) || defined(ZHANG_CONFIG)
const std::string CoarseSegEnginePath = "./data/DL_models/coaLocModel/best2.engine";
const std::string CoarseObjDetEnginePath = "./data/DL_models/coaLocModel/weldAreaDetRom.engine";
#elif GONG_RAIL_CONFIG
const std::string CoarseSegEnginePath = "./data/DL_models/coaLocModel/workpieceSegRail.engine";
const std::string CoarseObjDetEnginePath = "./data/DL_models/coaLocModel/weldAreaDetRail.engine";
#elif A17_CONFIG
const std::string CoarseSegEnginePath = "./data/DL_models/coaLocModel/workpieceSeg.engine";
const std::string CoarseObjDetEnginePath = "./data/DL_models/coaLocModel/weldAreaDet.engine";
#endif

class YoloSegInference : public QObject {
    Q_OBJECT
public:
    YoloSegInference();
    void whenPathNeedToInfer(std::string path);
    void whenImageNeedToInfer(std::vector<cv::Mat> cvImages);

private:
    cv::Mat res, image;
    int imgNum, detectedWp;
    std::vector<segYolo11::ObjectYolo11Seg> objs;
    std::shared_ptr<AbstractSegment> WorkpieceSegmentation{nullptr};  // 分割算法类
    std::vector<SegResult> segRes;                                    // 分割结果
    std::vector<std::vector<SegResult>> allSegResults;

    void initSegment();
    void inferSingleImage(cv::Mat &inputImage, double yAxisEncoderValue = std::numeric_limits<double>::quiet_NaN());
    void whenImageNeedToSave(const cv::Mat &inferResult, const std::string &savePrefix);
    void parseSegResults(const std::vector<SegResult> &segResults, std::vector<segYolo11::ObjectYolo11Seg> &objs, cv::Mat &res);
    void sortSegObjects(std::vector<segYolo11::ObjectYolo11Seg> &objs, const std::string &axis, const std::string &order);
    void colorizeAndDisplayConnectedComponents(cv::Mat &mask);  // 筛选连通域

signals:
    void sendSignalTocalculate();
    void sendAppendInferLog(QString message);
    void sendInferResultToMainWindow(cv::Mat res);
    void sendCoordinateTofit(std::vector<segYolo11::ObjectYolo11Seg> objs, int imgNum, double yAxisEncoderValue);
    void sendWeldBoxInfo(const std::vector<std::vector<std::array<double, 4>>> &boxInfos);
};

class YoloDetInference : public QObject {
    Q_OBJECT
public:
    YoloDetInference();
    void whenRecieveWpMaskInWorld(std::vector<cv::Point3d> worldCenters, std::vector<cv::Mat> worldMaskImages);

private:
    int imgNum, workpieceNum;
    cv::Mat res, image;
    std::vector<det::Object> objs;
    std::vector<DetResult> detRes;                                  // 目标检测结果
    std::shared_ptr<AbstractObjectDetect> weldsDetection{nullptr};  // 目标检测算法类
    std::vector<cv::Point3d> maskWorldCenters;
    std::vector<cv::Size> maskCanvasSizes;
    int rectRotationAngleYolo = rectRotationAngle;
    void initObjectDetect();
    void whenImageNeedToInfer(std::vector<cv::Mat> cvImages);
    void inferSegAndCalcTime();
    void inferSingleImage(cv::Mat &inputImage);
    void whenCoordinatesNeedToProceed(std::vector<std::vector<cv::Rect_<float>>> rect_Dets, std::vector<cv::Point3d> worldCenters);
    void parseDetResults(const std::vector<DetResult> &detResult, std::vector<det::Object> &objs, cv::Mat &res);
    void whenImageNeedToSave(const cv::Mat &inferResult, const std::string &savePrefix, const std::string &tag);
signals:
    void sendInferResultToMainWindow(cv::Mat res);
    void sendBoxInfoToDisplay(const std::vector<std::vector<std::array<double, 4>>> &boxInfos);
};

#endif  // YOLOINFERENCE_H
