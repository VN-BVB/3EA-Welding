#ifndef FITTINGWORKPIECECOORDINATE_H
#define FITTINGWORKPIECECOORDINATE_H

#include <QImage>
#include <QObject>
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include "deepLearning/objectDetect/yolo11/Yolo11ObjDetInference.h"
#include "deepLearning/segment/yolo11/Yolo11SegInference.h"
#include "workpieceCoarseLocalization/include/CoarseLocalizationMatrix.h"
#include "workpieceCoarseLocalization/include/maskImageProcessConfig.hpp"

using namespace segYolo11;
struct cameraConfig {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<double> globalPlane;
    cv::Mat cameraToBaseMatrix;           // 改名
    cv::Mat xAxisTrackDirection;          // 新增
    cv::Mat yAxisTrackDirection;          // 新增
    cv::Mat zAxisTrackDirection;          // 新增
    double xAxisReferenceEncoderValue = 0.0;// 新增
    double yAxisReferenceEncoderValue = 0.0;// 新增
};

struct ObjectInfo {
    ObjectYolo11Seg object;  // 检测到的物体
    cv::Point3d pt3d;        // 物体左上角的3D坐标
    cv::Point3d centerPt3d;  // 物体中心的3D坐标
    int validPixel;          // 物体的有效像素数量
    int cameraIndex;         // 相机序号
    double yAxisEncoderValue = std::numeric_limits<double>::quiet_NaN();
};

extern std::vector<cameraConfig> cameraParameters;  // 相机参数数量
using namespace CanvasDrawingConfig;

/**
 * @brief 工件坐标拟合类
 * @details 该类负责处理工件坐标的拟合计算和分类
 */
class FittingWorkpieceCoordinate : public QObject {
    Q_OBJECT
public slots:
    void whenFittingWorkpieceCoordinate(std::vector<segYolo11::ObjectYolo11Seg> objs, int imgNum, double yAxisEncoderValue);
    void whenFinishInferrence();
    void loadCalibrationParameters(const std::string &filename);
    void handleClickEvent(int x, int y);

public:
    FittingWorkpieceCoordinate();

    void Point2dto3d(std::vector<double> plane, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, std::vector<cv::Point2d> &Pt2ds,
                     std::vector<cv::Point3d> &Pt3ds);
    std::vector<cv::Point3d> transformCameraToBase(const std::vector<cv::Point3d> &cameraPoints, const cv::Mat &cameraToBaseMatrix);
    void saveAllObjectsToFile(std::string filePath);
    double calculateDistance(const cv::Point3d &p1, const cv::Point3d &p2);
    cv::Point3d computeCentroid(const std::vector<ObjectInfo> &group);
    std::vector<std::vector<ObjectInfo>> classifyWorkpieces(const std::vector<ObjectInfo> &allObjects, double threshold);
    void removeSmallCategories(std::vector<std::vector<ObjectInfo>> &categorizedObjects);
    std::vector<cv::Point3d> calculateCategoryCenters(std::vector<std::vector<ObjectInfo>> &categorizedObjects);
    void displayDetectedWorkpieces(const std::vector<std::vector<ObjectInfo>> &categorizedObjects, const std::vector<cv::Mat> &cvImagesToDisplay,
                                   const std::vector<cv::Point3d> &categoryCenters3d);

    void whenVerifyWorkpieceCoordinates();
    std::vector<cv::Point3d> pixel2WorldCoordPoint(std::vector<cv::Point2d> &Pt2ds, double yAxisEncoderValue);
    void drawGridAndAxes(cv::Mat &railMap);
    void drawDetectedWorkpieces(cv::Mat &railMap, const cv::Mat &resizedImage, cv::Point3d &worldCenter, int categoryIdx);
    void whenDisplayWeldSeamArea(workpieceBoxInWorld &boxInfo);
    void railMapRotated(cv::Mat &image, int angle);
    cv::Point2d projectAndRotateCenter(const cv::Point3d &center3d, const cv::Mat &canvasMat, int imageRows, int imageCols, int rotationAngle);
    // 获取结果
    void whenGetResultInfo(const std::vector<cv::Point3d> resultCenters, const std::vector<cv::Point3d> resultLeftTop);
    void whenGetWeldBoxInfo(const std::vector<std::vector<std::array<double, 4>>> &boxInfos);
    void computeIOUsWithOverlap(workpieceBoxInWorld &boxInfo);
    void sortWorkpieceBoxInfo(workpieceBoxInWorld &boxInfo, const std::string &axis, const std::string &order);
    void applyTrackOffsetCompensation(workpieceBoxInWorld &info);
    void computeBaseOffsetAndViewpointsFromMask(workpieceBoxInWorld &info);

private:
    //修改成三轴投影
    cv::Point3d computeTrackCompensation(const cv::Point3d &pt,const cv::Mat &xDir, const cv::Mat &yDir, const cv::Mat &zDir);
    cv::Point3d applyXAxisEncoderOffset(const cv::Point3d &pt, double yAxisEncoderValue);

private:
    std::vector<ObjectInfo> allObjects;
    // 世界坐标系下工件信息
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<double> plane;
    cv::Mat cameraToBaseMatrix;
    cv::Mat railMap;                                          // 长画布
    std::vector<std::vector<ObjectInfo>> categorizedObjects;  // 存储分类结果
    std::vector<cv::Point3d> categoryWorldCenters;            // 存储人工筛选前世界坐标下的中心点
    std::vector<cv::Point3d> categoryWorldLeftTopCenters;     // 存储人工筛选前世界坐标下的左上角点
    std::vector<cv::Point3d> filteredWorldCenters;            // 人工筛选后的工件中心点（世界坐标系）
    std::vector<cv::Point3d> filteredWorldTopLeftPoints;      // 人工筛选后的工件左上角点（世界坐标系）
    int distance = 100;                                       // 移动距离
    double threshold = 100.0;                                 // 工件坐标分类距离阈值
    int maxPixelCount = 3;                                    // 最多像素掩膜索取数（与类别尺寸取min）
    std::vector<int> selectedWorkpieces;                      // 存储被删除的工件索引
    std::vector<cv::Mat> worldMaskImages;                     // 存储世界坐标系下的工作掩膜图像
    std::vector<std::pair<cv::Rect, int>> workpieceROIs;      // 存储每个roi和它对应的类别索引
    float railPosition;                                       // 地轨位置
    std::string sortOrder;                                    // 工件顺序
    bool applyTrackCompensation = true;                       // 是否启用地轨方向向量补偿

signals:
    void appendFittingLog(QString message);             // 日志信号
    void sendWorkpieceResultToMainWindow(cv::Mat res);  // 发送信号用于显示图片
    void sendUpdateInferedCameraImgNum();               // 发送信号用于更新推理后相机图显示
    void sendFinalInfoToMain(const workpieceBoxInWorld &workpieceBoxInfoInWorld);
    void sendWorkpieceMaskImageInWorld(std::vector<cv::Point3d> worldCenters, std::vector<cv::Mat> worldMaskImages);

    friend class RailWeldingSystem;
    friend class WeldingMainWindow;
};

#endif  // FITTINGWORKPIECECOORDINATE_H
