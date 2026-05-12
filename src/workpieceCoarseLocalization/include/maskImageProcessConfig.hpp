#ifndef MASKIMAGEPROCESSCONFIG_H
#define MASKIMAGEPROCESSCONFIG_H
#include <plog/Log.h>

#include <opencv2/opencv.hpp>

#include "deepLearning/objectDetect/yolo11/common.hpp"
#include "deepLearning/segment/yolo11/common.hpp"
#include "robotFactory/AbstractRobot.h"

struct workpieceIOUInfo {
    cv::Mat cameraOriginalMat;                                                           // 相机原图
    cv::Mat cameraSegMat;                                                                // 原图下的掩膜检测
    std::pair<cv::Mat, cv::Mat> workpiece_weld_Mask;                                     // 焊缝检测前后的掩膜图像
    std::pair<segYolo11::ObjectYolo11Seg, std::vector<det::Object>> workpiece_weld_Obj;  // 检测数据
};

struct workpieceInfo {
    cv::Mat cameraOriginalMat;                                                           // 相机原图
    cv::Mat cameraSegMat;                                                                // 原图下的掩膜检测
    std::pair<cv::Mat, cv::Mat> workpiece_weld_Mask;                                     // 焊缝检测前后的掩膜图像
    std::pair<segYolo11::ObjectYolo11Seg, std::vector<det::Object>> workpiece_weld_Obj;  // 检测数据
    std::pair<cv::Point3d, cv::Point3d> workpieceAreaRect;                               // <center, topleft>
    std::vector<cv::Rect_<double>> weldAreaRect;                                         // 工件焊缝区域信息
    std::vector<cv::Point3d> photoPos;                                                   // 单一工件多次拍照位置
    std::vector<std::vector<cv::Rect_<double>>> rectOfPhotoPos;                          // 每个拍照位置对应的目标检测框
    std::vector<workpieceIOUInfo> workpieceIouInfos;
    robotPose robotViewPose;  // 机器人视点位姿
};

struct workpieceBoxInWorld {
    std::vector<workpieceInfo> workpieceInfoInWorld;
    cv::Mat trackDirection;  // 地轨方向向量
    cv::Mat finalRailMap;    // 最终长图
};

extern int cameraIndex;
extern std::string inferencePath;   // 推理路径
extern std::string configFilePath;  // 配置保存路径
extern std::string trackFilePath;
extern std::vector<cv::Mat> cvImagesCameraOri;
extern std::vector<cv::Mat> cvImagesWorkpieceSeg;
extern std::vector<cv::Mat> cvImagesWpMaskOri;
extern std::vector<cv::Mat> cvImagesWpMaskDet;
extern workpieceBoxInWorld workpieceFinalInfoInWorld;
extern workpieceBoxInWorld workpieceFinalInfoInWorldAfterVerify;
extern workpieceBoxInWorld workpieceFinalInfoInWorldAfterIOU;
extern std::string workbenchInsertGroup;
//------------------------------画布绘制参数---------------------------------------
namespace CanvasDrawingConfig {
// 长画布参数
const int pixelRow = 4000;                   // 原始画布高度
const int pixelCol = 6000;                   // 原始画布宽度
const int correctLineThickness = 20;         // 正常线厚度
const int warningLineThickness = 20;         // 警告线厚度
const int deleteLineThickness = 20;          // 删除线厚度
const int weldSeamLineThickness = 2;         // 焊缝区线厚度
const int gridSpacingX = 100;                // 坐标轴X间距
const int gridSpacingY = 100;                // 坐标轴Y间距
const int axisThickness = 2;                 // 坐标轴厚度
const cv::Scalar axisColor(0, 0, 0);         // 坐标轴颜色
const cv::Scalar correctColor(0, 255, 0);    // 正确颜色
const cv::Scalar warningColor(255, 125, 0);  // 警告颜色
const cv::Scalar deleteColor(255, 0, 0);     // 删除颜色
const cv::Scalar weldSeamColor(0, 0, 0);     // 焊缝区域颜色

const cv::Mat canvasMat = (cv::Mat_<double>(3, 3) << 1, 0, 100, 0, -1, pixelRow / 2, 0, 0, 1);  // 绘制坐标系偏移
const int railMapRotationAngle = 0;                                                             // 0 90 180 270  画布最后可视化的角度
const std::string sortWorldAxis = "X";                                                          // 世界坐标系下排序
const std::string sortWorldOrder = "up";                                                        // 世界坐标系下排序
const float iouThreshold = 0.0;                                                                 // 工件IOU合并阈值
const int wpIOUSearchNum = 6;                                                                   // 工件IOU搜索最大范围±
const bool standardOutput = true;                                                               // 强制规范南北工作台输出
}  // namespace CanvasDrawingConfig

//------------------------------推理掩膜变换---------------------------------------
namespace MaskTransformConfig {
const std::string segSavePath = "./data/workpieceCoaLoc/infer";
const std::string detSavePath = "./data/workpieceCoaLoc/maskInfer";
const int expandedWidth = 1024;  // 定义扩展后的画布大小(粗定位焊缝推理)
const int expandedHeight = 1024;
const int expendAdaptability = 1.3;
const int AdjustWorkpieceResolution = 1;  // 调整工件掩膜分辨率倍数
const int rectRotationAngle = 0;          // 90 180 270 //旋转工件提高召回率
const std::string sortAxis = "X";
const std::string sortOrder = "up";
const bool saveEveryImg = false;  // 每次点击都保存图像结果
}  // namespace MaskTransformConfig

class CoordinateMapper {
public:
    // 坐标映射类型枚举
    enum class CoordMappingType {
        XY,         // x->x, y->y
        NegX_Y,     // x->-x, y->y
        X_NegY,     // x->x, y->-y
        NegX_NegY,  // x->-x, y->-y
        YX,         // x->y, y->x
        NegY_X,     // x->-y, y->x
        Y_NegX,     // x->y, y->-x
        NegY_NegX   // x->-y, y->-x
    };

    // 映射函数：将 rel 从像素坐标变换到世界坐标系
    static cv::Point2d relativeMapToCoord(const cv::Point2d& rel, const cv::Point2d& worldCenter, CoordMappingType type) {
        double dx = rel.x;
        double dy = rel.y;
        double x = worldCenter.x;
        double y = worldCenter.y;

        switch (type) {
            case CoordMappingType::XY:
                return {x + dx, y + dy};
            case CoordMappingType::NegX_Y:
                return {x - dx, y + dy};
            case CoordMappingType::X_NegY:
                return {x + dx, y - dy};
            case CoordMappingType::NegX_NegY:
                return {x - dx, y - dy};
            case CoordMappingType::YX:
                return {x + dy, y + dx};
            case CoordMappingType::NegY_X:
                return {x - dy, y + dx};
            case CoordMappingType::Y_NegX:
                return {x + dy, y - dx};
            case CoordMappingType::NegY_NegX:
                return {x - dy, y - dx};
            default:
                return {x + dx, y + dy};  // 默认情况：不做变换
        }
    }
    template <typename T>
    static cv::Point_<T> rotateToOriginal(const cv::Point_<T>& rotatedPoint, int originalImageWidth, int originalImageHeight, int rotationAngle) {
        T x_rotated = rotatedPoint.x;
        T y_rotated = rotatedPoint.y;
        T x_original = 0;
        T y_original = 0;

        switch (rotationAngle) {
            case 0:
                x_original = x_rotated;
                y_original = y_rotated;
                break;
            case 90:
                x_original = y_rotated;
                y_original = static_cast<T>(originalImageWidth - x_rotated - 1);
                break;
            case 180:
                x_original = static_cast<T>(originalImageWidth - x_rotated - 1);
                y_original = static_cast<T>(originalImageHeight - y_rotated - 1);
                break;
            case 270:
                x_original = static_cast<T>(originalImageHeight - y_rotated - 1);
                y_original = x_rotated;
                break;
            default:
                x_original = x_rotated;
                y_original = y_rotated;
                break;
        }

        return cv::Point_<T>(x_original, y_original);
    }
    template <typename T>
    static cv::Point_<T> originalToRotated(const cv::Point_<T>& originalPoint, int originalImageWidth, int originalImageHeight, int rotationAngle) {
        T x_original = originalPoint.x;
        T y_original = originalPoint.y;
        T x_rotated = 0;
        T y_rotated = 0;

        switch (rotationAngle) {
            case 0:
                x_rotated = x_original;
                y_rotated = y_original;
                break;
            case 90:
                x_rotated = static_cast<T>(originalImageWidth - y_original - 1);
                y_rotated = x_original;
                break;
            case 180:
                x_rotated = static_cast<T>(originalImageWidth - x_original - 1);
                y_rotated = static_cast<T>(originalImageHeight - y_original - 1);
                break;
            case 270:
                x_rotated = y_original;
                y_rotated = static_cast<T>(originalImageHeight - x_original - 1);
                break;
            default:
                x_rotated = x_original;
                y_rotated = y_original;
                break;
        }

        return cv::Point_<T>(x_rotated, y_rotated);
    }
};
extern CoordinateMapper::CoordMappingType g_coordMappingType;  // 机器人与像素坐标系之间的关系

#endif  // MASKIMAGEPROCESSCONFIG_H
