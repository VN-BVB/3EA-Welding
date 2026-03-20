#ifndef WELDSEAMINFO_H
#define WELDSEAMINFO_H

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
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "robotFactory/AbstractRobot.h"
enum WELD_TYPE {              // 焊缝类型
    BACK_CORNER_BUTT,         // 背面边角对接
    FRONT_CORNER_BUTT,        // 正面边角对接
    BACK_BEAM_BUTT,           // 背面横梁对接
    FRONT_BEAM_BUTT,          // 正面横梁对接
    FRONT_HORIZONTAL_FILLET,  // 正面水平角接
    FRONT_VERTICAL_FILLET,    // 正面垂直角接
    // 大型工件
    Plate_Plate_Fillet = 100,
    TubeSide_Plate_F_H = 101,  // 管侧与板角接水平焊缝
    Tube_Plate_Fillet = 102,
    Tube_Tube_Fillet = 103
};
enum WORKPIECE_TYPE {
    STEEL_ANGLE,     // 角钢
    LARGE_WORKPIECE  // 大型工件
};
enum WELD_AREA_TYPE {     // 焊缝区域类型
                          // 角钢
    BACK_CORNER = 0,      // 背面边角
    FRONT_CORNER = 1,     // 正面边角
    FRONT_DOWN_BEAM = 2,  // 正面倒立横梁
    BACK_BEAM = 3,        // 背面横梁
    FRONT_UP_BEAM = 4,    // 正面正立横梁
                          // 大型工件
    Plate_Plate_F = 100,
    TubeSide_Plate_F = 101,
    Tube_Plate_F = 102,
    Tube_Tube_F = 103,
    Default
};

enum SEAM_SIDE {
    // 焊缝位置
    FRONT,  // 正面
    BACK,   // 反面
    LEFT,   // 左侧
    RIGHT   // 右侧
};
// 起弧熄弧动作控制
enum ARC_ACTION {
    ARC_START = 1,  // 起弧
    ARC_STOP = 0    // 熄弧
};
namespace MyToolFunc {

WELD_AREA_TYPE getWeldAreaType(int areaNum);                     // 获取焊缝区域类型
std::string getWeldAreaTypeString(WELD_AREA_TYPE weldAreaType);  // 获取焊缝区域类型字符串
std::string getWeldTypeString(WELD_TYPE weldType);               // 获取焊缝类型字符串

}  // namespace MyToolFunc

class WeldSeamInfo {
public:
    WeldSeamInfo();
    WeldSeamInfo& operator=(const WeldSeamInfo& other) noexcept;  // 浅拷贝赋值重载
    std::shared_ptr<WeldSeamInfo> clone() const;                  // 深拷贝克隆函数

    // 焊缝区域信息
    int areaNum = -1;                                        // 区域编号
    cv::Mat originalImg;                                     // 原始图像
    cv::Mat weldAreaImg;                                     // 焊缝区域图像
    std::shared_ptr<cv::Rect_<float>> rectPtr;               // 焊缝区域矩形框
    pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloud;  // 焊缝区域点云
    WELD_AREA_TYPE weldAreaType;                             // 焊缝区域类型

    // 焊缝信息
    bool detectSuccFlag = false;                                        // 焊缝检测成功标志
    std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInCamera;  // 焊缝端点 (相机坐标系下)
    std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInRobot;   // 焊缝端点 (机器人坐标系下)
    pcl::ModelCoefficients::Ptr weldPlane;                              // 焊缝所在平面
    std::vector<pcl::ModelCoefficients::Ptr> otherSurface;              // 其他母材表面
    WELD_TYPE weldType;                                                 // 焊缝类型
    pcl::ModelCoefficients::Ptr seamsLineToVal;                         // 焊缝验证直线

    // 分割相关
    std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsFromSeg;  // 焊缝端点 (相机坐标系下分割结果)
    cv::Mat segResultImg;                                              // 分割结果图像
    cv::Mat segMaskImg;                                                // 分割掩膜图像
    double width = 1.5;                                                // 焊缝宽度

    // 机器人位姿
    std::vector<robotPose> robotWeldPose;
};

#endif  // WELDSEAMINFO_H
