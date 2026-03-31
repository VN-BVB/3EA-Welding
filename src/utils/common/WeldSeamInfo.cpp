#include "WeldSeamInfo.h"

WeldSeamInfo::WeldSeamInfo() {
    weldAreaPointCloudInCamera.reset();
    weldAreaPointCloudInRobot.reset();
    cloudFuture = QFuture<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
}

// 浅拷贝赋值重载
WeldSeamInfo& WeldSeamInfo::operator=(const WeldSeamInfo& other) noexcept {
    if (this != &other) {
        // 基本类型和枚举类型直接赋值
        areaNum = other.areaNum;
        detectSuccFlag = other.detectSuccFlag;
        width = other.width;
        weldType = other.weldType;
        weldAreaType = other.weldAreaType;

        // OpenCV矩阵, 增加引用计数
        originalImg = other.originalImg;
        weldAreaImg = other.weldAreaImg;
        segResultImg = other.segResultImg;
        segMaskImg = other.segMaskImg;

        // PCL成员, 增加引用计数
        weldAreaPointCloudInCamera = other.weldAreaPointCloudInCamera;
        weldAreaPointCloudInRobot = other.weldAreaPointCloudInRobot;
        weldPlane = other.weldPlane;
        seamsLineToVal = other.seamsLineToVal;

        // 共享智能指针, 增加引用计数
        weldEndPointsInCamera = other.weldEndPointsInCamera;
        weldEndPointsInRobot = other.weldEndPointsInRobot;
        weldEndPointsFromSeg = other.weldEndPointsFromSeg;
        swingReferencePoints = other.swingReferencePoints;
        rectPtr = other.rectPtr;
        //
        otherSurface = other.otherSurface;
        robotWeldPose = other.robotWeldPose;
    }
    return *this;
}

// 深拷贝克隆函数
std::shared_ptr<WeldSeamInfo> WeldSeamInfo::clone() const {
    auto copy = std::make_shared<WeldSeamInfo>();

    // 基本类型复制
    copy->areaNum = areaNum;
    copy->detectSuccFlag = detectSuccFlag;
    copy->weldType = weldType;
    copy->weldAreaType = weldAreaType;
    copy->width = width;

    // OpenCV矩阵深拷贝
    copy->originalImg = originalImg.clone();
    copy->weldAreaImg = weldAreaImg.clone();
    copy->segResultImg = segResultImg.clone();
    copy->segMaskImg = segMaskImg.clone();

    if (rectPtr) {  // 矩形框深拷贝
        copy->rectPtr = std::make_shared<cv::Rect_<float>>(*rectPtr);
    }

    if (weldEndPointsInCamera) {  // 端点数据深拷贝
        copy->weldEndPointsInCamera = std::make_shared<std::vector<pcl::PointXYZ>>(*weldEndPointsInCamera);
    }

    if (weldEndPointsInRobot) {  // 端点数据深拷贝
        copy->weldEndPointsInRobot = std::make_shared<std::vector<pcl::PointXYZ>>(*weldEndPointsInRobot);
    }

    if (weldEndPointsFromSeg) {  // 端点数据深拷贝
        copy->weldEndPointsFromSeg = std::make_shared<std::vector<pcl::PointXYZ>>(*weldEndPointsFromSeg);
    }

    if (weldAreaPointCloudInCamera) {  // 点云数据深拷贝
        copy->weldAreaPointCloudInCamera = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*weldAreaPointCloudInCamera, *copy->weldAreaPointCloudInCamera);
    }

    if (weldAreaPointCloudInRobot) {  // 焊缝区域点云机器人深拷贝
        copy->weldAreaPointCloudInRobot = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*weldAreaPointCloudInRobot, *copy->weldAreaPointCloudInRobot);
    }

    if (weldPlane) {  // 平面参数深拷贝
        copy->weldPlane = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*weldPlane));
    }

    if (seamsLineToVal) {  // 验证直线深拷贝
        copy->seamsLineToVal = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*seamsLineToVal));
    }

    // 其他母材表面深拷贝
    copy->otherSurface.clear();
    for (const auto& surface : otherSurface) {
        if (surface) {
            copy->otherSurface.push_back(pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*surface)));
        }
    }

    // 机器人位姿深拷贝
    copy->robotWeldPose = robotWeldPose;
    copy->swingReferencePoints = swingReferencePoints;

    return copy;
}

// 获取焊缝区域类型
WELD_AREA_TYPE MyToolFunc::getWeldAreaType(int areaNum) {
    switch (areaNum) {
        case 0:
            return WELD_AREA_TYPE::BACK_CORNER;
        case 1:
            return WELD_AREA_TYPE::FRONT_CORNER;
        case 2:
            return WELD_AREA_TYPE::FRONT_DOWN_BEAM;
        case 3:
            return WELD_AREA_TYPE::BACK_BEAM;
        case 4:
            return WELD_AREA_TYPE::FRONT_UP_BEAM;
        case 100:
            return WELD_AREA_TYPE::Plate_Plate_F;
        case 101:
            return WELD_AREA_TYPE::TubeSide_Plate_F;
        case 102:
            return WELD_AREA_TYPE::Tube_Plate_F;
        case 103:
            return WELD_AREA_TYPE::Tube_Tube_F;
        default:
            return WELD_AREA_TYPE::Default;
    }
}

// 获取焊缝区域类型字符串
std::string MyToolFunc::getWeldAreaTypeString(WELD_AREA_TYPE weldAreaType) {
    switch (weldAreaType) {
        case WELD_AREA_TYPE::BACK_CORNER:
            return "BACK_CORNER";
        case WELD_AREA_TYPE::FRONT_CORNER:
            return "FRONT_CORNER";
        case WELD_AREA_TYPE::FRONT_DOWN_BEAM:
            return "FRONT_DOWN_BEAM";
        case WELD_AREA_TYPE::BACK_BEAM:
            return "BACK_BEAM";
        case WELD_AREA_TYPE::FRONT_UP_BEAM:
            return "FRONT_UP_BEAM";
        case WELD_AREA_TYPE::Plate_Plate_F:
            return "Plate_Plate_F";
        case WELD_AREA_TYPE::TubeSide_Plate_F:
            return "TubeSide_Plate_F";
        case WELD_AREA_TYPE::Tube_Plate_F:
            return "Tube_Plate_F";
        case WELD_AREA_TYPE::Tube_Tube_F:
            return "Tube_Tube_F";
        case WELD_AREA_TYPE::Default:
            return "Default";
        default:
            return "unknown type";
    }
}

// 获取焊缝类型字符串
std::string MyToolFunc::getWeldTypeString(WELD_TYPE weldType) {
    switch (weldType) {
        case WELD_TYPE::BACK_CORNER_BUTT:
            return "BACK_CORNER_BUTT";
        case WELD_TYPE::FRONT_CORNER_BUTT:
            return "FRONT_CORNER_BUTT";
        case WELD_TYPE::BACK_BEAM_BUTT:
            return "BACK_BEAM_BUTT";
        case WELD_TYPE::FRONT_BEAM_BUTT:
            return "FRONT_BEAM_BUTT";
        case WELD_TYPE::FRONT_HORIZONTAL_FILLET:
            return "FRONT_HORIZONTAL_FILLET";
        case WELD_TYPE::FRONT_VERTICAL_FILLET:
            return "FRONT_VERTICAL_FILLET";
        case WELD_TYPE::Plate_Plate_Fillet_V:
            return "Plate_Plate_Fillet";
        case WELD_TYPE::TubeSide_Plate_F_H:
            return "TubeSide_Plate_Fillet";
        case WELD_TYPE::Tube_Plate_Fillet:
            return "Tube_Plate_Fillet";
        case WELD_TYPE::Tube_Tube_Fillet:
            return "Tube_Tube_Fillet";
        default:
            return "unknown type";
    }
}
