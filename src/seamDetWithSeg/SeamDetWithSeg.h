#ifndef SEAMDETWITHSEG_H
#define SEAMDETWITHSEG_H

#include <direct.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <plog/Log.h>

#include <QObject>
#include <algorithm>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "deepLearning/segment/AbstractSegment.h"
#include "utils/common/ThreadPool.h"

class WeldSeamInfo;
class AbstractSegment;
class StructLightConfig;

// 平面类
class Plane {  // 采用的平面方程: Ax + By + Cz = 1;
public:
    Plane();
    Plane(double a, double b, double c) : A(a), B(b), C(c) {}

    void initPara();
    void setPlanePara(double a, double b, double c);

    double A = 0;
    double B = 0;
    double C = 0;
};

class SeamDetWithSeg : public QObject {
    Q_OBJECT
public:
    explicit SeamDetWithSeg(QObject* parent = nullptr);

    void initSegment();  // 初始化分割类
    void initPara();     // 初始化参数

    void drawMaskToBackground(const SegResult& res, cv::Mat& output);      // 将分割结果绘制到和原图一样大的背景
    void detectSignalSeamWithSeg(std::shared_ptr<WeldSeamInfo> seamInfo);  // 使用分割方法计算单一焊缝
    bool point2dTo3d(std::vector<cv::Point>& pixelSet, pcl::PointCloud<pcl::PointXYZ>::Ptr& seamPointsSet,
                     Plane& p);  // 二维像素映射到三维点

    void fusionPointCloudAndSegRes(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 融合点云和分割的计算结果

signals:
    void sendDetSeamWithSeg(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 发出求解完成的焊缝

public slots:
    void whenDetSeamWithSeg(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 求解焊缝

private:
    // 计算过程中需要用到的参数
    StructLightConfig& structLightConfig;
    double fx = 0;  // 相机内参
    double fy = 0;
    double u0 = 0;
    double v0 = 0;
    int imgNum = 0;         // 焊缝图像序号
    int pointCloudNum = 0;  // 焊缝点云序号

    // 实例分割模型路径
#if defined(ROM_CONFIG) || defined(LI_CONFIG)
    const std::string segEnginePath = "./data/DL_models/segment/segmentRom.engine";
#elif GONG_RAIL_CONFIG
    const std::string segEnginePath = "./data/DL_models/segment/segmentRail.engine";
#elif A17_CONFIG
    const std::string segEnginePath = "./data/DL_models/segment/segment.engine";
#elif defined(ZHANG_CONFIG)
    const std::string segEnginePath = "./data/DL_models/segment/segmentRom.engine";
#endif

    // 计算过程需要用到的工具类
    std::shared_ptr<AbstractSegment> weldsSegmentation{nullptr};  // 分割算法类
    std::vector<SegResult> segRes;                                // 分割结果

    ThreadPool* threadPool = new ThreadPool(4);  // 线程池
};

#endif  // SEAMDETWITHSEG_H
