#ifndef POINTCLOUDRECONSTRUCTION_H
#define POINTCLOUDRECONSTRUCTION_H

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
#include <QtConcurrent>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
class StructLightConfig;
class WeldSeamInfo;
class AbstractObjectDetect;
class DetResult;

class PointCloudReconstruction {
public:
    PointCloudReconstruction();

    // 点云重建函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr localReconstruct(int minU, int maxU, int minV, int maxV);  // 局部点云重建
    std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaReconstructToSA();  // 焊缝区域点云重建(加拟合背景平面)
    std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaReconstructToLW();  // 焊缝区域点云重建(加拟合背景平面)

    void initDistortionMap();

private:
    // 计算过程中需要用到的参数
    StructLightConfig& structLightConfig;
    const double myPi = 3.1415926535897932384626433832795;
    static const int cameraWidth = 1600;  // 相机分辨率
    static const int cameraHeight = 1200;
    static const int projectorWidth = 1280;  // 投影仪分辨率
    static const int projectorHeight = 720;
    static const int phaseShiftImgNum = 12;  // 相移步数
    const int garyCodeTotalImgNum = 9;   // 格雷码总图片数, 6个传统格雷码 + 2个全黑白 + 1个互补格雷码 = 9
    const int garyCodeEncodeImgNum = 6;  // 传统格雷码编解码的图像数量
    const double ransacPlaneThreshold = 2;      // Ransac拟合背景平面的距离阈值
    const double workPlaneRemoveThreshold = 2;  // 计算采集点云各点距离拟合平面的距离，大于此阈值，则保留
    double modulationThreshold = 2;             // 调制度阈值
    cv::Mat cameraDistortion;                   // 畸变系数集合 Opencv: k1, k2, p1, p2, k3
    cv::Mat projectorDistortion;                // 畸变系数集合 Opencv: k1, k2, p1, p2, k3
    double sinTable[phaseShiftImgNum];          // sin表
    double cosTable[phaseShiftImgNum];          // cos表
    cv::Mat cameraMapX;                         // 畸变表x
    cv::Mat cameraMapY;                         // 畸变表y

    // 初始化函数
    void initPara();  // 初始化需要用到的参数
#ifdef SMART_CAMERA
    void initObjectDetect();  // 初始化目标检测类
#endif
    void reInitialize();  //  变量重新初始化

    // 点云重建的具体计算步骤函数
    void imageDistribute();  // 0. 将采集到的图像放入相移和格雷码容器
    void makeMaskForReconstruct(int minU, int maxU, int minV, int maxV);  // 1.1 更新全点云重建的mask
    void makeMaskForSeamsDet();                                           // 1.2 更新焊缝区域目标框的mask
    void makeMaskForSeamsDetToLW();                                       // 1.2 更新焊缝区域目标框的mask
    void solveWrapPhase();                                                // 2. 相移法求包裹相位
    void decodeGrayCode();                                                // 3. 解码格雷码
    void phaseUnwrap();                                                   // 4. 相位展开, 求绝对相位
    void cameraProjectMatch(std::vector<cv::Point2d>& cameraCoord, std::vector<double>& projectCoord, int minU = 0,
                            int maxU = cameraWidth, int minV = 0, int maxV = cameraHeight);  // 5.0.1 相机和投影仪匹配对应点
    void calcPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<cv::Point2d>& cameraCoord,
                        std::vector<double>& projectCoord);                      // 5.0.2 计算点云
    void pointCloudPostProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);  // 5.0.3 点云后处理
#ifdef SMART_CAMERA
    void reconstructForWorkbench();  // 5.1 背景平面的三维重建, 拟合背景平面参数
    void reconstructForSeamArea();   // 5.2 目标检测框的三维重建, 填充到对应焊缝信息结构体
#endif
    void reconstructPoint();  // 5.3 点云三维重建

    // 计算前采集到的数据
    std::shared_ptr<std::vector<cv::Mat>> primaryCameraCapturedImg{nullptr};    // 主相机采集到的图像
    std::shared_ptr<std::vector<cv::Mat>> secondaryCameraCapturedImg{nullptr};  // 次相机采集到的图像
    std::vector<cv::Mat> phaseShiftImages;                                      // 相移图集合
    std::vector<cv::Mat> grayCodeImages;                                        // 格雷码图集合

    // 计算过程的中间量
    cv::Mat sinSum = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                     // sin和
    cv::Mat cosSum = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                     // cos和
    cv::Mat wrapPhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                  // 包裹相位
    cv::Mat modulation = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                 // 调制度
    cv::Mat K1 = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);                          // 每一像素的K1
    cv::Mat K2 = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);                          // 每一像素的K2
    cv::Mat binarizationThreshold = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);      // 每一像素的二值化阈值
    cv::Mat absolutePhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);              // 绝对相位
    cv::Mat unDistortionAbsolutePhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);  // 去除相机畸变后的绝对相位
    cv::Mat maskForReconstruct = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);  // 用于重建的掩模(每次重建时更新)
    cv::Mat maskForGlobal = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);  // 全局重建的掩模(初始化时更新一次)
#ifdef SMART_CAMERA
    cv::Mat maskForWorkbench =
        cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);  // 工作台重建的掩模(去除焊缝区域, 重建焊缝区域时更新)
    cv::Mat maskForWorkpiece =
        cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);  // 工件焊缝区域重建的掩模(只有焊缝区域, 重建焊缝区域时更新)
#endif

    // 计算结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;  // 重建点云
#ifdef SMART_CAMERA
    Eigen::VectorXf workbenchCoeff = Eigen::VectorXf::Ones(4);  // 工作台的背景平面参数
    pcl::PointCloud<pcl::PointXYZ>::Ptr workbenchPointCloud;    // 工作台点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonPlanePointCloud;     // 工件点云
    std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo;    // 焊缝区域信息
#endif

    // clang-format off
    std::map<uchar, uchar> VK1 {  // 格雷码对应表
        {0,  0 }, {1,  1 }, {3,  2 }, {2,  3 }, {6,  4 }, {7,  5 }, {5,  6 }, {4,  7 }, {12, 8 }, {13, 9 }, {15, 10}, {14, 11}, {10, 12}, {11, 13}, {9,  14}, {8,  15},
        {24, 16}, {25, 17}, {27, 18}, {26, 19}, {30, 20}, {31, 21}, {29, 22}, {28, 23}, {20, 24}, {21, 25}, {23, 26}, {22, 27}, {18, 28}, {19, 29}, {17, 30}, {16, 31},
        {48, 32}, {49, 33}, {51, 34}, {50, 35}, {54, 36}, {55, 37}, {53, 38}, {52, 39}, {60, 40}, {61, 41}, {63, 42}, {62, 43}, {58, 44}, {59, 45}, {57, 46}, {56, 47},
        {40, 48}, {41, 49}, {43, 50}, {42, 51}, {46, 52}, {47, 53}, {45, 54}, {44, 55}, {36, 56}, {37, 57}, {39, 58}, {38, 59}, {34, 60}, {35, 61}, {33, 62}, {32, 63}
    };
    std::map<uchar, uchar> VK2 {  // 互补格雷码对应表
        {0,   0 }, {1,   1 }, {3,   1 }, {2,   2 }, {6,   2 }, {7,   3 }, {5,   3 }, {4,   4 }, {12,  4 }, {13,  5 }, {15,  5 }, {14,  6 }, {10,  6 }, {11,  7 }, {9,   7 }, {8,   8 },
        {24,  8 }, {25,  9 }, {27,  9 }, {26,  10}, {30,  10}, {31,  11}, {29,  11}, {28,  12}, {20,  12}, {21,  13}, {23,  13}, {22,  14}, {18,  14}, {19,  15}, {17,  15}, {16,  16},
        {48,  16}, {49,  17}, {51,  17}, {50,  18}, {54,  18}, {55,  19}, {53,  19}, {52,  20}, {60,  20}, {61,  21}, {63,  21}, {62,  22}, {58,  22}, {59,  23}, {57,  23}, {56,  24},
        {40,  24}, {41,  25}, {43,  25}, {42,  26}, {46,  26}, {47,  27}, {45,  27}, {44,  28}, {36,  28}, {37,  29}, {39,  29}, {38,  30}, {34,  30}, {35,  31}, {33,  31}, {32,  32},
        {96,  32}, {97,  33}, {99,  33}, {98,  34}, {102, 34}, {103, 35}, {101, 35}, {100, 36}, {108, 36}, {109, 37}, {111, 37}, {110, 38}, {106, 38}, {107, 39}, {105, 39}, {104, 40},
        {120, 40}, {121, 41}, {123, 41}, {122, 42}, {126, 42}, {127, 43}, {125, 43}, {124, 44}, {116, 44}, {117, 45}, {119, 45}, {118, 46}, {114, 46}, {115, 47}, {113, 47}, {112, 48},
        {80,  48}, {81,  49}, {83,  49}, {82,  50}, {86,  50}, {87,  51}, {85,  51}, {84,  52}, {92,  52}, {93,  53}, {95,  53}, {94,  54}, {90,  54}, {91,  55}, {89,  55}, {88,  56},
        {72,  56}, {73,  57}, {75,  57}, {74,  58}, {78,  58}, {79,  59}, {77,  59}, {76,  60}, {68,  60}, {69,  61}, {71,  61}, {70,  62}, {66,  62}, {67,  63}, {65,  63}, {64,  64}
    };
    // clang-format on

    // 目标检测模型相关参数
    /* 由于焊缝区域检测所需的图片要等投影仪触发相机采图获取, 为减少数据交互次数和代码逻辑复杂度,
       将目标检测模型集成在结构光相机端进行推理, 采用条件编译的方式便于结构光相机模块在其他项目中集成 */
#ifdef SMART_CAMERA
    // 模型路径
#if defined(ROM_CONFIG) || defined(LI_CONFIG)
    const std::string objDetEnginePath = "./data/DL_models/objDec/objDecRom.engine";
#elif GONG_RAIL_CONFIG
    const std::string objDetEnginePath = "./data/DL_models/objDec/objDecRail.engine";
#elif A17_CONFIG
    const std::string objDetEnginePath = "./data/DL_models/objDec/objDec.engine";
#endif
    // 类别名称和颜色
    std::vector<std::string> classNames = {"BackCorner", "FrontCorner", "FrontDownBeam", "BackBeam", "FrontUpBeam"};
    std::vector<std::vector<unsigned int>> colors = {
        {0,   114, 189},
        {217, 83,  25 },
        {237, 177, 32 },
        {126, 47,  142},
        {119, 172, 48 }
    };
    double scoreThreshold = 0.25;
    double iouThreshold = 0.25;
    double labelNum = 5;
    int imgSize = 1024;
    // 计算过程需要用到的工具类
    std::shared_ptr<AbstractObjectDetect> weldsCoarsePosition{nullptr};  // 目标检测算法类
    std::shared_ptr<std::vector<DetResult>> detRes{nullptr};             // 目标检测结果
#endif

    friend class StructLightCamera;
};

#endif  // POINTCLOUDRECONSTRUCTION_H
