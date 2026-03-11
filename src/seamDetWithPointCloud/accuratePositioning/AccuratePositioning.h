#ifndef ACCURATEPOSITIONING_H
#define ACCURATEPOSITIONING_H

#include <QObject>

#include <plog/Log.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d_omp.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <unordered_set>

#include <omp.h>

enum class WeldType {
    PlateToPlate,    // 板板焊接
    TubeToPlate1,    // 管板焊接1
    TubeToPlate2,    // 管板焊接2
    TubeToTube       // 管管焊接
};

// 三维点结构体，用于表示空间中的点坐标
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

// NURBS曲线结构体，用于表示非均匀有理B样条曲线
struct NURBSCurve {
    int degree;           // 曲线阶数
    std::vector<double> knots;    // 节点向量，定义曲线的参数化
    std::vector<float> weights;     // 权重向量，控制控制点的影响程度
    std::vector<Point3D> controlPoints;    // 控制点序列，定义曲线形状
};

// 焊枪位姿结构体，描述焊接过程中焊枪的位置和方向
struct Posture {
    std::vector<Point3D> position;       // 焊枪位置
    std::vector<std::vector<pcl::ModelCoefficients>> adjacentSurfaces;    // 相邻表面系数（如平面、圆柱面）
    std::vector<Point3D> tangentVectors;       // 切线向量，表示焊接方向
    std::vector<Eigen::Quaternionf> toolPose;      // 焊枪姿态（四元数表示旋转）
    std::vector<Point3D> x;      // 焊缝坐标系X轴方向
    std::vector<Point3D> y;      // 焊缝坐标系Y轴方向
    std::vector<Point3D> z;      // 焊缝坐标系Z轴方向
    Posture() {}

    Posture(const std::vector<Point3D>& pos,
            const std::vector<std::vector<pcl::ModelCoefficients>>& surfaces,
            const std::vector<Eigen::Quaternionf>& pose,
            const std::vector<Point3D>& tangents = {})
        : position(pos), adjacentSurfaces(surfaces), toolPose(pose), tangentVectors(tangents) {}
};

// 精定位可视化结果结构体，存储精定位计算的结果
struct PositioningResult {
    bool success;          // 定位是否成功
    WeldType weldType;        // 焊缝类型（如管管焊缝、板板焊缝等）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult;        // 处理后的点云数据
    std::vector<std::vector<pcl::PointXYZ>> extremePoints;    // 端点位置
    std::vector<Point3D> generatePoints;       // NURBS曲线生成的点序列，用于NURBS曲线的可视化
    std::vector<Point3D> generatePoints2;      // NURBS曲线生成的点序列，用于NURBS曲线的可视化
    pcl::ModelCoefficients cylinderCoefficients;             // 圆柱面拟合系数
    pcl::ModelCoefficients cylinderCoefficients2;            // 圆柱面拟合系数

    std::vector<std::vector<Point3D>> position;             // 多段焊缝焊缝点的位置序列
    std::vector<std::vector<Eigen::Quaternionf>> toolPose;  // 对应的焊枪姿态
    std::vector<std::vector<Point3D>> x;                 // 焊缝坐标系X轴方向向量序列
    std::vector<std::vector<Point3D>> y;                 // 焊缝坐标系Y轴方向向量序列
    std::vector<std::vector<Point3D>> z;                 // 焊缝坐标系Z轴方向向量序列

    PositioningResult()
        : cloudResult(new pcl::PointCloud<pcl::PointXYZ>) {}
};

Q_DECLARE_METATYPE(PositioningResult)

class AccuratePositioning : public QObject
{
    Q_OBJECT
public:
    explicit AccuratePositioning(QObject *parent = nullptr);

private:

private:
    //提取点云中某一点半径R内邻域的点集
    static std::vector<pcl::PointXYZ> extractNeighbors(const pcl::PointXYZ& point, double radius, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    //得到协方差矩阵
    static Eigen::Matrix3d computeCovariance(std::vector<double> xVector, std::vector<double> yVector, std::vector<double> zVector);

    //计算LOBB
    static inline void computeLOBBFromNeighbors(const std::vector<pcl::PointXYZ>& neighbors, double& l, double& w, double& h);

    //计算FDN
    static inline std::vector<pcl::PointXYZ> buildFDN(const pcl::PointXYZ& p, const std::vector<pcl::PointXYZ>& seed, const std::vector<pcl::PointXYZ>& candidates, double tau_d, double tau_r);

    //计算中心偏移量和平整度
    static void calculateF(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<double>& fVector, double radius);

    //非线性激活
    static void nonlinearActivation(std::vector<double> vector, std::vector<double>& activatedVector);

    //k-means++聚类中心初始化
    static std::vector<double> kmeansPlusPlusInit(const std::vector<double>& data, int k);

    //k-means++算法
    static std::vector<int> kmeansPP(const std::vector<double> &data, int k, int maxIter, std::vector<int> &clusteringCounts);

    //提取特征点
    static void compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& creaseCloud, double radius);

    //均匀降采样
    void pointcloudUniformDownsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leafSize, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //体素降采样
    void pointcloudVoxelDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //RANSAC平面拟合
    void planeFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudResult,  std::vector<pcl::ModelCoefficients> &planeEquations, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &planeClouds, int maxPlanes);

    //L1Median细化
    void L1Median(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudInput, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudOutput, int maxIterations = 5);

    //计算两点之间的距离
    double calculateDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2);

    //找到距离最远的两个内点
    void findFarthestPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers, pcl::PointXYZ& farthestPoint1, pcl::PointXYZ& farthestPoint2);

    //点投影到线上
    void pointcloudProjectLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //提取直线
    void extractLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudResult, std::vector<std::vector<pcl::PointXYZ>>& extremePoint, std::vector<pcl::ModelCoefficients>& modelCoefficients, int maxLines);

    //平面求相交直线并计算端点
    void computePlaneIntersectionLines(const std::vector<pcl::ModelCoefficients>& planeEquations, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planeClouds, std::vector<Posture>& postures, std::vector<pcl::ModelCoefficients>& lineEquations, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudResult);

    //拟合平面和两个圆柱面
    void fitPlaneAnd2Cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients& planeCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers, std::vector<pcl::ModelCoefficients>& cylinderCoefficients);

    //拟合两个圆柱面
    void fit2Cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::ModelCoefficients>& cylinderCoefficients, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers);

    //拟合NURBS曲线
    NURBSCurve NURBSfitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult, int p, int n, std::vector<float> weights);

    //计算点到直线的距离
    double distanceToSegment(const Point3D& p, const Point3D& a, const Point3D& b);

    //计算NURBS曲线上的点坐标
    Point3D calculateNURBSPoint(double t, NURBSCurve nurbsCurve);

    //NURBS曲线等弓高误差变步长法离散化
    std::vector<Point3D> discretizeNURBSCurve(double hMax, double tStart, double tEnd, double initialStep, NURBSCurve nurbsCurve);

    //求圆柱面和平面的交点
    void computeCylinderPlaneIntersection(const pcl::ModelCoefficients& planeCoefficients, const pcl::ModelCoefficients& cylinderCoefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& result, pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud);

    //求两圆柱面的交点
    void compute2CylinderIntersection(const std::vector<pcl::ModelCoefficients>& cylinderCoefficients, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //拟合一个平面和一个圆柱面
    void fitPlaneAndCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients& planeCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers, pcl::ModelCoefficients& cylinderCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderInliers);

    //求圆柱面和平面的交点
    void computefitPlaneAndCylinderIntersection(const pcl::ModelCoefficients& planeCoefficients, const pcl::ModelCoefficients& cylinderCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //欧式聚类去除噪点
    void euclideanClusteringRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //提取圆柱边缘
    void extractCylinderBoundary(const pcl::ModelCoefficients& cylinderCoefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderInliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);

    //NURBS曲线径向投影
    NURBSCurve projectNURBSCurveRadially(const NURBSCurve& curve, const pcl::ModelCoefficients& cylinderCoefficients, double distance);

    //将点云从相机坐标系转换到基座坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCameraToBaseFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cameraCloud, Eigen::Matrix4d transformationMatrix);

    //判断支管主管
    void judgeMainorBranch(std::vector<pcl::ModelCoefficients>& cylinderCoefficients, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers);

    //计算板板焊缝的焊枪姿态
    void computeToolPostureInPlatePlate(std::vector<Posture>& postures);

signals:
    void positioningComplete(const PositioningResult& result);  //精定位完成信号

public slots:
    void whenAccuratePosition(WeldType weldType, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);   //精定位信号

};

#endif // ACCURATEPOSITIONING_H
