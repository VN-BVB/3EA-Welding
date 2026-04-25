#ifndef TUBETUBEFILLETSEAMSDET_H
#define TUBETUBEFILLETSEAMSDET_H

#include <QObject>

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"
struct FeatureResult;
struct PtTheta {
    pcl::PointXYZ pt;
    float theta = 0.0f;
};

struct CylinderModelCache {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis = Eigen::Vector3f::Zero();
    float radius = 0.0f;
    // 以该圆柱轴为 z 方向构造的局部正交基
    Eigen::Vector3f u = Eigen::Vector3f::Zero();
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
};

struct TubeTubeSeamContext {
    CylinderModelCache primary;
    CylinderModelCache secondary;

    float axisDot = 0.0f;
    bool axesNearlyParallel = false;
};
class TubeTubeFilletSeamsDet : public AbstractSeamDet {
public:
    explicit TubeTubeFilletSeamsDet(QObject* parent = nullptr);
    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;
    struct PtTheta {
        pcl::PointXYZ pt;
        float theta;
    };

private:
    void singleSeamReinitialize();
    void ransacCylinder();
    void extractSeamPointsFromCylinderPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                            pcl::ModelCoefficients::Ptr plane_coeff, double thresh_plane);
    void removePlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool buildCylinderCache(const pcl::ModelCoefficients::Ptr& coeffs, CylinderModelCache& cache);
    bool solveSeamEndPoints();
    bool solveTheorySeamEndPoints(std::vector<pcl::PointXYZ>& filletSeamsTheoryTP);
    bool solveActualSeamPoints(const std::vector<pcl::PointXYZ>& filletSeamsTheoryTP, std::vector<pcl::PointXYZ>& filletSeamsActualTP);
    bool moveAlongOrdered(const std::vector<PtTheta>& ordered, float offset, bool from_start, PtTheta& result, int& cut_idx);
    bool extractLocalVoxelRegionAroundSeamSamples(const pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud, const std::vector<pcl::PointXYZ>& seamSamples,
                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& outCloud);
    bool projectPointToCylinderByFixedRadialDir(const Eigen::Vector3f& point, const Eigen::Vector3f& theoryPt, const CylinderModelCache& cyl,
                                                Eigen::Vector3f& projPoint);
    bool refineTheoryPointsToActualPoints(const std::vector<pcl::PointXYZ>& theoryPts, const pcl::PointCloud<pcl::PointXYZ>::Ptr& refCloud,
                                          const Eigen::Vector3f& searchDirInput, const CylinderModelCache& projectCylinder,
                                          std::vector<pcl::PointXYZ>& actualPts);

private:
    bool saveFlag = false;
    bool detectSuccFlag = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCloudPrimary;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCloudSecondary;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seamEndPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axisRangeCloud;

    TubeTubeSeamContext seamCtx_;

    int areaNum = -1;
    double Max_cluster_radius = 8;           // 欧式聚类提取最大点集半径
    int Ransac_plane_Iterations = 10000;     // Ransac拟合平面的迭代数
    int Ransac_cylinder_Iterations = 10000;  // Ransac拟合圆柱的迭代数
    double Ransac_plane_Dth = 0.5;           // ransac拟合平面的距离阈值
    double Ransac_cylinder_Dth = 1.0;        // ransac拟合圆柱面的距离阈值

    int Statistic_NeighPoints = 20;           // 统计滤波近邻点数
    float Statistic_sigma = 6.0;              // 统计滤波系数
    double extendCylinderInPlaneArea = 15.0;  // 选取焊缝区域衍生
    float t_step = 1.0f;                      // 轴向分辨率（mm）
    float theta_step = 2.0f * M_PI / 180.0f;  // n°一格
    double widthThreshRatio = 0.8;            // 筛选残留点云宽度阈值比例
    int sample_num = 3 + 3 * 2;               // 交线采样点数量，3是起点中点终点，乘2是中点两边
    // ===== 搜索阈值 =====
    const float maxAxisOffset = 30.0f;    // 沿第一主轴方向允许前后搜索
    const float maxPerpDist = 10.0f;      // 到“过理论点、方向为第一主轴的直线”的最大距离
    const float maxEuclidDist = 25.0f;    // 兜底欧式距离，管管缺口建议别太小
    float filletSeamsBlendWeight = 1.0f;  // 理论点与实际点权重

    std::vector<pcl::PointXYZ> filletSeamsTP;             // 焊缝的端点
    pcl::ModelCoefficients::Ptr cylinderCoeffsPrimary;    // 焊缝区域圆柱第一母材系数
    pcl::ModelCoefficients::Ptr cylinderCoeffsSecondary;  // 焊接区域圆柱第二母材系数
};

#endif  // TUBETUBEFILLETSEAMSDET_H
