#ifndef GANTRAYFRAMETRAJECTORYPLANNING_H
#define GANTRAYFRAMETRAJECTORYPLANNING_H

#include <QObject>
#include <QtConcurrent>
#include <memory>
#include <vector>

#include "AbstractTrajectoryPlanning.h"
#include "utils/common/WeldSeamInfo.h"

class GantrayFrameTrajectoryPlanning : public AbstractTrajectoryPlanning {
    Q_OBJECT
public:
    explicit GantrayFrameTrajectoryPlanning(QObject *parent = nullptr);
    ~GantrayFrameTrajectoryPlanning() override = default;

    // 实现抽象基类的纯虚函数

    void write2File(const std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo, int endOfLeftSeams) override;

public slots:
    void whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) override;

private:
    void normalizeSurfaceDirectionInCamera(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void transSeamsOri(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void generateWeldPose(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void saveToMatlabFull(const std::string &filename, const Eigen::Vector3f &P0, const Eigen::Vector3f &P1, const Eigen::Vector3f &mid,
                          const Eigen::Vector3f &X, const Eigen::Vector3f &Y, const Eigen::Vector3f &Z, const pcl::ModelCoefficients::Ptr &plane,
                          const pcl::ModelCoefficients::Ptr &cylinder);
    Eigen::Vector3d abcToDirection(double a, double b, double c);
    void applyWeldGunWithdraw(robotPose &pose, double withdrawDistance);
    void compensateSeams(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void planPlatePlateFilletSeamOrientation(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    bool checkCylinderPlaneCollision(const Eigen::Vector3d &center, const Eigen::Vector3d &axis, double radius, const Eigen::Vector4f &plane);
    bool checkCylinderCylinderCollision(const Eigen::Vector3d &p1, const Eigen::Vector3d &d1, double r1, const Eigen::Matrix<float, 7, 1> &cyl);

private:
    WORKPIECE_SIDE_OF_ROBOT workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;  // 当前工件位于机器人基座的方向pi
    // txt存储点
    std::fstream outfile;  // 读取存在mask坐标的txt文件
    std::string outfile_name = "./data/SeamCoordinate.txt";

    float tubeSidePlateFilletPlanePoseW = 0.7f;  // 管侧与板角接焊缝靠近三角肘板平面法向量权重
    float platePlateFilletPlanePoseW_H = 0.5f;   // 板板水平角接靠近立板法向量权重（变大--靠近底）
    float platePlateFillettiltW_H = 0.5f;        // 板板水平角接靠近焊缝权重（变大--靠近焊缝，1为45）
    float platePlateFilletPlanePoseW_V = 0.5f;   // 板板垂直角接靠近立板法向量权重（变大-靠近侧壁）
    float platePlateFilletWeldPoseW_V = 0.45f;   // 板板垂直角接靠近焊缝方向向量权重 （变大-靠近Z，增大与地面角度）

    double moveSpeed = 170 * 60;           // 过渡运动速度
    double weldingSpeedDefault = 5 * 60;   // 焊接速度(默认速度，宽度检测失败时用这个速度)
    double weldingCurrent = 160;           // 焊接电流(默认焊接电流)
    double weldingCurrent_Vertical = 130;  // 焊接电流(竖直焊缝用这个电流)
    double weldingVoltage = 24;            // 焊接电压(默认焊接电压)
    double weldingVoltage_Vertical = 18;   // 焊接电压(竖直焊缝用这个电压)
    friend class RailWeldingSystem;
};
#endif  // GANTRAYFRAMETRAJECTORYPLANNING_H
