#ifndef GANTRAYFRAMETRAJECTORYPLANNING_H
#define GANTRAYFRAMETRAJECTORYPLANNING_H

#include <QObject>
#include <QtConcurrent>
#include <memory>
#include <vector>

#include "AbstractTrajectoryPlanning.h"
#include "src/robotTrajectoryPlanning/utils/debugcollisioncheck.h"
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
    void debugWeldingCollisionCheck(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    Eigen::Vector3d abcToDirection(double a, double b, double c);
    void applyWeldGunWithdraw(robotPose &pose, double withdrawDistance);
    void compensateSeams(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void planPlatePlateFilletSeamOrientation(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    void computeSwingReferencePointsForSeam(std::vector<std::shared_ptr<WeldSeamInfo>> &weldSeamInfo);
    bool computePlatePlateFilletVerticalSwingPoints(const std::shared_ptr<WeldSeamInfo> &info, const robotPose &refPose,
                                                    std::vector<double> &swingPoints);  // 计算摆焊点
    bool computeTubePlateFilletSwingPoints(const std::shared_ptr<WeldSeamInfo> &info, const robotPose &basePose, std::vector<double> &swingPoints);
    bool computeTubeTubeFilletSwingPoints(const std::shared_ptr<WeldSeamInfo> &info, const robotPose &basePose, std::vector<double> &swingPoints);

private:
    WORKPIECE_SIDE_OF_ROBOT workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;  // 当前工件位于机器人基座的方向pi
    // txt存储点
    std::fstream outfile;  // 读取存在mask坐标的txt文件
    std::string outfile_name = "./data/SeamCoordinate.txt";
    debugCollisionCheck checker;
    const float maxExtraOffset = 300.0f;  // 碰撞检测最高抬起限制

    float tubeSidePlateFilletPlanePoseW = 0.5f;  // 管侧与板角接焊缝靠近三角肘板平面法向量权重
    float platePlateFilletPlanePoseW_H = 0.5f;   // 板板水平角接靠近立板法向量权重（变大--靠近底）
    float platePlateFillettiltW_H = 0.5f;        // 板板水平角接靠近焊缝权重（变大--靠近焊缝，1为45）
    float platePlateFilletPlanePoseW_V = 0.5f;   // 板板垂直角接靠近立板法向量权重（变大-靠近侧壁）
    float platePlateFilletWeldPoseW_V = 0.45f;   // 板板垂直角接靠近焊缝方向向量权重 （变大-靠近Z，增大与地面角度）
    float tubePlateFilletPlanePoseW = 0.5f;      // 管板角接焊缝母材方向权重（变大，靠近圆柱，即朝着平面法向量偏）
    float tubePlateFilletWeldPoseW = 1.0f;       // 管板角接焊缝焊缝方向权重（为1时不偏，越小越向焊接方向偏移）
    float tubeTubeFilletCylinderPoseW = 0.5f;    // 管管角接焊缝母材方向权重（ 越大：越偏向第二个圆柱径向向量）
    float tubeTubeFilletWeldPoseW = 1.0f;        // 管管角接焊缝焊缝方向权重（为1时不偏，越小越向焊接方向偏移）

    double moveSpeed = 170 * 60;           // 过渡运动速度
    double weldingSpeedDefault = 5 * 60;   // 焊接速度(默认速度，宽度检测失败时用这个速度)
    double weldingCurrent = 160;           // 焊接电流(默认焊接电流)
    double weldingCurrent_Vertical = 130;  // 焊接电流(竖直焊缝用这个电流)
    double weldingVoltage = 24;            // 焊接电压(默认焊接电压)
    double weldingVoltage_Vertical = 18;   // 焊接电压(竖直焊缝用这个电压)

    friend class RailWeldingSystem;
};
#endif  // GANTRAYFRAMETRAJECTORYPLANNING_H
