#ifndef LARGEWORKPIECETRAJECTORYPLANNING_H
#define LARGEWORKPIECETRAJECTORYPLANNING_H

#include <QObject>
#include <memory>
#include <vector>

#include "AbstractTrajectoryPlanning.h"
#include "utils/common/WeldSeamInfo.h"

class LargeWorkpieceTrajectoryPlanning : public AbstractTrajectoryPlanning {
    Q_OBJECT
public:
    explicit LargeWorkpieceTrajectoryPlanning(QObject* parent = nullptr);
    ~LargeWorkpieceTrajectoryPlanning() override = default;

    // 实现抽象基类的纯虚函数

    void write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams) override;

public slots:
    void whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) override;

private:
    void transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);
    void determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);

private:
    WORKPIECE_SIDE_OF_ROBOT workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;  // 当前工件位于机器人基座的方向
    // txt存储点
    std::fstream outfile;  // 读取存在mask坐标的txt文件
    std::string outfile_name = "./data/SeamCoordinate.txt";

    friend class RailWeldingSystem;
};

#endif  // LARGEWORKPIECETRAJECTORYPLANNING_H
