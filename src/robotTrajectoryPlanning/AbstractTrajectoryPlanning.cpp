#include "AbstractTrajectoryPlanning.h"

#include "plog/Log.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "settingPara/SettingPara.h"

AbstractTrajectoryPlanning::AbstractTrajectoryPlanning(QObject* parent)
    : QObject(parent), trajectoryConfig(TrajectoryPlanningConfig::getInstance()), settingPara(SettingPara::getInstance()) {
    this->initConfig();
}

// 初始化配置信息
void AbstractTrajectoryPlanning::initConfig() {
    // this->writeConfig();  // 写配置文件
    this->readConfig();  // 读配置文件
    // this->printConfig();  // 打印配置文件

    TrajectoryPlanningConfig::getInstance().myDataStructure2LibDataStructure();  // 自定义数据类型转换为库数据类型
}

// 写配置文件
void AbstractTrajectoryPlanning::writeConfig() {
    {  // 写
        std::ofstream os("./data/config/Trajectory_Planning_config.json");
        cereal::JSONOutputArchive jsonOutputArchive(os);
        jsonOutputArchive(cereal::make_nvp("config about Trajectory Planning", TrajectoryPlanningConfig::getInstance()));
    }
}

// 读配置文件
void AbstractTrajectoryPlanning::readConfig() {
    {  // 读
        std::ifstream is("./data/config/Trajectory_Planning_config.json");
        cereal::JSONInputArchive inputArchive(is);
        inputArchive(TrajectoryPlanningConfig::getInstance());
        PLOGD << "轨迹规划配置文件读取成功";
    }
}

// 打印配置文件
void AbstractTrajectoryPlanning::printConfig() {
    {  // 打印
        cereal::JSONOutputArchive jsonOutputArchive(std::cout);
        jsonOutputArchive(cereal::make_nvp("config about Trajectory Planning", TrajectoryPlanningConfig::getInstance()));
        std::cout << std::endl;
    }
}

void AbstractTrajectoryPlanning::writeWeldPoint(std::fstream& outfile, double x, double y, double z, double a, double b, double c, double speed,
                                                ARC_ACTION arcAction, SWING_WELD_ACTION weldAction, double current, double voltage, double p1x,
                                                double p1y, double p1z, double p2x, double p2y, double p2z) {
    outfile << x << " " << y << " " << z << " " << a << " " << b << " " << c << " " << speed << " " << static_cast<int>(arcAction) << " "
            << static_cast<int>(weldAction) << " " << current << " " << voltage << " " << p1x << " " << p1y << " " << p1z << " " << p2x << " " << p2y
            << " " << p2z << std::endl;
}
