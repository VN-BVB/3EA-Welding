
// #include <plog/Init.h>
// #include <plog/Initializers/ConsoleInitializer.h>
// #include <plog/Initializers/RollingFileInitializer.h>
// #include <plog/Log.h>
// #include <vtkFileOutputWindow.h>
// #include <vtkOutputWindow.h>

// #include <QApplication>
// #include <QTextCursor>
// // clang-format off
// #include <winsock2.h>
// #include <windows.h>

// #include "crashHandler/CrashHandler.h"
// #include "ui/WeldingMainWindow.h"
// // clang-format on
// #include "crashHandler/CrashHandler.h"
// #include "robotFactory/AbstractRobot.h"
// #include "src/rail/RailWidget.h"
// #include "structLightCamera/StructLightCamera.h"
// void initPlog();          // 初始化日志类
// void registerMetaType();  // 注册元数据类型
// int main(int argc, char *argv[]) {
//     QApplication a(argc, argv);
//     vtkOutputWindow::SetGlobalWarningDisplay(0);  // 取消VTK窗口显示
//     CrashHandler::Init(L"data/debug");            // 初始化Mini转储
//     initPlog();                                   // 初始化日志类
//     registerMetaType();                           // 注册元数据类型

//     WeldingMainWindow w;
//     w.show();
//     return a.exec();
// }
// // 初始化日志类
// void initPlog() {
//     // 日志信息分类等级: none = 0, fatal = 1, error = 2, warning = 3, info = 4, debug = 5, verbose = 6
//     // 设置初始化等级后, 等级『数值大于』设置值的日志信息就会被『忽略』
//     plog::init(plog::debug, "./data/log/log.csv", 1000000000, 100);
//     static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
//     plog::get()->addAppender(&consoleAppender);  // Also add logging to the console.
// }
// // 注册元数据类型
// void registerMetaType() {
//     qRegisterMetaType<QImage>("QImage");
//     qRegisterMetaType<cv::Mat>("cv::Mat");
//     qRegisterMetaType<QString>("QString");
//     qRegisterMetaType<robotPose>("robotPose");
//     qRegisterMetaType<QTextCursor>("QTextCursor");
//     qRegisterMetaType<QVector<bool>>("QVector<bool>");
//     qRegisterMetaType<robotJointAngle>("robotJointAngle");
//     qRegisterMetaType<QVector<quint16>>("QVector<quint16>");
//     qRegisterMetaType<CAMERA_WORK_MODE>("CAMERA_WORK_MODE");
//     qRegisterMetaType<std::vector<DEVICE>>("std::vector<DEVICE>");
//     qRegisterMetaType<std::vector<QString>>("std::vector<QString>");
//     qRegisterMetaType<std::shared_ptr<AbstractAxis>>("std::shared_ptr<AbstractAxis>");
//     // qRegisterMetaType<std::vector<COARES_LOC_CAMERA>>("std::vector<COARES_LOC_CAMERA>");
//     qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
//     qRegisterMetaType<std::vector<std::shared_ptr<WeldSeamInfo>>>("std::vector<std::shared_ptr<WeldSeamInfo>>");
//     qRegisterMetaType<std::vector<std::vector<QTableWidgetItem *>>>("std::vector<std::vector<QTableWidgetItem*>>");
//     qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
// }
#include <QString>
#include <fstream>
#include <iostream>
#include <vector>

// 模拟 packPoint（你用自己的也行）
std::vector<QString> packPoint(double x, double y, double z, double rx, double ry, double rz, double speed, double arc, double swing, double current,
                               double voltage, double p1x, double p1y, double p1z, double p2x, double p2y, double p2z) {
    std::vector<QString> pt;
    pt.push_back(QString::number(x));
    pt.push_back(QString::number(y));
    pt.push_back(QString::number(z));
    pt.push_back(QString::number(rx));
    pt.push_back(QString::number(ry));
    pt.push_back(QString::number(rz));
    pt.push_back(QString::number(speed));
    pt.push_back(QString::number(arc));
    pt.push_back(QString::number(swing));
    pt.push_back(QString::number(current));
    pt.push_back(QString::number(voltage));
    pt.push_back(QString::number(p1x));
    pt.push_back(QString::number(p1y));
    pt.push_back(QString::number(p1z));
    pt.push_back(QString::number(p2x));
    pt.push_back(QString::number(p2y));
    pt.push_back(QString::number(p2z));
    return pt;
}

// 模拟发送
void SendPoint(const std::vector<QString>& pt) {
    std::cout << "Send: ";
    for (auto& s : pt) {
        std::cout << s.toStdString() << " ";
    }
    std::cout << std::endl;
}

int main() {
    std::ifstream infile("./data/SeamCoordinate.txt");

    double x, y, z, rx, ry, rz;
    double moveSpeed, arc;
    double swingWeldAction;
    double weldingCurrent, weldingVoltage;
    double p1x, p1y, p1z;
    double p2x, p2y, p2z;

    std::vector<std::vector<QString>> data;

    // ================= 读取 =================
    while (infile >> x >> y >> z >> rx >> ry >> rz >> moveSpeed >> arc >> swingWeldAction >> weldingCurrent >> weldingVoltage >> p1x >> p1y >> p1z >>
           p2x >> p2y >> p2z) {
        data.push_back(packPoint(x, y, z, rx, ry, rz, moveSpeed, arc, swingWeldAction, weldingCurrent, weldingVoltage, p1x, p1y, p1z, p2x, p2y, p2z));
    }

    std::cout << "总点数: " << data.size() << std::endl;

    if (data.empty()) return 0;

    // 奇数补齐
    if (data.size() % 2 == 1) {
        data.push_back(data.back());
    }

    // ================= 核心测试逻辑 =================
    int m_i_Pointnum = 0;

    while (m_i_Pointnum < data.size()) {
        int swingType = data[m_i_Pointnum][8].toInt();

        // ================= 曲线段 =================
        if (swingType == 101) {
            int start = m_i_Pointnum;
            int end = start;

            while (end < data.size() && data[end][8].toInt() == 101) {
                end++;
            }

            int count = end - start;

            std::cout << "\n==== 曲线段 ====\n";
            std::cout << "起点: " << start << " 终点: " << end - 1 << " 数量: " << count << std::endl;

            for (int i = start; i < end; i++) {
                SendPoint(data[i]);
            }

            m_i_Pointnum = end;
        } else {
            std::cout << "\n---- 直线点 ----\n";
            SendPoint(data[m_i_Pointnum]);
            m_i_Pointnum++;
        }
    }

    std::cout << "\n=== 测试完成 ===" << std::endl;

    return 0;
}
