#include <plog/Init.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>

#include <QApplication>
#include <QTextCursor>

// clang-format off
#include <winsock2.h>
#include <windows.h>

#include "crashHandler/CrashHandler.h"
#include "ui/WeldingMainWindow.h"
// clang-format on
#include "crashHandler/CrashHandler.h"
#include "robotFactory/AbstractRobot.h"
#include "src/rail/RailWidget.h"
#include "structLightCamera/StructLightCamera.h"
void initPlog();          // 初始化日志类
void registerMetaType();  // 注册元数据类型
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    vtkOutputWindow::SetGlobalWarningDisplay(0);  // 取消VTK窗口显示
    CrashHandler::Init(L"data/debug");            // 初始化Mini转储
    initPlog();                                   // 初始化日志类
    registerMetaType();                           // 注册元数据类型

    WeldingMainWindow w;
    w.show();
    return a.exec();
}
// 初始化日志类
void initPlog() {
    // 日志信息分类等级: none = 0, fatal = 1, error = 2, warning = 3, info = 4, debug = 5, verbose = 6
    // 设置初始化等级后, 等级『数值大于』设置值的日志信息就会被『忽略』
    plog::init(plog::debug, "./data/log/log.csv", 1000000000, 100);
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::get()->addAppender(&consoleAppender);  // Also add logging to the console.
}
// 注册元数据类型
void registerMetaType() {
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<robotPose>("robotPose");
    qRegisterMetaType<QTextCursor>("QTextCursor");
    qRegisterMetaType<QVector<bool>>("QVector<bool>");
    qRegisterMetaType<robotJointAngle>("robotJointAngle");
    qRegisterMetaType<QVector<quint16>>("QVector<quint16>");
    qRegisterMetaType<CAMERA_WORK_MODE>("CAMERA_WORK_MODE");
    qRegisterMetaType<std::vector<DEVICE>>("std::vector<DEVICE>");
    qRegisterMetaType<std::vector<QString>>("std::vector<QString>");
    qRegisterMetaType<std::shared_ptr<AbstractAxis>>("std::shared_ptr<AbstractAxis>");
    // qRegisterMetaType<std::vector<COARES_LOC_CAMERA>>("std::vector<COARES_LOC_CAMERA>");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<std::vector<std::shared_ptr<WeldSeamInfo>>>("std::vector<std::shared_ptr<WeldSeamInfo>>");
    qRegisterMetaType<std::vector<std::vector<QTableWidgetItem *>>>("std::vector<std::vector<QTableWidgetItem*>>");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
}
