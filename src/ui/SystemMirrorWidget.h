#ifndef SYSTEMMIRRORWIDGET_H
#define SYSTEMMIRRORWIDGET_H

#include <QVTKOpenGLWidget.h>

#include <QObject>

#include "src/robotFactory/AbstractRobot.h"
#include "stable.h"

struct systemParts {
    vtkAssembly* assembly;
    std::string filename;
    double PositionXYZ[3];
    double Rotate[3];
    double Origin[3];
    double rgb[3];
};

class SystemMirrorWidget : public QVTKOpenGLWidget {
public:
    SystemMirrorWidget(QObject* parent = nullptr);

    float railPosition = 0;  // 地轨当前位置

    void setRailPosition(float position);                                           // 设置地轨位置
    void setRobotJointAngle(robotJointAngle angle);                                 // 设置机器人关节角
    void RobotArmAssemblyLink(systemParts parameter);                               // 构建各模块对象
    void importSTL();                                                               // 加载各模块模型
    void setCameraPos(int fromX, int fromY, int fromZ, int toX, int toY, int toZ);  // 设置画面相机拍照位置
    void displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr visualCloud, const std::array<double, 3>& color = {1.0, 0.0, 0.0});  // 点云显示
    void displayLines(const std::shared_ptr<std::vector<pcl::PointXYZ>>& lineEndpoints, const std::array<double, 3>& color = {0.0, 1.0, 0.0},
                      double lineWidth = 5.0);  // 显示直线
    void clearPointCloud();                     // 清空界面中的点云
    void clearLines();                          // 清空界面中的直线段

private:
    std::vector<systemParts> parameter;
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtkSmartPointer<vtkAssembly> rail = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link0 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link1 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link2 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link3 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link4 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link5 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> link6 = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> Tlink = vtkSmartPointer<vtkAssembly>::New();

    std::vector<vtkSmartPointer<vtkActor>> pointCloudActors;
    std::vector<vtkSmartPointer<vtkActor>> lineActors;

    friend class WeldingMainWindow;
};

#endif  // SYSTEMMIRRORWIDGET_H
