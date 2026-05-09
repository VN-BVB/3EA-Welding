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

    void setXAxisPosition(float position);
    void setYAxisPosition(float position);
    void setZAxisPosition(float position);

    void setRobotJointAngle(robotJointAngle angle);                                                                  // 设置机器人关节角
    void RobotArmAssemblyLink(systemParts parameter);                                                                // 构建各模块对象
    void importSTL();                                                                                                // 加载各模块模型
    void setCameraPos(int fromX, int fromY, int fromZ, int toX, int toY, int toZ, float upX, float upY, float upZ);  // 设置画面相机拍照位置
    void displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr visualCloud, const std::array<double, 3>& color = {1.0, 0.0, 0.0});  // 点云显示
    void displayLines(const std::shared_ptr<std::vector<pcl::PointXYZ>>& lineEndpoints, const std::array<double, 3>& color = {0.0, 1.0, 0.0},
                      double lineWidth = 5.0);  // 显示直线
    void clearPointCloud();                     // 清空界面中的点云
    void clearLines();                          // 清空界面中的直线段

private:
    std::vector<systemParts> parameter;
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_X_Bottom = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_X_MovePlate = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_Y_Pillar = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_Y_Beam = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_Y_MovePlate = vtkSmartPointer<vtkAssembly>::New();
    vtkSmartPointer<vtkAssembly> externalAxis_Z_MovePillar = vtkSmartPointer<vtkAssembly>::New();

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
    class CameraPrintCallback : public vtkCommand {
    public:
        static CameraPrintCallback* New() { return new CameraPrintCallback; }

        void SetRenderer(vtkRenderer* ren) { renderer = ren; }

        void Execute(vtkObject* caller, unsigned long eventId, void* callData) override {
            if (!renderer) return;

            vtkCamera* camera = renderer->GetActiveCamera();
            if (!camera) return;

            double pos[3];
            double focal[3];
            double up[3];

            camera->GetPosition(pos);
            camera->GetFocalPoint(focal);
            camera->GetViewUp(up);

            qDebug() << "setCameraPos(" << pos[0] << "," << pos[1] << "," << pos[2] << "," << focal[0] << "," << focal[1] << "," << focal[2] << ");"
                     << " ViewUp = " << up[0] << "," << up[1] << "," << up[2];
        }

    private:
        vtkRenderer* renderer = nullptr;
    };
};

#endif  // SYSTEMMIRRORWIDGET_H
