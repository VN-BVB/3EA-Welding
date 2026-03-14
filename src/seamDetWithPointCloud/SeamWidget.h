#ifndef SEAMWIDGET_H
#define SEAMWIDGET_H

#include "src/stable.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

class AccuratePositioning;
struct PositioningResult;

#include "steelDefaultDet//AccuratePositioning.h"

namespace Ui {
class SeamWidget;
}

class SeamWidget : public QWidget {
    Q_OBJECT

public:
    explicit SeamWidget(QWidget* parent = nullptr);
    ~SeamWidget();

private:
    Ui::SeamWidget* ui;

public:
    void initVtkWindow();  // 初始化vtk显示页面
    void registerMetaType();
    void initPositionSystem();  // 初始化定位系统

    void pointCloudDisplay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);          // 点云显示
    void pointCloudResultDisplay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);    // 特征点云显示
    void displayLine(std::vector<std::vector<pcl::PointXYZ>> extremePoint);            // 显示直线段
    void nurbsCurveDisplay(std::vector<Point3D> generatePoints);                       // nurbs曲线可视化
    void displayCylinderModel(const pcl::ModelCoefficients& coefficients, int i = 0);  // 圆柱面模型可视化
    // 焊缝坐标系和焊枪姿态可视化
    void displayCoordinateaxisAndToolPosture(std::vector<Point3D> position, std::vector<Point3D> xAxis,
                                             std::vector<Point3D> yAxis, std::vector<Point3D> zAxis,
                                             std::vector<Eigen::Quaternionf> toolPose);
    void positioningResultDisplay(const PositioningResult& result);  // 定位结果显示

signals:
    void sendAccPosition(int weldType, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);  // 精定位信号
private slots:
    void on_btnAccPosition_clicked();

    void on_btnLoadingPointCloud_clicked();

private:
    std::shared_ptr<AccuratePositioning> accuratePositioning{nullptr};  // 精定位系统控制类
    QThread* accuratePositioningThread = new QThread;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // 点云
    vtkSmartPointer<vtkRenderer> renderer1;
    vtkSmartPointer<vtkRenderer> renderer2;

    QElapsedTimer algorithmExecutionTimer;  // 用于测量算法执行时间
};

#endif  // SEAMWIDGET_H
