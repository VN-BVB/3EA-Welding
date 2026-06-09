#include "SystemMirrorWidget.h"

SystemMirrorWidget::SystemMirrorWidget(QObject* parent) {
    renderWindow->AddRenderer(renderer);
    this->SetRenderWindow(renderWindow);

    importSTL();

    (void)parent;
}

// 设置X位置
void SystemMirrorWidget::setXAxisPosition(float position) {
    parameter[1].assembly->SetPosition(position, 0, 0);
    this->GetRenderWindow()->Render();
}
// 设置Y位置
void SystemMirrorWidget::setYAxisPosition(float position) {
    parameter[4].assembly->SetPosition(0, position, 0);
    this->GetRenderWindow()->Render();
}
// 设置Z位置
void SystemMirrorWidget::setZAxisPosition(float position) {
    parameter[5].assembly->SetPosition(0, 0, position);
    this->GetRenderWindow()->Render();
}

// 设置机器人关节角
void SystemMirrorWidget::setRobotJointAngle(robotJointAngle angle) {
    parameter[7].assembly->SetOrientation(0, 0, angle.joint1);
    parameter[8].assembly->SetOrientation(0, angle.joint2 - 90, 0);
    parameter[9].assembly->SetOrientation(0, -angle.joint3, 0);
    parameter[10].assembly->SetOrientation(0, 0, angle.joint4);
    parameter[11].assembly->SetOrientation(0, -angle.joint5, 0);
    parameter[12].assembly->SetOrientation(0, 0, angle.joint6);

    this->GetRenderWindow()->Render();
}

// 加载各模块模型
void SystemMirrorWidget::importSTL() {
    parameter = {
        {externalAxis_X_Bottom,     "./data/3D_Models/3ea/STL/3EA_WR_01_X_Bottom.stl",    {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.90, 0.60, 0.60}},
        {externalAxis_X_MovePlate,  "./data/3D_Models/3ea/STL/3EA_WR_02_X_MovePlate.stl", {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.45, 0.8, 0.55} },
        {externalAxis_Y_Pillar,     "./data/3D_Models/3ea/STL/3EA_WR_03_Y_Pillar.stl",    {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.90, 0.60, 0.60}},
        {externalAxis_Y_Beam,       "./data/3D_Models/3ea/STL/3EA_WR_04_Y_Beam.stl",      {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.90, 0.60, 0.60}},
        {externalAxis_Y_MovePlate,  "./data/3D_Models/3ea/STL/3EA_WR_05_Y_MovePlate.stl", {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.45, 0.8, 0.55} },
        {externalAxis_Z_MovePillar, "./data/3D_Models/3ea/STL/3EA_WR_06_Z_Pillar.stl",    {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.45, 0.8, 0.55} },
        {link0,                     "./data/3D_Models/robot/an_chuan_STL/link base.stl",  {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.2, 0.45, 1.0}  },
        {link1,                     "./data/3D_Models/robot/an_chuan_STL/Link 1.stl",     {0, 0, 0},      {0, 0, 0},     {0, 0, 0},      {0.2, 0.45, 1.0}  },
        {link2,                     "./data/3D_Models/robot/an_chuan_STL/Link 2.stl",     {155, 0, 450},  {90, 0, 0},    {155, 0, 450},  {0.2, 0.45, 1.0}  },
        {link3,                     "./data/3D_Models/robot/an_chuan_STL/Link 3.stl",     {769, 0, 450},  {90, 0, 0},    {769, 0, 450},  {0.2, 0.45, 1.0}  },
        {link4,                     "./data/3D_Models/robot/an_chuan_STL/Link 4.stl",     {969, 0, -190}, {180, 0, 0},   {969, 0, -190}, {0.2, 0.45, 1.0}  },
        {link5,                     "./data/3D_Models/robot/an_chuan_STL/Link 5.stl",     {969, 0, -190}, {90, 0, 0},    {969, 0, -190}, {0.2, 0.45, 1.0}  },
        {link6,                     "./data/3D_Models/robot/an_chuan_STL/Link 6.stl",     {969, 0, -190}, {180, 0, 0},   {969, 0, -190}, {0.2, 0.45, 1.0}  },
        {Tlink,                     "./data/3D_Models/robot/an_chuan_STL/weldgun.stl",    {969, 0, -290}, {180, -90, 0}, {969, 0, -290}, {0.4, 0.4, 0.4}   },
    };

    // 连接各模块
    for (int i = 0; i < parameter.size(); i++) {
        RobotArmAssemblyLink(parameter[i]);
        if (i < parameter.size() - 1) {
            parameter[i].assembly->AddPart(parameter[i + 1].assembly);
        }
        parameter[i].assembly->SetOrigin(parameter[i].Origin);
    }
    renderer->AddActor(externalAxis_X_Bottom);
    // 显示第一个零件坐标系
    auto axes = vtkSmartPointer<vtkAxesActor>::New();

    axes->SetTotalLength(300, 300, 300);
    axes->SetShaftTypeToCylinder();
    axes->SetAxisLabels(true);
    axes->SetPickable(false);
    axes->SetDragable(false);

    renderer->AddActor(axes);

    renderer->SetBackground(0.85, 0.85, 0.85);
    this->GetRenderWindow()->AddRenderer(renderer);
    setRobotJointAngle(robotJointAngle(0, 0, 0, 0, 0, 0));

    setCameraPos(-3300, 3360, -1280, 2250, -915, 1500, 0.2, -0.37, -0.9);  // 设置画面相机拍照位置
    // 实时显示相机位置
    //  vtkSmartPointer<CameraPrintCallback> cameraCallback = vtkSmartPointer<CameraPrintCallback>::New();

    // cameraCallback->SetRenderer(renderer);

    // this->GetRenderWindow()->GetInteractor()->AddObserver(vtkCommand::InteractionEvent, cameraCallback);
    this->GetRenderWindow()->Render();
}

// 设置画面相机拍照位置
void SystemMirrorWidget::setCameraPos(int fromX, int fromY, int fromZ, int toX, int toY, int toZ, float upX, float upY, float upZ) {
    renderer->GetActiveCamera()->SetPosition(fromX, fromY, fromZ);  // 相机在哪拍
    renderer->GetActiveCamera()->SetFocalPoint(toX, toY, toZ);      // 相机往哪拍
    renderer->GetActiveCamera()->SetViewUp(upX, upY, upZ);
    renderer->ResetCameraClippingRange();

    this->GetRenderWindow()->Render();
}

// 点云显示
void SystemMirrorWidget::displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr visualCloud, const std::array<double, 3>& color) {
    if (!renderer || !visualCloud || visualCloud->empty()) return;

    auto vtkPointsObj = vtkSmartPointer<vtkPoints>::New();
    for (const auto& pt : visualCloud->points) {
        vtkPointsObj->InsertNextPoint(pt.x + railPosition, pt.y, pt.z + 450);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtkPointsObj);

    auto glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetPointSize(0.01);
    actor->GetProperty()->SetOpacity(0.1);

    renderer->AddActor(actor);
    // 挂到 link0 下面
    // parameter[6].assembly->AddPart(actor);
    pointCloudActors.push_back(actor);  // 保存点云Actor
}

// 显示直线 / 曲线
void SystemMirrorWidget::displayLines(const std::shared_ptr<std::vector<pcl::PointXYZ>>& lineEndpoints, const std::array<double, 3>& color,
                                      double lineWidth) {
    if (!renderer || !lineEndpoints || lineEndpoints->empty()) return;
    bool asPolyline = lineEndpoints->size() >= 3;

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();

    // ================= 1. 添加点 =================
    for (size_t i = 0; i < lineEndpoints->size(); ++i) {
        points->InsertNextPoint((*lineEndpoints)[i].x + railPosition, (*lineEndpoints)[i].y, (*lineEndpoints)[i].z + 450);
    }

    // ================= 2. 构建线 =================
    if (asPolyline) {
        // 曲线（连续折线）
        for (vtkIdType i = 0; i < static_cast<vtkIdType>(lineEndpoints->size()) - 1; ++i) {
            auto line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, i);
            line->GetPointIds()->SetId(1, i + 1);
            lines->InsertNextCell(line);
        }
    } else {
        //  原始逻辑：每两个点一条线
        if (lineEndpoints->size() % 2 != 0) {
            qWarning("displayLines(): 点数量应为偶数，每两个点组成一条线段。");
            return;
        }

        for (vtkIdType i = 0; i < static_cast<vtkIdType>(lineEndpoints->size()); i += 2) {
            auto line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, i);
            line->GetPointIds()->SetId(1, i + 1);
            lines->InsertNextCell(line);
        }
    }

    // ================= 3. 构建 PolyData =================
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // ================= 4. 映射 =================
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetLineWidth(lineWidth);

    renderer->AddActor(actor);
    lineActors.push_back(actor);
}
// 清空界面中的点云
void SystemMirrorWidget::clearPointCloud() {
    if (!renderer) return;

    for (const auto& actor : pointCloudActors) {
        renderer->RemoveActor(actor);
        // parameter[6].assembly->RemovePart(actor);
        // renderer->RemoveActor(actor); // 保留也没问题，防止旧的世界actor。
    }
    pointCloudActors.clear();  // 清空保存的引用
}

// 清空界面中的直线段
void SystemMirrorWidget::clearLines() {
    if (!renderer) return;

    for (const auto& actor : lineActors) {
        renderer->RemoveActor(actor);
    }
    lineActors.clear();
}

void SystemMirrorWidget::RobotArmAssemblyLink(systemParts parameter) {
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

    reader->SetFileName(parameter.filename.c_str());
    reader->Update();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    actor->GetProperty()->SetDiffuse(0.93);  // 漫反射系数
    actor->GetProperty()->SetDiffuseColor(colors->GetColor3d("LightSteelBlue").GetData());
    actor->GetProperty()->SetSpecular(0.1);  // 镜面反射系数
    actor->GetProperty()->SetSpecularPower(6.0);

    actor->RotateZ(parameter.Rotate[2]);
    actor->RotateY(parameter.Rotate[1]);
    actor->RotateX(parameter.Rotate[0]);
    actor->SetPosition(parameter.PositionXYZ);
    actor->GetProperty()->SetColor(parameter.rgb);
    // if (parameter.assembly == externalAxis_Z_MovePillar || parameter.assembly == link0) {
    //     actor->GetProperty()->SetOpacity(0.5);
    // }
    parameter.assembly->AddPart(actor);
}
