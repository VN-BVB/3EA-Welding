#include "SeamWidget.h"

#include "ui_SeamWidget.h"

SeamWidget::SeamWidget(QWidget* parent)
    : QWidget(parent), ui(new Ui::SeamWidget), accuratePositioning(std::make_shared<AccuratePositioning>(nullptr)) {
    ui->setupUi(this);

    setWindowState(Qt::WindowMaximized);  // 设置全屏
    setWindowTitle(u8"3DA_Welding_test  三轴焊接测试系统");

    this->initVtkWindow();       // 初始化vtk显示页面
    this->initPositionSystem();  // 初始化定位系统
    this->registerMetaType();
}

SeamWidget::~SeamWidget() {
    PLOGD << "正在退出三轴焊接测试系统... ...";
    delete ui;
}

void SeamWidget::initVtkWindow() {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    renderer1 = vtkSmartPointer<vtkRenderer>::New();
    renderer2 = vtkSmartPointer<vtkRenderer>::New();

    auto renderWindow1 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow1->AddRenderer(renderer1);
    ui->qvtkWidget->SetRenderWindow(renderWindow1);

    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    ui->qvtkWidget2->SetRenderWindow(renderWindow2);
}

void SeamWidget::registerMetaType() {
    qRegisterMetaType<WeldType>("WeldType");
    qRegisterMetaType<Point3D>("Point3D");
    qRegisterMetaType<std::vector<Point3D>>("std::vector<Point3D>");
    qRegisterMetaType<std::vector<std::vector<Point3D>>>("std::vector<std::vector<Point3D>>");
    qRegisterMetaType<std::vector<std::vector<Eigen::Quaternionf>>>("std::vector<std::vector<Eigen::Quaternionf>>");
    qRegisterMetaType<std::vector<std::vector<pcl::PointXYZ>>>("std::vector<std::vector<pcl::PointXYZ>>");
    qRegisterMetaType<PositioningResult>("PositioningResult");
    qRegisterMetaType<pcl::ModelCoefficients>("pcl::ModelCoefficients");
}

void SeamWidget::initPositionSystem() {
    if (accuratePositioning) {
        accuratePositioning->moveToThread(accuratePositioningThread);
        accuratePositioningThread->start();

        // 精定位信号槽
        connect(this, &SeamWidget::sendAccPosition, accuratePositioning.get(),
                [this](int weldTypeInt, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
                    WeldType weldType = static_cast<WeldType>(weldTypeInt);
                    accuratePositioning->whenAccuratePosition(weldType, cloud);
                });

        // 连接精定位完成信号
        connect(accuratePositioning.get(), &AccuratePositioning::positioningComplete, this,
                [this](const PositioningResult& result) { this->positioningResultDisplay(result); });

        PLOGD << "精定位系统类初始化成功";
    } else {
        PLOGE << "精定位系统类初始化失败";
    }
}

void SeamWidget::pointCloudDisplay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->points.empty()) {
        PLOGE << "点云为空";
        return;
    }

    renderer2->RemoveAllViewProps();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < cloud->size(); i++) {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    actor->GetProperty()->SetPointSize(2);
    actor->GetProperty()->SetColor(1, 1, 1);

    auto camera = vtkSmartPointer<vtkCamera>::New();
    camera->ParallelProjectionOn();

    renderer2->AddActor(actor);
    renderer2->SetBackground(0, 0, 0);

    ui->qvtkWidget2->GetRenderWindow()->AddRenderer(renderer2);
    renderer2->SetActiveCamera(camera);
    renderer2->ResetCamera();

    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void SeamWidget::pointCloudResultDisplay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->points.empty()) {
        PLOGE << "特征点云为空";
        return;
    }

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < cloud->size(); i++) {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    actor->GetProperty()->SetPointSize(10);
    actor->GetProperty()->SetColor(0, 255, 0);
    renderer2->AddActor(actor);

    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void SeamWidget::displayLine(std::vector<std::vector<pcl::PointXYZ>> extremePoint) {
    for (size_t i = 0; i < extremePoint.size(); i++) {
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(extremePoint[i][0].x, extremePoint[i][0].y, extremePoint[i][0].z);
        lineSource->SetPoint2(extremePoint[i][1].x, extremePoint[i][1].y, extremePoint[i][1].z);
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        actor->GetProperty()->SetColor(255.0, 0.0, 0.0);
        actor->GetProperty()->SetLineWidth(8.0);
        renderer2->AddActor(actor);
        ui->qvtkWidget2->GetRenderWindow()->Render();
    }
}

void SeamWidget::nurbsCurveDisplay(std::vector<Point3D> generatePoints) {
    if (generatePoints.empty()) {
        PLOGE << "控制点为空";
        return;
    }
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < generatePoints.size(); ++i) {
        double x = generatePoints[i].x;
        double y = generatePoints[i].y;
        double z = generatePoints[i].z;
        points->InsertNextPoint(x, y, z);
    }

    lines->InsertNextCell(generatePoints.size());
    for (int i = 0; i < generatePoints.size(); ++i) {
        lines->InsertCellPoint(i);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(255, 0, 0);
    actor->GetProperty()->SetLineWidth(5);

    renderer2->AddActor(actor);

    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void SeamWidget::displayCylinderModel(const pcl::ModelCoefficients& coefficients, int i) {
    double pointOnAxis[3] = {coefficients.values[0], coefficients.values[1], coefficients.values[2]};
    double axisDirection[3] = {coefficients.values[3], coefficients.values[4], coefficients.values[5]};
    double radius = coefficients.values[6];

    vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(radius);
    cylinderSource->SetHeight(500.0);
    cylinderSource->SetResolution(50);
    cylinderSource->Update();

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    double axisLength =
        sqrt(axisDirection[0] * axisDirection[0] + axisDirection[1] * axisDirection[1] + axisDirection[2] * axisDirection[2]);

    if (axisLength > 0) {
        double normalizedAxis[3] = {axisDirection[0] / axisLength, axisDirection[1] / axisLength, axisDirection[2] / axisLength};

        double yAxis[3] = {0.0, 1.0, 0.0};
        double rotationAxis[3];
        vtkMath::Cross(yAxis, normalizedAxis, rotationAxis);

        double rotationAxisLength =
            sqrt(rotationAxis[0] * rotationAxis[0] + rotationAxis[1] * rotationAxis[1] + rotationAxis[2] * rotationAxis[2]);

        if (rotationAxisLength < 1e-6) {
            if (normalizedAxis[1] < 0) {
                transform->RotateWXYZ(180.0, 1.0, 0.0, 0.0);
            }
        } else {
            rotationAxis[0] /= rotationAxisLength;
            rotationAxis[1] /= rotationAxisLength;
            rotationAxis[2] /= rotationAxisLength;

            double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(yAxis, normalizedAxis)));

            transform->Translate(pointOnAxis[0], pointOnAxis[1], pointOnAxis[2]);
            if (angle > 0.1) {
                transform->RotateWXYZ(angle, rotationAxis[0], rotationAxis[1], rotationAxis[2]);
            }
        }
    }

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    if (i == 0) {
        actor->GetProperty()->SetColor(0.0, 0.8, 0.8);
    } else if (i == 1) {
        actor->GetProperty()->SetColor(0.5, 0.7, 1.0);
    }

    actor->GetProperty()->SetOpacity(1.0);
    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetEdgeColor(1.0, 0.0, 0.0);

    renderer2->AddActor(actor);

    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void SeamWidget::displayCoordinateaxisAndToolPosture(std::vector<Point3D> position, std::vector<Point3D> xAxis,
                                                     std::vector<Point3D> yAxis, std::vector<Point3D> zAxis,
                                                     std::vector<Eigen::Quaternionf> toolPose) {
    if (position.empty() || xAxis.empty() || yAxis.empty() || zAxis.empty() || toolPose.empty()) {
        PLOGE << "坐标轴和焊枪位姿数据为空";
        return;
    }

    if (position.size() != xAxis.size() || position.size() != yAxis.size() || position.size() != zAxis.size() ||
        position.size() != toolPose.size()) {
        PLOGE << "坐标轴和焊枪位姿数据大小不匹配";
        return;
    }

    for (size_t i = 0; i < position.size(); ++i) {
        auto drawAxis = [&](const Point3D& dir, double r, double g, double b) {
            vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
            line->SetPoint1(position[i].x, position[i].y, position[i].z);
            line->SetPoint2(position[i].x + dir.x * 20.0, position[i].y + dir.y * 20.0, position[i].z + dir.z * 20.0);

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(line->GetOutputPort());

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetColor(r, g, b);
            actor->GetProperty()->SetLineWidth(5.0);

            renderer2->AddActor(actor);
        };

        drawAxis(xAxis[i], 1.0, 0.0, 0.0);
        drawAxis(yAxis[i], 0.0, 1.0, 0.0);
        drawAxis(zAxis[i], 0.0, 0.0, 1.0);
    }

    vtkSmartPointer<vtkPoints> originPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> originVertices = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i < position.size(); ++i) {
        vtkIdType id = originPoints->InsertNextPoint(position[i].x, position[i].y, position[i].z);
        originVertices->InsertNextCell(1, &id);
    }

    vtkSmartPointer<vtkPolyData> originPolyData = vtkSmartPointer<vtkPolyData>::New();
    originPolyData->SetPoints(originPoints);
    originPolyData->SetVerts(originVertices);

    vtkSmartPointer<vtkPolyDataMapper> originMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    originMapper->SetInputData(originPolyData);

    vtkSmartPointer<vtkActor> originActor = vtkSmartPointer<vtkActor>::New();
    originActor->SetMapper(originMapper);
    originActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    originActor->GetProperty()->SetPointSize(10);

    renderer2->AddActor(originActor);

    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void SeamWidget::positioningResultDisplay(const PositioningResult& result) {
    qint64 elapsedMs = algorithmExecutionTimer.elapsed();
    PLOGD << "精定位算法执行时间: " << elapsedMs << " ms (" << elapsedMs / 1000.0 << " s)";

    if (!result.success) {
        PLOGE << "精定位失败";
        return;
    }

    PLOGD << "精定位结果可视化... ...";

    switch (result.weldType) {
        case WeldType::PlateToPlate: {
            PLOGD << "板板焊缝精定位结果可视化";
            //        pointCloudResultDisplay(result.cloudResult);
            displayLine(result.extremePoints);

            for (size_t i = 0; i < result.position.size(); ++i) {
                displayCoordinateaxisAndToolPosture(result.position[i], result.x[i], result.y[i], result.z[i],
                                                    result.toolPose[i]);
            }
        } break;

        case WeldType::TubeToPlate1: {
            PLOGD << "管板焊缝1精定位结果可视化";
            pointCloudResultDisplay(result.cloudResult);
            displayLine(result.extremePoints);
            //        displayCylinderModel(result.cylinderCoefficients, 0);
            //        displayCylinderModel(result.cylinderCoefficients2, 1);
        } break;

        case WeldType::TubeToPlate2: {
            PLOGD << "管板焊缝2精定位结果可视化";
            pointCloudResultDisplay(result.cloudResult);
            nurbsCurveDisplay(result.generatePoints);
        } break;

        case WeldType::TubeToTube: {
            PLOGD << "管管焊缝精定位结果可视化";
            pointCloudResultDisplay(result.cloudResult);
            nurbsCurveDisplay(result.generatePoints);
            nurbsCurveDisplay(result.generatePoints2);
        } break;
    }
}

void SeamWidget::on_btnAccPosition_clicked() {
    algorithmExecutionTimer.start();

    int weldTypeInt = -1;

    if (ui->radioButton->isChecked()) {
        weldTypeInt = 0;  // 板板焊接
        PLOGD << "选择焊缝类型: 板板焊接";
    } else if (ui->radioButton_2->isChecked()) {
        weldTypeInt = 1;  // 管板焊接1
        PLOGD << "选择焊缝类型: 管板焊接1";
    } else if (ui->radioButton_3->isChecked()) {
        weldTypeInt = 2;  // 管板焊接2
        PLOGD << "选择焊缝类型: 管板焊接2";
    } else if (ui->radioButton_4->isChecked()) {
        weldTypeInt = 3;  // 管管焊接
        PLOGD << "选择焊缝类型: 管管焊接";
    } else {
        QMessageBox::warning(this, u8"警告", u8"请先选择焊缝类型！");
        PLOGE << "未选择焊缝类型";
        return;
    }

    emit sendAccPosition(weldTypeInt, cloud);
}

void SeamWidget::on_btnLoadingPointCloud_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.pcd)");
    if (!fileName.isEmpty()) {
        std::string file_name = fileName.toStdString();
        pcl::io::loadPCDFile(file_name, *cloud);
        PLOGD << "加载点云文件";
    } else {
        QMessageBox::information(this, u8"警告", u8"读取失败！");
        PLOGE << "点云文件加载失败";
        return;
    }
    pointCloudDisplay(cloud);
}
