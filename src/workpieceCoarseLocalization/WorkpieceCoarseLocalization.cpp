#include "WorkpieceCoarseLocalization.h"

#include "ui_WorkpieceCoarseLocalization.h"

std::string inferencePath = "./data/workpieceCoaLoc/Test";                                           // 推理路径
std::string trackFilePath = "./data/config/getTrackDirection.json";                                  // 地轨单位向量保存路径
std::string configFilePath = "./data/config/workpiece_localization_calib.json";                      // 相机参数保存路径
CoordinateMapper::CoordMappingType g_coordMappingType = CoordinateMapper::CoordMappingType::X_NegY;  // 机器人与像素坐标系之间的关系
std::string workbenchInsertGroup;                                                                    // 工作台选择
std::vector<cv::Mat> cvImagesCameraOri;                                                              // 推理前相机原图
std::vector<cv::Mat> cvImagesWorkpieceSeg;                                                           // 推理后相机工件掩膜图
std::vector<cv::Mat> cvImagesWpMaskOri;                                                              // 焊缝检测前工件掩膜原图
std::vector<cv::Mat> cvImagesWpMaskDet;                                                              // 焊缝检测后工件掩膜图
workpieceBoxInWorld workpieceFinalInfoInWorld;             // 保存确认前的结果，结果进度为分割工件
workpieceBoxInWorld workpieceFinalInfoInWorldAfterVerify;  // 保存确认后的结果，结果进度为检测焊缝
workpieceBoxInWorld workpieceFinalInfoInWorldAfterIOU;     // 保存iou合并，排序的数据，结果进度为全部

WorkpieceCoarseLocalization::WorkpieceCoarseLocalization(QWidget* parent) : QWidget(parent), ui(new Ui::WorkpieceCoarseLocalization) {
    ui->setupUi(this);

    // 拟合工件手动控件
    connect(ui->mapView, &ScalableGraphicsView::senderSignalPixelCoordinates, fittingWorkpieceCoordinate,
            &FittingWorkpieceCoordinate::handleClickEvent);  //
    connect(ui->mapView, &ScalableGraphicsView::senderSignalPixelCoordinates, this, &WorkpieceCoarseLocalization::whenViewWorldCoordinateLabel);
    // 注册Qt没有的数据类型
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<std::vector<segYolo11::ObjectYolo11Seg>>("std::vector<segYolo11::ObjectYolo11Seg>");
    qRegisterMetaType<std::vector<det::Object>>("std::vector<det::Object>");
    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<std::vector<cv::Point3d>>("std::vector<cv::Point3d>");
    qRegisterMetaType<std::vector<std::vector<std::array<double, 4>>>>("std::vector<std::vector<std::array<double, 4>>>");
    qRegisterMetaType<workpieceBoxInWorld>("workpieceBoxInWorld");
    qRegisterMetaType<std::shared_ptr<workpieceBoxInWorld>>("std::shared_ptr<workpieceBoxInWorld>");

    // 深度学习与主线程
    yolo11SegInference->moveToThread(fittingWorkpieceSubThread);
    yolo11RectInference->moveToThread(fittingWorkpieceSubThread);
    fittingWorkpieceCoordinate->moveToThread(fittingWorkpieceSubThread);
    connect(this, &WorkpieceCoarseLocalization::sendCommandToInferPath, yolo11SegInference, &YoloSegInference::whenPathNeedToInfer);
    connect(yolo11SegInference, &YoloSegInference::sendInferResultToMainWindow, this, &WorkpieceCoarseLocalization::whenGetInferResult);
    connect(yolo11RectInference, &YoloDetInference::sendInferResultToMainWindow, this, &WorkpieceCoarseLocalization::whenGetInferResult);

    // 深度学习-拟合工件坐标坐标
    connect(yolo11SegInference, &YoloSegInference::sendCoordinateTofit, fittingWorkpieceCoordinate,
            &FittingWorkpieceCoordinate::whenFittingWorkpieceCoordinate);
    connect(yolo11SegInference, &YoloSegInference::sendSignalTocalculate, fittingWorkpieceCoordinate,
            &FittingWorkpieceCoordinate::whenFinishInferrence);
    connect(yolo11SegInference, &YoloSegInference::sendAppendInferLog, this, &WorkpieceCoarseLocalization::whenAppendLog);
    connect(fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::sendWorkpieceResultToMainWindow, this,
            &WorkpieceCoarseLocalization::whenGetWorkpieceRailMap);
    connect(fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::appendFittingLog, this, &WorkpieceCoarseLocalization::whenAppendLog);
    connect(this, &WorkpieceCoarseLocalization::sendVerifyCoordinatesInManual, fittingWorkpieceCoordinate,
            &FittingWorkpieceCoordinate::whenVerifyWorkpieceCoordinates);
    connect(fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::sendFinalInfoToMain, this, &WorkpieceCoarseLocalization::getLocalizationResult);
    connect(fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::sendWorkpieceMaskImageInWorld, yolo11RectInference,
            &YoloDetInference::whenRecieveWpMaskInWorld);
    connect(yolo11RectInference, &YoloDetInference::sendBoxInfoToDisplay, fittingWorkpieceCoordinate,
            &FittingWorkpieceCoordinate::whenGetWeldBoxInfo);
    connect(fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::sendUpdateInferedCameraImgNum, this,
            &WorkpieceCoarseLocalization::whenUpdateComboCameraImg);

    // 相机
    baslerControl->moveToThread(cameraControlSubThread);
    connect(this, &WorkpieceCoarseLocalization::sendOpenCamera, baslerControl, &CoarsePositioningCamera::openCamera);
    connect(this, &WorkpieceCoarseLocalization::sendDisconnectCamera, baslerControl, &CoarsePositioningCamera::closeCamera);
    connect(this, &WorkpieceCoarseLocalization::sendCameraToShow, baslerControl, &CoarsePositioningCamera::whenChooseCameraToShow);
    connect(this, &WorkpieceCoarseLocalization::sendSignalToInfer, baslerControl, &CoarsePositioningCamera::whenCameraImageInfer);
    connect(baslerControl, &CoarsePositioningCamera::sendImageToView, this, &WorkpieceCoarseLocalization::whenGetImage);
    connect(baslerControl, &CoarsePositioningCamera::sendSerialNumber, this, &WorkpieceCoarseLocalization::whenUpdateComboBox);
    connect(baslerControl, &CoarsePositioningCamera::sendCvImagesToInfer, yolo11SegInference, &YoloSegInference::whenImageNeedToInfer);
    connect(baslerControl, &CoarsePositioningCamera::appendCameraLog, this, &WorkpieceCoarseLocalization::whenAppendLog);

    // inferenceSubThread->start();
    cameraControlSubThread->start();
    fittingWorkpieceSubThread->start();
}

WorkpieceCoarseLocalization::~WorkpieceCoarseLocalization() {
    delete ui;
    delete yolo11SegInference;
    delete yolo11RectInference;
    delete fittingWorkpieceCoordinate;
}

void WorkpieceCoarseLocalization::whenGetImage(cv::Mat res) { ui->qImageWidget->setOpenCVImage(res); }

void WorkpieceCoarseLocalization::whenGetInferResult(cv::Mat res) {
    if (res.empty()) {
        PLOGE << "推理结果 res 为空";
        return;
    }
    if (res.channels() != 3) {
        PLOGE << "推理结果通道数异常: " << res.channels();
    }
    ui->qImageWidget->setOpenCVImage(res);
}

void WorkpieceCoarseLocalization::whenGetWorkpieceResult(cv::Mat res) { ui->qImageWidget->setOpenCVImage(res); }

void WorkpieceCoarseLocalization::whenGetWorkpieceRailMap(cv::Mat res) {
    QImage img = QImage(res.data, res.cols, res.rows, res.step, QImage::Format_RGB888);
    // 开始绘图
    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);

    // 设置字体（高度约为图像高度的30%）
    int fontPixelSize = res.rows * 0.3 / 2;
    QFont font("Microsoft YaHei", fontPixelSize);
    painter.setFont(font);
    painter.setPen(QColor(255, 0, 0));  // 黑色文字

    // 设置绘图区域：图像上半部分
    QString text;
    QRect textRect;
    if (workbenchInsertGroup == "positive") {
        // text = u8"南工作台在上面";
        text = u8"↑↑↑↑↑↑↑↑↑↑↑↑";
        textRect = QRect(0, res.rows / 2, res.cols, res.rows / 2);
    } else if (workbenchInsertGroup == "negative") {
        // text = u8"北工作台在下面";
        text = u8"↓↓↓↓↓↓↓↓↓↓↓↓";
        textRect = QRect(0, 0, res.cols, res.rows / 2);
    }
    painter.drawText(textRect, Qt::AlignCenter, text);
    painter.end();
    // 获取 QGraphicsScene
    QGraphicsScene* scene = ui->mapView->scene();  // 获取 mapView 的场景
    // 创建 QGraphicsPixmapItem 并将图像添加到场景中
    // QPixmap pixmap("D:/YOLO/yolo11_Seg_C++/data/workpieceCoaLoc/Test/camera_0_20250402_131335.bmp");
    QGraphicsPixmapItem* pixmapItem = new QGraphicsPixmapItem(QPixmap::fromImage(img));

    // 清空场景并添加新的图像项到场景
    scene->clear();              // 清空场景上的所有项
    scene->addItem(pixmapItem);  // 添加图像项到场景
    ui->mapView->setOriginalImageInfo(res.cols, res.rows, railMapRotationAngle);

    // 更新视图（如果没有立即显示，尝试刷新视图）
    ui->mapView->setScene(scene);  // 确保场景设置正确
}

void WorkpieceCoarseLocalization::whenViewWorldCoordinateLabel(int x, int y) { ui->coordinateLabel->setText(QString("X: %1, Y: %2").arg(x).arg(y)); }
// 这个函数中的求方向向量部分应该在fitting中实现，不应该在ui中，后续要进行修改，应该在这个函数的上一级；
void WorkpieceCoarseLocalization::getLocalizationResult(const workpieceBoxInWorld& workpieceBoxInfoInWorld) {
    resultPtr = std::make_shared<workpieceBoxInWorld>(workpieceBoxInfoInWorld);  // std::shared_ptr<workpieceBoxInWorld>
    whenUpdateComboWp(static_cast<int>(workpieceBoxInfoInWorld.workpieceInfoInWorld.size()));

    if (resultPtr->workpieceInfoInWorld.size() > 0) {
        cv::Mat res = resultPtr->workpieceInfoInWorld[0].workpiece_weld_Mask.second;
        if (!res.empty()) {
            // PLOGD << "getLocalizationResult";
            ui->qImageWidget->setOpenCVImage(res);
        }
    }

    emit sendCoarseLoc(resultPtr);  // 发出粗定位信息到系统控制类
    // return resultPtr;
}

void WorkpieceCoarseLocalization::whenAppendLog(const QString message) { ui->textCalibratation->append(message); }

void WorkpieceCoarseLocalization::whenUpdateComboBox(const std::vector<std::string>& SerialNumbers) {
    ui->comboCameras->clear();  // 清空现有项

    // 将每个序列号添加到 QComboBox
    for (const auto& serial : SerialNumbers) {
        ui->comboCameras->addItem(QString::fromStdString(serial));  // 将 std::string 转为 QString
    }

    emit sendCameraComboBox(SerialNumbers);  // // 发出粗定位相机下拉框信息
}
void WorkpieceCoarseLocalization::whenUpdateComboWp(const int size) {
    ui->comboWorkpieceNum->clear();  // 清空现有项

    for (int i = 0; i < size; ++i) {
        ui->comboWorkpieceNum->addItem(QString::number(i));
    }

    emit sendCoarseLocWorkpieceNum(size);  // 发出粗定位工件数量
}

void WorkpieceCoarseLocalization::startCoarseLocalization() { emit sendSignalToInfer(); }

void WorkpieceCoarseLocalization::printWorkpieceBoxInfo(const workpieceBoxInWorld* info, int workpieceIndex) {
    if (!info) {
        std::cout << "[printWorkpieceBoxInfo] nullptr received!" << std::endl;
        return;
    }

    const auto& vec = info->workpieceInfoInWorld;
    // qDebug() << "---vec---" << vec.size();
    if (workpieceIndex < 0 || workpieceIndex >= vec.size()) {
        std::cout << "[printWorkpieceBoxInfo] Invalid index: " << workpieceIndex << " (size: " << vec.size() << ")" << std::endl;
        return;
    }

    const auto& wp = vec[workpieceIndex];
    std::cout << "========== workpiece[" << workpieceIndex << "] Debug Info ==========" << std::endl;

    // 原图和掩膜图
    std::cout << "[cameraOriginalMat] size: " << wp.cameraOriginalMat.size() << std::endl;
    if (!wp.cameraOriginalMat.empty()) cv::imshow("cameraOriginalMat_" /*+ std::to_string(workpieceIndex) */, wp.cameraOriginalMat);

    std::cout << "[cameraSegMat] size: " << wp.cameraSegMat.size() << std::endl;
    if (!wp.cameraSegMat.empty()) cv::imshow("cameraSegMat_" /*+ std::to_string(workpieceIndex) */, wp.cameraSegMat);

    // 焊缝掩膜
    std::cout << "[workpiece_weld_Mask] before/after:" << wp.workpiece_weld_Mask.first.size() << " / " << wp.workpiece_weld_Mask.second.size()
              << std::endl;
    if (!wp.workpiece_weld_Mask.first.empty()) cv::imshow("weldMask_before_" /*+ std::to_string(workpieceIndex) */, wp.workpiece_weld_Mask.first);
    if (!wp.workpiece_weld_Mask.second.empty()) cv::imshow("weldMask_after_" /*+ std::to_string(workpieceIndex) */, wp.workpiece_weld_Mask.second);

    // 焊缝检测对象
    const auto& segObj = wp.workpiece_weld_Obj.first;
    const auto& detObjs = wp.workpiece_weld_Obj.second;
    std::cout << "[workpiece_weld_Obj] segLabel: " << segObj.label << ", prob: " << segObj.prob << std::endl;
    std::cout << "  Detected objects: " << detObjs.size() << std::endl;
    for (size_t j = 0; j < detObjs.size(); ++j) {
        const auto& d = detObjs[j];
        // std::cout << "    Det[" << j << "]: label=" << d.label << ", prob=" << d.prob << ", rect=(" << d.rect.x << "," <<
        // d.rect.y
        // << ","
        //           << d.rect.width << "," << d.rect.height << ")" << ", rotated_rect.angle=" << d.rotated_rect.angle <<
        //           std::endl;
    }

    // 区域坐标
    const auto& center = wp.workpieceAreaRect.first;
    const auto& topleft = wp.workpieceAreaRect.second;
    std::cout << "[workpieceAreaRect] center=(" << center.x << "," << center.y << "," << center.z << "), topleft=(" << topleft.x << "," << topleft.y
              << "," << topleft.z << ")" << std::endl;

    // 焊缝区域矩形框
    std::cout << "[weldAreaRect] count: " << wp.weldAreaRect.size() << std::endl;
    for (size_t j = 0; j < wp.weldAreaRect.size(); ++j) {
        const auto& r = wp.weldAreaRect[j];
        std::cout << "  Rect[" << j << "]: x=" << r.x << ", y=" << r.y << ", w=" << r.width << ", h=" << r.height << std::endl;
    }

    // IOU 信息
    for (const auto& iou : wp.workpieceIouInfos) {
        if (!iou.cameraOriginalMat.empty()) {
            std::cout << "========== [workpieceIouInfo] ==========" << std::endl;

            std::cout << "  [cameraOriginalMat] size: " << iou.cameraOriginalMat.size() << std::endl;
            if (!iou.cameraOriginalMat.empty()) cv::imshow("iou_cameraOriginalMat_" /*+ std::to_string(workpieceIndex) */, iou.cameraOriginalMat);

            std::cout << "  [cameraSegMat] size: " << iou.cameraSegMat.size() << std::endl;
            if (!iou.cameraSegMat.empty()) cv::imshow("iou_cameraSegMat_" /*+ std::to_string(workpieceIndex) */, iou.cameraSegMat);

            std::cout << "  [workpiece_weld_Mask] before size: " << iou.workpiece_weld_Mask.first.size()
                      << ", after size: " << iou.workpiece_weld_Mask.second.size() << std::endl;
            if (!iou.workpiece_weld_Mask.first.empty())
                cv::imshow("iou_weldMask_before_" /*+ std::to_string(workpieceIndex) */, iou.workpiece_weld_Mask.first);
            if (!iou.workpiece_weld_Mask.second.empty())
                cv::imshow("iou_weldMask_after_" /*+ std::to_string(workpieceIndex) */, iou.workpiece_weld_Mask.second);

            const auto& iouSegObj = iou.workpiece_weld_Obj.first;
            const auto& iouDetObjs = iou.workpiece_weld_Obj.second;
            std::cout << "  [workpiece_weld_Obj] segLabel: " << iouSegObj.label << ", prob: " << iouSegObj.prob << std::endl;
            std::cout << "    Detected objects: " << iouDetObjs.size() << std::endl;
            for (size_t j = 0; j < iouDetObjs.size(); ++j) {
                const auto& d = iouDetObjs[j];
                // std::cout << "    Det[" << j << "]: label=" << d.label << ", prob=" << d.prob << ", rect=(" << d.rect.x << ","
                // << d.rect.y << ","
                //           << d.rect.width << "," << d.rect.height << ")" << ", rotated_rect.angle=" << d.rotated_rect.angle <<
                //           std::endl;
            }
        }
    }

    // 全局信息
    std::cout << "[trackDirection] size: " << info->trackDirection.rows << "x" << info->trackDirection.cols << std::endl;
    if (!info->trackDirection.empty()) {
        std::cout << info->trackDirection << std::endl;
        // cv::imshow("trackDirection", info->trackDirection);
    }

    std::cout << "[finalRailMap] size: " << info->finalRailMap.rows << "x" << info->finalRailMap.cols << std::endl;
    if (!info->finalRailMap.empty()) {
        cv::imshow("finalRailMap", info->finalRailMap);
    }

    cv::waitKey(0);
}
void WorkpieceCoarseLocalization::whenNeedToSaveImg() {
    // 获取当前时间并格式化
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream dateTimeStream;
    dateTimeStream << std::put_time(localTime, "%Y%m%d_%H%M%S");

    // 保存相机原图
    if (cvImagesCameraOri.empty()) {
        emit sendMessage2MainWindow(u8"相机原图列表为空，跳过保存");
    } else {
        for (size_t i = 0; i < cvImagesCameraOri.size(); ++i) {
            if (cvImagesCameraOri[i].empty()) {
                emit sendMessage2MainWindow(QString(u8"相机 %1 原图为空，跳过保存").arg(i + 1));
                continue;
            }
            std::string filename = segSavePath + "/camera" + std::to_string(i) + "_" + dateTimeStream.str() + "_ori.bmp";
            if (!cv::imwrite(filename, cvImagesCameraOri[i])) {
                emit sendMessage2MainWindow(QString(u8"保存相机 %1 原图失败").arg(i + 1));
            }
        }
    }

    // 保存分割图
    if (cvImagesWorkpieceSeg.empty()) {
        emit sendMessage2MainWindow(u8"工件分割图列表为空，跳过保存");
    } else {
        for (size_t i = 0; i < cvImagesWorkpieceSeg.size(); ++i) {
            if (cvImagesWorkpieceSeg[i].empty()) {
                emit sendMessage2MainWindow(QString(u8"工件 %1 分割图为空，跳过保存").arg(i + 1));
                continue;
            }
            std::string filename = segSavePath + "/camera" + std::to_string(i) + "_" + dateTimeStream.str() + "_result.bmp";
            if (!cv::imwrite(filename, cvImagesWorkpieceSeg[i])) {
                emit sendMessage2MainWindow(QString(u8"保存工件 %1 分割图失败").arg(i + 1));
            }
        }
    }

    // 保存掩膜原图
    if (cvImagesWpMaskOri.empty()) {
        emit sendMessage2MainWindow(u8"工件掩膜原图列表为空，跳过保存");
    } else {
        for (size_t i = 0; i < cvImagesWpMaskOri.size(); ++i) {
            if (cvImagesWpMaskOri[i].empty()) {
                emit sendMessage2MainWindow(QString(u8"工件 %1 掩膜原图为空，跳过保存").arg(i + 1));
                continue;
            }
            std::string filename = detSavePath + "/workpiece" + std::to_string(i) + "_" + dateTimeStream.str() + "_ori.bmp";
            if (!cv::imwrite(filename, cvImagesWpMaskOri[i])) {
                emit sendMessage2MainWindow(QString(u8"保存工件 %1 掩膜原图失败").arg(i + 1));
            }
        }
    }

    // 保存掩膜检测图
    if (cvImagesWpMaskDet.empty()) {
        emit sendMessage2MainWindow(u8"工件掩膜检测图列表为空，跳过保存");
    } else {
        for (size_t i = 0; i < cvImagesWpMaskDet.size(); ++i) {
            if (cvImagesWpMaskDet[i].empty()) {
                emit sendMessage2MainWindow(QString(u8"工件 %1 掩膜检测图为空，跳过保存").arg(i + 1));
                continue;
            }
            std::string filename = detSavePath + "/workpiece" + std::to_string(i) + "_" + dateTimeStream.str() + "_result.bmp";
            if (!cv::imwrite(filename, cvImagesWpMaskDet[i])) {
                emit sendMessage2MainWindow(QString(u8"保存工件 %1 掩膜检测图失败").arg(i + 1));
            }
        }
    }

    emit sendMessage2MainWindow(u8"图像保存完毕");
}

void WorkpieceCoarseLocalization::whenImageNeedToInfer(std::string workbenchGroup, std::string locationMode) {
    workbenchInsertGroup = workbenchGroup;
    if (locationMode == "SimulatePosition") {
        emit sendCommandToInferPath(inferencePath);
    } else if (locationMode == "CameraPosition") {
        startCoarseLocalization();
    }
}
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
void WorkpieceCoarseLocalization::on_btnConnectCamera_clicked() {
    baslerControl->cameraFlag = true;
    emit sendOpenCamera();
}

void WorkpieceCoarseLocalization::on_btnDisconnectCamera_clicked() {
    baslerControl->cameraFlag = false;
    emit sendDisconnectCamera();
    ui->comboCameras->clear();
    cv::Mat res(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
    ui->qImageWidget->setOpenCVImage(res);
    if (timer && timer->isActive()) {
        timer->stop();
    }
}

void WorkpieceCoarseLocalization::on_btnSaveImage_clicked() {
    // if (!timer) {
    //     timer = new QTimer(this);

    //     connect(timer, &QTimer::timeout, this, [this]() { handleSaveImageLogic(); });
    // }

    // timer->start(1500);  // 每1000ms触发一次
    handleSaveImageLogic();
}
void WorkpieceCoarseLocalization::handleSaveImageLogic() {
    QString selectedOption = ui->comboSaveImage->currentText();
    int saveType = -1;
    bool validOption = true;

    if (selectedOption == u8"相机手眼标定") {
        saveType = 0;
    } else if (selectedOption == u8"平面拟合") {
        saveType = 1;
    } else if (selectedOption == u8"地轨标定") {
        saveType = 2;
    } else if (selectedOption == u8"保存图像") {
    } else {
        qDebug() << u8"未选择有效操作！";
        validOption = false;
    }

    if (validOption) {
        baslerControl->imageNumberToSaveInCalibration++;
        baslerControl->saveTypeEnable = saveType;
    }
}
void WorkpieceCoarseLocalization::on_btnInferPath_clicked() { emit sendCommandToInferPath(inferencePath); }

void WorkpieceCoarseLocalization::on_btnStartInfer_clicked() { startCoarseLocalization(); }

void WorkpieceCoarseLocalization::on_btn_VerifyCoordinates_clicked() { emit sendVerifyCoordinatesInManual(); }

void WorkpieceCoarseLocalization::on_comboCameras_currentTextChanged(const QString& currentCamera) {
    std::string CurrentCamera = currentCamera.toStdString();
    baslerControl->currentS_N = CurrentCamera;
    emit sendCameraToShow();
}

void WorkpieceCoarseLocalization::on_btnGetWorkpieceInfo_clicked() {
    int selectedIndex = ui->comboWorkpieceNum->currentIndex();  // 获取当前选中项的索引
    printWorkpieceBoxInfo(resultPtr.get(), selectedIndex);      // 使用这个索引替代原来的固定值
}

void WorkpieceCoarseLocalization::on_comboWorkpieceNum_currentIndexChanged(int index) {
    if (index >= 0) {
        cv::Mat res = resultPtr->workpieceInfoInWorld[index].workpiece_weld_Mask.second;
        const auto& iouInfos = resultPtr->workpieceInfoInWorld[index].workpieceIouInfos;

        if (!iouInfos.empty()) {
            int n = static_cast<int>(iouInfos.size());

            // 缩放后用于上下拼接的掩膜列表
            std::vector<cv::Mat> resizedIouMats;
            for (const auto& iouInfo : iouInfos) {
                cv::Mat mask = iouInfo.workpiece_weld_Mask.second;
                if (mask.empty()) continue;

                // 缩放为宽度与主体一致，高度为主体高度除以iou数量
                cv::Mat resizedMask;
                cv::resize(mask, resizedMask, cv::Size(res.cols, res.rows / n));
                resizedIouMats.push_back(resizedMask);
            }

            // 竖直拼接 iou 掩膜图（如果都为空，这步会跳过）
            cv::Mat verticalConcatIou;
            if (!resizedIouMats.empty()) {
                cv::vconcat(resizedIouMats, verticalConcatIou);
            } else {
                verticalConcatIou = cv::Mat::zeros(res.rows, res.cols, res.type());  // 如果全空，生成空白
            }

            // 左右拼接 主体 和 iou 掩膜图
            cv::Mat concatResult;
            cv::Mat resPadded, iouPadded;
            if (res.rows != verticalConcatIou.rows) {
                PLOGD << "拼接图像适应中";
                int maxRows = std::max(res.rows, verticalConcatIou.rows);
                cv::copyMakeBorder(res, resPadded, 0, maxRows - res.rows, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
                cv::copyMakeBorder(verticalConcatIou, iouPadded, 0, maxRows - verticalConcatIou.rows, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
            } else {
                resPadded = res.clone();
                iouPadded = verticalConcatIou.clone();
            }
            cv::hconcat(resPadded, iouPadded, concatResult);

            ui->qImageWidget->setOpenCVImage(concatResult);
            emit sendImg2MainWindow(concatResult);
        } else {
            // 无 iou 信息，直接显示主体
            ui->qImageWidget->setOpenCVImage(res);
            emit sendImg2MainWindow(res);
        }
    }
}
void WorkpieceCoarseLocalization::whenUpdateComboCameraImg() {
    ui->comboCameraInferedNum->clear();  // 清空现有项
    if (cvImagesWorkpieceSeg.size() > 0 && cvImagesWorkpieceSeg.size() < 4) {
        for (int i = 0; i < cvImagesWorkpieceSeg.size(); ++i) {
            ui->comboCameraInferedNum->addItem(QString::number(i));
        }
    } else if (cvImagesWorkpieceSeg.size() > 4) {
        for (int i = 0; i < 3; ++i) {
            ui->comboCameraInferedNum->addItem(QString::number(i));
        }
    }
}
void WorkpieceCoarseLocalization::on_comboCameraInferedNum_currentIndexChanged(int index) {
    if (index >= 0 && cvImagesWorkpieceSeg.size() > 0) {
        if (cvImagesWorkpieceSeg.size() > 4) {
            ui->qImageWidget->setOpenCVImage(cvImagesWorkpieceSeg[index + 3]);
        } else {
            ui->qImageWidget->setOpenCVImage(cvImagesWorkpieceSeg[index]);
        }
    }
}
