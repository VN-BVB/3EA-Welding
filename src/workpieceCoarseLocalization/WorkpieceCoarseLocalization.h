#ifndef WORKPIECECOARSELOCALIZATIONMAINWINDOW_H
#define WORKPIECECOARSELOCALIZATIONMAINWINDOW_H

#include <QMainWindow>
#include <QMetaType>
#include <QMouseEvent>
#include <QThread>

#include "src/camera_control/basler/CoarsePositioningCamera.h"
#include "src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h"
#include "src/utils/image_widget/ScalableGraphicsView.h"
#include "src/yoloInference/YoloInference.h"
#include "src/calibration/worker/CalibrationWorker.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class WorkpieceCoarseLocalization;
}
QT_END_NAMESPACE

class WorkpieceCoarseLocalization : public QWidget {
    Q_OBJECT
public:
    WorkpieceCoarseLocalization(QWidget *parent = nullptr);
    ~WorkpieceCoarseLocalization();

private:
    Ui::WorkpieceCoarseLocalization *ui;

    YoloSegInference *yolo11SegInference = new YoloSegInference;                              // 分割类
    YoloDetInference *yolo11RectInference = new YoloDetInference;                             // 目标检测类
    FittingWorkpieceCoordinate *fittingWorkpieceCoordinate = new FittingWorkpieceCoordinate;  // 工件拟合
    CoarsePositioningCamera *baslerControl = new CoarsePositioningCamera;                     // 相机类
    CalibrationWorker *calibrationWorker = new CalibrationWorker;                             // 标定类

    // QThread *inferenceSubThread = new QThread;         // 深度学习推理线程
    QThread *cameraControlSubThread = new QThread;     // 相机线程
    QThread *fittingWorkpieceSubThread = new QThread;  // 坐标拟合线程
    QThread *calibrationThread = new QThread;          //标定线程
    QGraphicsScene *scene = new QGraphicsScene;        // 创建一个 QGraphicsScene
    std::shared_ptr<workpieceBoxInWorld> resultPtr;
    bool detectionEnabled;
    QTimer *timer = nullptr;

    void handleSaveImageLogic();
private slots:
    void on_btnConnectCamera_clicked();
    void on_btnSaveImage_clicked();
    void on_btnInferPath_clicked();
    void on_btnStartInfer_clicked();
    void on_btn_VerifyCoordinates_clicked();
    void on_comboCameras_currentTextChanged(const QString &currentCamera);
    void on_btnDisconnectCamera_clicked();
    void on_btnGetWorkpieceInfo_clicked();
    void on_comboWorkpieceNum_currentIndexChanged(int index);
    void whenNeedToSaveImg();
    void on_comboCameraInferedNum_currentIndexChanged(int index);
    void on_btnStartCalibration_clicked();

public slots:
    void startCoarseLocalization();
    void whenGetInferResult(cv::Mat res);
    void whenAppendLog(const QString message);
    void whenGetImage(cv::Mat res);
    void whenGetWorkpieceResult(cv::Mat res);
    void whenGetWorkpieceRailMap(cv::Mat res);
    void whenUpdateComboBox(const std::vector<std::string> &SerialNumbers);
    void whenUpdateComboCameraImg();
    void whenUpdateComboWp(const int size);
    void whenViewWorldCoordinateLabel(int x, int y);
    void getLocalizationResult(const workpieceBoxInWorld &workpieceBoxInfoInWorld);
    void printWorkpieceBoxInfo(const workpieceBoxInWorld *info, int workpieceIndex);
    void whenImageNeedToInfer(std::string workbenchGroup, std::string locationMode);
signals:
    void sendCommandToInferPath(std::string path);
    void sendDisconnectCamera();
    void sendOpenCamera();
    void sendSignalToCalibratate();
    void sendSignalToSaveCalibPara();
    void sendVerifyCoordinatesInManual();
    void sendRobotConnect();
    void sendRobotDisconnect();
    void sendCameraToShow();
    void sendSignalToInfer();

    void sendCoarseLoc(std::shared_ptr<workpieceBoxInWorld> coarseLocRes);  // 发出工件粗定位完成信号
    void sendCameraComboBox(std::vector<std::string> cameraSerial);         // 发出粗定位相机下拉框信息
    void sendCoarseLocWorkpieceNum(int num);                                // 发出粗定位工件数量
    void sendImg2MainWindow(cv::Mat img);                                   // 发出图像到主UI
    void sendMessage2MainWindow(QString message);

    friend class SettingWidget;
    friend class RailWeldingSystem;
    friend class WeldingMainWindow;
};

#endif  // WORKPIECECOARSELOCALIZATIONMAINWINDOW_H
