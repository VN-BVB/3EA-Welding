#ifndef COARSECALIBRATION_H
#define COARSECALIBRATION_H

#include <limits>

#include <QObject>
#include <QString>

#include "CalibrationData.h"

class CalibrationConfig;

class CoarseCalibration : public QObject {
    Q_OBJECT

public:
    explicit CoarseCalibration(QObject* parent = nullptr);
    ~CoarseCalibration() override;

public slots:
    //相机、平面、手眼、三轴方向向量标定槽函数（在主ui线程中通过invokeMethod方法调用）
    void whenCalibrateCamera(const QString& calibrationRootPath, const QString& resultFilePath);
    void whenCalibratePlane(const QString& calibrationRootPath, const QString& resultFilePath);
    void whenCalibrateHandEye(const QString& calibrationRootPath, const QString& resultFilePath);
    void whenCalibrateAxis(const QString& calibrationRootPath, const QString& resultFilePath, int axisTypeValue);
    //待定
    void whenQueryCoordinate(const QString& calibrationRootPath, const QString& resultFilePath, double pixelU, double pixelV);
    void whenUpdateCurrentXAxisEncoderValue(double encoderValue);

signals:
    void sendMessage(const QString& message);
    void sendCalibrationResult(const QString& calibrationName, bool success);
    void sendCoordinateResult(const QString& resultText, bool success);

private:
    // 保留 Windows 枚举顺序，仅筛选小写 .bmp，避免改变原有标定图片处理顺序。
    static void listBmpImages(const std::string& directory, std::vector<std::string>& imagePaths);
    bool buildCoordinateRequest(AxisCalibration::Type axisType, const cv::Point2d& pixelPoint, double currentEncoderValue,
                                CoordinateCalculation::AxisRequest& request) const;

    CalibrationConfig& calibrationConfig;  // 所有标定入口共享同一个资源管理实例。
    const BoardConfig boardConfig;         // 圆点板参数；圆心间距单位为米。
    double currentXAxisEncoderValue = std::numeric_limits<double>::quiet_NaN();
};

#endif  // COARSECALIBRATION_H
