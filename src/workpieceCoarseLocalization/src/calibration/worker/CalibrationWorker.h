#ifndef CALIBRATIONWORKER_H
#define CALIBRATIONWORKER_H

#include <QObject>
#include <QString>

#include <limits>

class CalibrationWorker : public QObject
{
    Q_OBJECT

public:
    explicit CalibrationWorker(QObject *parent = nullptr);
    ~CalibrationWorker() override;

public slots:
    void runCameraCalibration(const QString &calibRootPath, const QString &resultFilePath);
    void runPlaneCalibration(const QString &calibRootPath, const QString &resultFilePath);
    void runHandEyeCalibration(const QString &calibRootPath, const QString &resultFilePath);
    void runExternalAxisCalibration(const QString &calibRootPath, const QString &resultFilePath, int axisTypeValue);
    void runQueryCoordinate(const QString &calibRootPath, const QString &resultFilePath, double pixelU, double pixelV);
    void updateCurrentXAxisEncoderValue(double currentEncoderValue);

signals:
    void messageRaised(const QString &message);
    void warningRaised(const QString &title, const QString &message);
    void calibrationFinished(const QString &calibrationName, bool success);
    void coordinateQueryFinished(const QString &resultText, bool success);

private:
    double currentXAxisEncoderValue = std::numeric_limits<double>::quiet_NaN();
};

#endif  // CALIBRATIONWORKER_H
