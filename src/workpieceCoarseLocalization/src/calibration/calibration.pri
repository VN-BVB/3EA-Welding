INCLUDEPATH += $$PWD/../..
HEADERS += \
    $$PWD/worker/CalibrationWorker.h \
    $$PWD/core/calibration_module/CalibrationModule.h \
    $$PWD/core/calibration_module/model/CoreTypes.h \
    $$PWD/core/calibration_module/calibration/CameraAndLaserPlaneCalibration.h \
    $$PWD/core/calibration_module/handeye/include/calibration.h \
    $$PWD/core/calibration_module/handeye/include/Camera_Calibration.h \
    $$PWD/core/calibration_module/handeye/include/handEyeCalibration.h \
    $$PWD/core/calibration_module/handeye/include/others.h

SOURCES += \
    $$PWD/worker/CalibrationWorker.cpp \
    $$PWD/core/calibration_module/CalibrationModule.cpp \
    $$PWD/core/calibration_module/calibration/CameraAndLaserPlaneCalibration.cpp \
    $$PWD/core/calibration_module/handeye/src/calibration.cpp \
    $$PWD/core/calibration_module/handeye/src/handEyeCalibration.cpp \
    $$PWD/core/calibration_module/handeye/src/others.cpp
