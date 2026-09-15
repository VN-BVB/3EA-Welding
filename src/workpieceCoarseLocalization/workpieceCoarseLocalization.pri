INCLUDEPATH += $$PWD
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/src

SOURCES += \
    $$PWD/WorkpieceCoarseLocalization.cpp \
    $$PWD/include/CoarseLocalizationMatrix.cpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.cpp \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.cpp \
    $$PWD/src/calibration/worker/CalibrationWorker.cpp \
    $$PWD/src/calibration/core/calibration_module/CalibrationModule.cpp \
    $$PWD/src/calibration/core/calibration_module/calibration/CameraAndLaserPlaneCalibration.cpp \
    $$PWD/src/calibration/core/calibration_module/handeye/src/calibration.cpp \
    $$PWD/src/calibration/core/calibration_module/handeye/src/handEyeCalibration.cpp \
    $$PWD/src/calibration/core/calibration_module/handeye/src/others.cpp \
    $$PWD/src/yoloInference/YoloInference.cpp

HEADERS += \
    $$PWD/WorkpieceCoarseLocalization.h \
    $$PWD/include/CoarseLocalizationMatrix.h \
    $$PWD/include/maskImageProcessConfig.hpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.h \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h \
    $$PWD/src/calibration/worker/CalibrationWorker.h \
    $$PWD/src/calibration/core/calibration_module/CalibrationModule.h \
    $$PWD/src/calibration/core/calibration_module/model/CoreTypes.h \
    $$PWD/src/calibration/core/calibration_module/calibration/CameraAndLaserPlaneCalibration.h \
    $$PWD/src/calibration/core/calibration_module/handeye/include/calibration.h \
    $$PWD/src/calibration/core/calibration_module/handeye/include/Camera_Calibration.h \
    $$PWD/src/calibration/core/calibration_module/handeye/include/handEyeCalibration.h \
    $$PWD/src/calibration/core/calibration_module/handeye/include/others.h \
    $$PWD/src/yoloInference/YoloInference.h

FORMS += \
    $$PWD/WorkpieceCoarseLocalization.ui
