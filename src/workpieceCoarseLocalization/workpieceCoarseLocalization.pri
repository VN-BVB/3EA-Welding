INCLUDEPATH += $$PWD
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/src
INCLUDEPATH += $$PWD/src/calibration
INCLUDEPATH += $$PWD/../..

SOURCES += \
    $$PWD/WorkpieceCoarseLocalization.cpp \
    $$PWD/include/CoarseLocalizationMatrix.cpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.cpp \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.cpp \
    $$PWD/src/yoloInference/YoloInference.cpp \
    $$PWD/src/calibration/config/CalibrationConfig.cpp \
    $$PWD/src/calibration/calculation/CameraCalibration.cpp \
    $$PWD/src/calibration/calculation/PlaneCalibration.cpp \
    $$PWD/src/calibration/calculation/AxisCalibration.cpp \
    $$PWD/src/calibration/calculation/CoordinateCalculation.cpp \
    $$PWD/src/calibration/CoarseCalibration.cpp \
    $$PWD/src/calibration/handeye/Calibration.cpp \
    $$PWD/src/calibration/handeye/HandEyeCalibration.cpp \
    $$PWD/src/calibration/handeye/HandEyeCalculation.cpp \
    $$PWD/src/calibration/common/BoardDetection.cpp \
    $$PWD/src/calibration/common/PoseCalculation.cpp \
    $$PWD/src/calibration/common/CalibrationStatistics.cpp \
    $$PWD/src/calibration/common/CalibrationPlot.cpp \
    $$PWD/src/calibration/common/CameraPlaneAlgorithms.cpp

HEADERS += \
    $$PWD/WorkpieceCoarseLocalization.h \
    $$PWD/include/CoarseLocalizationMatrix.h \
    $$PWD/include/maskImageProcessConfig.hpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.h \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h \
    $$PWD/src/yoloInference/YoloInference.h \
    $$PWD/src/calibration/CalibrationData.h \
    $$PWD/src/calibration/config/CalibrationConfig.h \
    $$PWD/src/calibration/calculation/CameraCalibration.h \
    $$PWD/src/calibration/calculation/PlaneCalibration.h \
    $$PWD/src/calibration/calculation/AxisCalibration.h \
    $$PWD/src/calibration/calculation/CoordinateCalculation.h \
    $$PWD/src/calibration/CoarseCalibration.h \
    $$PWD/src/calibration/handeye/Calibration.h \
    $$PWD/src/calibration/handeye/HandEyeCalibration.h \
    $$PWD/src/calibration/handeye/HandEyeCalculation.h \
    $$PWD/src/calibration/common/BoardDetection.h \
    $$PWD/src/calibration/common/PoseCalculation.h \
    $$PWD/src/calibration/common/CalibrationStatistics.h \
    $$PWD/src/calibration/common/CalibrationPlot.h \
    $$PWD/src/calibration/common/CameraPlaneAlgorithms.h

FORMS += \
    $$PWD/WorkpieceCoarseLocalization.ui
