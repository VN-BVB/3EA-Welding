SOURCES += \
    $$PWD/WorkpieceCoarseLocalization.cpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.cpp \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.cpp \
    $$PWD/src/yoloInference/YoloInference.cpp

HEADERS += \
    $$PWD/WorkpieceCoarseLocalization.h \
    $$PWD/include/CoarseLocalizationMatrix.h \
    $$PWD/include/maskImageProcessConfig.hpp \
    $$PWD/src/camera_control/basler/CoarsePositioningCamera.h \
    $$PWD/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h \
    $$PWD/src/yoloInference/YoloInference.h

FORMS += \
    $$PWD/WorkpieceCoarseLocalization.ui
