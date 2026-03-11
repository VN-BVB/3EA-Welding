SOURCES += \
    $$PWD/AbstractSeamDet.cpp \
    $$PWD/SeamDetWithPointCloud.cpp \
    $$PWD/SeamWidget.cpp \



HEADERS += \
    $$PWD/AbstractSeamDet.h \
    $$PWD/QhullLock.h \
    $$PWD/SeamDetWithPointCloud.h \
    $$PWD/SeamWidget.h \



FORMS += \
    $$PWD/SeamWidget.ui

include( ./accuratePositioning/accuratePositioning.pri )
