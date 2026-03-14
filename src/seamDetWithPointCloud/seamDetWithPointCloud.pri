SOURCES += \
    $$PWD/SeamWidget.cpp \
    $$PWD/AbstractSeamDet.cpp \
    $$PWD/SeamDetWithPointCloud.cpp


HEADERS += \
    $$PWD/AbstractSeamDet.h \
    $$PWD/QhullLock.h \
    $$PWD/SeamWidget.h \
    $$PWD/SeamDetWithPointCloud.h


FORMS += \
    $$PWD/SeamWidget.ui

include( ./steelDefaultDet/steelDefaultDet.pri)
include( ./steelAngelDet/steelAngelDet.pri )
