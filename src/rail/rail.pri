HEADERS += \
    $$PWD/AbstractAxis.h \
    $$PWD/AbstractAxisFactory.h \
    $$PWD/PLCCommunication.h \
    $$PWD/RailWidget.h \
    $$PWD/concrete_axis/AxisManager.h \
    $$PWD/concrete_axis/axis_register.h

SOURCES += \
    $$PWD/PLCCommunication.cpp \
    $$PWD/RailWidget.cpp \
    $$PWD/concrete_axis/AxisManager.cpp

FORMS += \
    $$PWD/RailWidget.ui
