QT       += core gui serialbus network opengl concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += release debug_info

TARGET = 3ea_robot_welding
TEMPLATE = app
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += NOMINMAX
DEFINES += _CRT_SECURE_NO_WARNINGS
DEFINES += _SCL_SECURE_NO_WARNINGS
DEFINES += _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING

QMAKE_LFLAGS += -openmp
QMAKE_CXXFLAGS += -openmp
QMAKE_CXXFLAGS += /MP
QMAKE_CXXFLAGS_RELEASE = -ZI -MD
# 在发布模式下, 也可以使用调试器来调试程序。
QMAKE_LFLAGS_RELEASE = /DEBUG

DEFINES += SMART_CAMERA
QMAKE_CXXFLAGS += /bigobj


# 以下为第三方库配置选项
DEFINES += LI_CONFIG
# DEFINES += ROM_CONFIG

contains(DEFINES, LI_CONFIG)  { include(./3rdParty/3rdPartyLi.pri) }
contains(DEFINES, ROM_CONFIG) { include(./3rdParty/3rdPartyRom.pri) }


include(./src/src.pri)

SOURCES += \
    main.cpp

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

# 指定uic文件目录
UI_DIR = $$OUT_PWD/release/ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
