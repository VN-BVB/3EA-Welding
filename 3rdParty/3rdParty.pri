# TensorRT
INCLUDEPATH += E:/TensorRT-8.4.1.5/include
LIBS += E:/TensorRT-8.4.1.5/lib/*.lib

# CUDA
INCLUDEPATH += 'C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6/include'
LIBS += 'C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6/lib/x64/*.lib'

# PCL1.9.1
include(D:/PCL1.9.1new/PCL191new.pri)

# OpenCV
INCLUDEPATH += E:/ProgramData/opencv/build/include/
INCLUDEPATH += E:/ProgramData/opencv/build/include/opencv2/
LIBS += -LE:/ProgramData/opencv/build/x64/vc15/lib/ -lopencv_world440

# Basler
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include/Base
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include/GenApi
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include/genapic
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include/pylon
INCLUDEPATH += E:/ProgramData/Basler/pylon6/Development/include/pylonc
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/GCBase_MD_VC141_v3_1_Basler_pylon.lib
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/GenApi_MD_VC141_v3_1_Basler_pylon.lib
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/PylonBase_v6_3.lib
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/PylonC.lib
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/PylonGUI_v6_3.lib
LIBS += E:/ProgramData/Basler/pylon6/Development/lib/x64/PylonUtility_v6_3.lib

# TJ_TCP
INCLUDEPATH += E:/ProgramData/TJ_Projector/TCP
LIBS += E:/ProgramData/TJ_Projector/TCP/TJSTProjectorApi.lib

# 引入plog日志库
INCLUDEPATH += ./3rdparty/plog/include

# 引入cereal序列化反序列化库
INCLUDEPATH += ./3rdparty/cereal/include

# 宝元机器人
INCLUDEPATH += ./3rdParty/BaoYuanRobot
LIBS += ./3rdParty/BaoYuanRobot/sc2_vc_x64.lib

#libmous
INCLUDEPATH +=./3rdparty/libmodbus/include
LIBS +=./3rdparty/libmodbus/X64/*.lib
