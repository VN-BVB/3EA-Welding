# TensorRT
INCLUDEPATH += D:/TensorRT-8.6.1.6/include
LIBS += D:/TensorRT-8.6.1.6/lib/*.lib

# CUDA
INCLUDEPATH += 'D:/Program Files/CUDA/v11.8/include'
LIBS += 'D:/Program Files/CUDA/v11.8/lib/x64/*.lib'

# PCL1.9.1new
include(D:/PCL1.9.1new/PCL191new.pri)

# OpenCV
INCLUDEPATH += E:/ProgramData/opencv/build/include/
INCLUDEPATH += E:/ProgramData/opencv/build/include/opencv2/
LIBS += -LE:/ProgramData/opencv/build/x64/vc15/lib/ -lopencv_world440

# Basler pylon7
INCLUDEPATH += $$quote(D:/Basler/pylon7/Development/include)
LIBS += $$quote(D:/Basler/pylon7/Development/lib/x64/*.lib)

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
