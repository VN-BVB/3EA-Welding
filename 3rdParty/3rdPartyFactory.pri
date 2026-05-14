# # 引入PCL点云库
include(D:\ProgramData\PCL1.9.1\PCL191new.pri)

# 引入OpenCV库
INCLUDEPATH += D:/ProgramData/opencv/build/include/
INCLUDEPATH += D:/ProgramData/opencv/build/include/opencv2/
LIBS += -LD:/ProgramData/opencv/build/x64/vc15/lib/ -lopencv_world440

# 引入Eigen矩阵运算库
INCLUDEPATH += D:/ProgramData\eigen-git-mirror-master


# # 引入Coin3d机器人显示库
# INCLUDEPATH += D:/ProgramData/Coin3d/include
# LIBS += D:\ProgramData\Coin3d\lib\*.lib
# 引入Basler相机库
INCLUDEPATH += $$quote(D:/ProgramData/Basler/pylon7/Development/include)
LIBS +=  $$quote(D:/ProgramData/Basler/pylon7/Development/lib/x64/*.lib)

# TJ_TCP
INCLUDEPATH += D:/ProgramData/TJ_Projector/TCP
LIBS += D:/ProgramData/TJ_Projector/TCP/TJSTProjectorApi.lib

# 引入plog日志库
INCLUDEPATH += ./3rdparty/plog/include

# 引入cereal序列化反序列化库
INCLUDEPATH += ./3rdparty/cereal/include

# 宝元机器人
INCLUDEPATH += ./3rdParty/BaoYuanRobot
LIBS += ./3rdParty/BaoYuanRobot/sc2_vc_x64.lib
#引入libmous
INCLUDEPATH +=./3rdparty/libmodbus/include
LIBS +=./3rdparty/libmodbus/X64/*.lib

# CUDA11.8
INCLUDEPATH += 'D:/ProgramData/CUDA/v11.8/include'
LIBS += 'D:/ProgramData/CUDA/v11.8/lib/x64/*.lib'
# TensorRT
INCLUDEPATH += D:\ProgramData\TensorRT-8.6.1.6/include
LIBS += D:\ProgramData\TensorRT-8.6.1.6/lib/*.lib
