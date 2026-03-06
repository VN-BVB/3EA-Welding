# Use Precompiled headers (PCH)
# CONFIG += precompile_header
PRECOMPILED_HEADER = src/stable.h

HEADERS += \
    src/stable.h
include( ./ui/ui.pri )
include( ./rail/rail.pri )
include( ./utils/utils.pri )
include( ./errorSave/errorSave.pri )
include( ./settingPara/settingPara.pri )
include( ./deepLearning/deepLearning.pri )
include( ./crashHandler/crashHandler.pri )
include( ./robotFactory/robotFactory.pri )
include( ./weldingSystem/weldingSystem.pri )
include( ./cameraFactory/cameraFactory.pri )
include( ./projectFactory/projectFactory.pri )
include( ./structLightCamera/structLightCamera.pri )
include( ./seamDetWithPointCloud/seamDetWithPointCloud.pri )
include( ./robotTrajectoryPlanning/robotTrajectoryPlanning.pri )
