# Use Precompiled headers (PCH)
# CONFIG += precompile_header
PRECOMPILED_HEADER = src/stable.h

HEADERS += \
    src/stable.h
include( ./ui/ui.pri )
include( ./rail/rail.pri )
include( ./utils/utils.pri )
include( ./deepLearning/deepLearning.pri )
include( ./crashHandler/crashHandler.pri )

