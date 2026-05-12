#ifndef TEST_H
#define TEST_H

#include <QObject>
#include <string>

#include "workpieceCoarseLocalization/src/fittingWorkpieceCoordinate/Fittingworkpiececoordinate.h"

class Test {
public:
    Test();

    workpieceBoxInWorld runWorkpieceCoarseLocalizationDemo(const std::string &enginePath = "./data/test/DL_models/best.engine",
                                                           const std::string &imageDir = "./data/test/data",
                                                           const std::string &outputDir = "./data/test/output/workpiece_coarse_localization");

    const workpieceBoxInWorld &getCoarseLocalizationDemoResult() const;

private:
    workpieceBoxInWorld coarseLocalizationDemoResult;
};

#endif  // TEST_H
