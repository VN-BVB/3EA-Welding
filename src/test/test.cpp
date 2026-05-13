#include "test.h"

#include <QDir>
#include <QFileInfo>
#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "deepLearning/segment/yolo11/Yolo11SegInference.h"

namespace {

segYolo11::ObjectYolo11Seg makeSegObject(const SegResult &result) {
    segYolo11::ObjectYolo11Seg obj;
    obj.rect = cv::Rect_<float>(result.topLeftX, result.topLeftY, result.bottomRightX - result.topLeftX, result.bottomRightY - result.topLeftY);
    obj.label = result.classId;
    obj.prob = result.score;
    obj.boxMask = result.maskOnly.clone();
    return obj;
}

cv::Mat inferResultImage(const std::vector<SegResult> &results, const cv::Mat &fallbackImage) {
    for (const auto &result : results) {
        if (!result.segRes.empty()) {
            return result.segRes.clone();
        }
    }
    return fallbackImage.clone();
}

void keepLargestConnectedComponent(cv::Mat &mask) {
    if (mask.empty()) {
        return;
    }

    cv::Mat grayMask;
    if (mask.channels() > 1) {
        cv::cvtColor(mask, grayMask, cv::COLOR_BGR2GRAY);
    } else {
        grayMask = mask.clone();
    }

    cv::threshold(grayMask, grayMask, 127, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(grayMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        mask = grayMask;
        return;
    }

    std::vector<double> externalArea(contours.size(), 0.0);
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        if (hierarchy[i][3] != -1) {
            continue;
        }

        externalArea[i] = cv::contourArea(contours[i]);
        for (int j = 0; j < static_cast<int>(contours.size()); ++j) {
            if (hierarchy[j][3] == i) {
                externalArea[i] -= cv::contourArea(contours[j]);
            }
        }
    }

    const auto maxIt = std::max_element(externalArea.begin(), externalArea.end());
    if (maxIt == externalArea.end() || *maxIt <= 0.0) {
        mask = grayMask;
        return;
    }

    const int maxAreaIndex = static_cast<int>(std::distance(externalArea.begin(), maxIt));
    cv::Mat filteredMask = grayMask.clone();
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        if (i != maxAreaIndex && hierarchy[i][3] == -1) {
            cv::drawContours(filteredMask, contours, i, cv::Scalar(0), cv::FILLED);
        }
    }

    mask = filteredMask;
}

void saveWorkpieceBoxSummary(const workpieceBoxInWorld &boxInfo, const std::vector<std::string> &sourceImagePaths, const std::string &outputDir) {
    const std::string yamlPath = outputDir + "/workpieceBoxInWorld.yml";
    cv::FileStorage fs(yamlPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to save workpieceBoxInWorld summary: " << yamlPath << std::endl;
        return;
    }

    fs << "workpiece_count" << static_cast<int>(boxInfo.workpieceInfoInWorld.size());
    fs << "workpieces" << "[";

    for (size_t i = 0; i < boxInfo.workpieceInfoInWorld.size(); ++i) {
        const auto &info = boxInfo.workpieceInfoInWorld[i];
        const auto &segObj = info.workpiece_weld_Obj.first;
        const std::string sourceImage = i < sourceImagePaths.size() ? sourceImagePaths[i] : std::string();

        fs << "{:";
        fs << "index" << static_cast<int>(i);
        fs << "source_image" << sourceImage;
        fs << "label" << segObj.label;
        fs << "prob" << static_cast<double>(segObj.prob);
        fs << "rect" << "[" << segObj.rect.x << segObj.rect.y << segObj.rect.width << segObj.rect.height << "]";
        fs << "mask_non_zero" << cv::countNonZero(segObj.boxMask);
        fs << "weld_object_count" << static_cast<int>(info.workpiece_weld_Obj.second.size());
        fs << "}";
    }

    fs << "]";
}

}  // namespace

Test::Test() {}

workpieceBoxInWorld Test::runWorkpieceCoarseLocalizationDemo(const std::string &enginePath, const std::string &imageDir,
                                                             const std::string &outputDir) {
    if (!QFileInfo(QString::fromStdString(enginePath)).isFile()) {
        throw std::runtime_error("Engine file does not exist: " + enginePath);
    }

    QDir().mkpath(QString::fromStdString(outputDir));

    std::vector<std::string> imagePaths;
    cv::glob(imageDir + "/*.png", imagePaths, false);
    std::sort(imagePaths.begin(), imagePaths.end());
    if (imagePaths.empty()) {
        throw std::runtime_error("No png images found in: " + imageDir);
    }

    std::shared_ptr<AbstractSegment> workpieceSegmentation = std::make_shared<Yolo11SegInference>();
    workpieceSegmentation->setEngine_path(enginePath);
    workpieceSegmentation->setClassNames({"WorkpieceCL"});
    workpieceSegmentation->setColors({
        {0,   114, 189},
        {217, 83,  25 },
        {237, 177, 32 },
        {126, 47,  142},
        {119, 172, 48 }
    });
    workpieceSegmentation->setScore_thres(0.25f);
    workpieceSegmentation->setIou_thres(0.25f);
    workpieceSegmentation->setSeg_channels(32);
    workpieceSegmentation->setSize(cv::Size(1024, 1024));
    workpieceSegmentation->initialization();

    coarseLocalizationDemoResult = workpieceBoxInWorld{};
    cvImagesCameraOri.clear();
    cvImagesWorkpieceSeg.clear();

    std::vector<std::string> sourceImagePathsByWorkpiece;
    int imageIndex = 0;

    for (const auto &imagePath : imagePaths) {
        cv::Mat image = cv::imread(imagePath);
        if (image.empty()) {
            std::cerr << "Skip empty image: " << imagePath << std::endl;
            continue;
        }

        cvImagesCameraOri.emplace_back(image.clone());

        std::vector<SegResult> segResults;
        cv::Mat inferImage = image.clone();
        workpieceSegmentation->inference(inferImage, segResults);

        std::sort(segResults.begin(), segResults.end(), [](const SegResult &a, const SegResult &b) { return a.topLeftX < b.topLeftX; });

        cv::Mat resultImage = inferResultImage(segResults, image);
        cvImagesWorkpieceSeg.emplace_back(resultImage.clone());

        const QFileInfo fileInfo(QString::fromStdString(imagePath));
        const std::string baseName = fileInfo.completeBaseName().toStdString();
        cv::imwrite(outputDir + "/" + baseName + "_seg_result.png", resultImage);

        int objectIndex = 0;
        for (const auto &segResult : segResults) {
            segYolo11::ObjectYolo11Seg obj = makeSegObject(segResult);
            keepLargestConnectedComponent(obj.boxMask);

            workpieceInfo info;
            info.cameraOriginalMat = image.clone();
            info.cameraSegMat = resultImage.clone();
            info.workpiece_weld_Obj = std::make_pair(obj, std::vector<det::Object>());
            coarseLocalizationDemoResult.workpieceInfoInWorld.emplace_back(std::move(info));
            sourceImagePathsByWorkpiece.emplace_back(imagePath);

            cv::imwrite(outputDir + "/" + baseName + "_cls_" + std::to_string(obj.label) + "_mask_" + std::to_string(objectIndex) + ".png",
                        obj.boxMask);
            ++objectIndex;
        }

        std::cout << "Image " << imageIndex << " detected workpieces: " << segResults.size() << std::endl;
        ++imageIndex;
    }

    FittingWorkpieceCoordinate fittingWorkpieceCoordinate;
    fittingWorkpieceCoordinate.computeBaseOffsetAndViewpointsFromMask(coarseLocalizationDemoResult);  //----------------------

    workpieceFinalInfoInWorld = coarseLocalizationDemoResult;
    saveWorkpieceBoxSummary(coarseLocalizationDemoResult, sourceImagePathsByWorkpiece, outputDir);

    std::cout << "Workpiece coarse localization demo finished. workpieceBoxInWorld size: " << coarseLocalizationDemoResult.workpieceInfoInWorld.size()
              << std::endl;

    return coarseLocalizationDemoResult;
}

const workpieceBoxInWorld &Test::getCoarseLocalizationDemoResult() const { return coarseLocalizationDemoResult; }
