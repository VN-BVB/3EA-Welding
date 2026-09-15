#include "YoloInference.h"

#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <map>

namespace {

struct CaptureInfo {
    std::string filename;
    double yAxisEncoderValue = std::numeric_limits<double>::quiet_NaN();

    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("filename", filename),
           cereal::make_nvp("yAxisEncoderValue", yAxisEncoderValue));
    }
};

std::map<std::string, double> loadCaptureYAxisEncoderMap(const std::string& folderPath) {
    std::map<std::string, double> captureYAxisMap;
    std::ifstream captureFile(folderPath + "/captures.json");
    if (!captureFile.is_open()) {
        return captureYAxisMap;
    }

    try {
        std::vector<CaptureInfo> captures;
        cereal::JSONInputArchive archive(captureFile);
        archive(cereal::make_nvp("captures", captures));
        for (const auto& capture : captures) {
            captureYAxisMap[capture.filename] = capture.yAxisEncoderValue;
        }
    } catch (const std::exception&) {
        captureYAxisMap.clear();
    }

    return captureYAxisMap;
}

std::string extractFilename(const std::string& filePath) {
    const size_t pos = filePath.find_last_of("/\\");
    if (pos == std::string::npos) {
        return filePath;
    }
    return filePath.substr(pos + 1);
}

}  // namespace

YoloSegInference::YoloSegInference() { initSegment(); }
YoloDetInference::YoloDetInference() { initObjectDetect(); }
void YoloSegInference::initSegment() {
    std::vector<std::string> classNames = {"Plate_Plate_F","TubeSide_Plate_F","Tube_Plate_F","Tube_Tube_F"};
    std::vector<std::vector<unsigned int>> colors = {
        {0  , 255, 0  },
        {128, 255, 0  },
        {255, 0,   128},
        {0  , 255, 128},
    };
    WorkpieceSegmentation = std::make_shared<Yolo11SegInference>();
    WorkpieceSegmentation->setEngine_path(CoarseSegEnginePath);
    WorkpieceSegmentation->setClassNames(classNames);  // 标签
    WorkpieceSegmentation->setColors(colors);          // 标签颜色
    WorkpieceSegmentation->setScore_thres(0.45f);      // 置信度
    WorkpieceSegmentation->setIou_thres(0.05f);        // 交并比
    WorkpieceSegmentation->setSeg_channels(32);        // 分割通道:seg_channels->num_classes = num_channels - seg_channels - 4;
    WorkpieceSegmentation->setSize(cv::Size(1024, 1024));

    WorkpieceSegmentation->initialization();  // 完成类的初始化 (读取模型文件, 移入显卡等)
}

void YoloDetInference::initObjectDetect() {
    std::vector<std::string> classNames = {"back_corner", "front_corner", "back_beam", "front_beam", "other_type"};
    std::vector<std::vector<unsigned int>> colors = {
        {0,   128, 255},
        {128, 255, 0  },
        {255, 0,   128},
        {0,   255, 128},
        {128, 0,   255}
    };
    weldsDetection = std::make_shared<Yolo11ObjDetInference>();
    weldsDetection->setEngine_path(CoarseObjDetEnginePath);
    weldsDetection->setClassNames(classNames);
    weldsDetection->setColors(colors);
    weldsDetection->setScore_thres(0.25f);
    weldsDetection->setIou_thres(0.2f);
    weldsDetection->setLabelsNum(5);
    weldsDetection->setSize(cv::Size{1024, 1024});

    weldsDetection->initialization();  // 完成类的初始化 (读取模型文件, 移入显卡等)
}

void YoloSegInference::inferSingleImage(cv::Mat& inputImage, double yAxisEncoderValue) {
    WorkpieceSegmentation->inference(inputImage, segRes);
    parseSegResults(segRes, objs, res);
    objs.erase(std::remove_if(objs.begin(), objs.end(), [](const segYolo11::ObjectYolo11Seg& obj) {
                   return cv::countNonZero(obj.boxMask) < 2500;  // 掩膜像素数少于 2500 的丢弃
               }), objs.end());
    emit sendAppendInferLog(QString(u8"图片%1检测工件数量:%2").arg(imgNum + 1).arg(objs.size()));
    detectedWp += objs.size();
    sortSegObjects(objs, sortAxis, sortOrder);
    for (auto& obj : objs) {
        colorizeAndDisplayConnectedComponents(obj.boxMask);
    }
    for (const auto& obj : objs) {
        workpieceInfo info;
        info.cameraOriginalMat = inputImage.clone();
        info.cameraSegMat = res.clone();
        info.workpiece_weld_Obj = std::make_pair(obj, std::vector<det::Object>());
        workpieceFinalInfoInWorld.workpieceInfoInWorld.push_back(info);
    }
    emit sendCoordinateTofit(objs, imgNum, yAxisEncoderValue);
    emit sendInferResultToMainWindow(res);
    if (saveEveryImg) {
        whenImageNeedToSave(res, segSavePath);
    }
    imgNum++;
    cvImagesWorkpieceSeg.emplace_back(res.clone());
    allSegResults.push_back(segRes);
}

void YoloSegInference::whenPathNeedToInfer(std::string path) {
    imgNum = 0;
    detectedWp = 0;
    allSegResults.clear();
    workpieceFinalInfoInWorld = workpieceBoxInWorld{};
    cvImagesCameraOri.clear();
    cvImagesWorkpieceSeg.clear();
    const auto captureYAxisMap = loadCaptureYAxisEncoderMap(path);
    auto inferImagesFromPaths = [&](const std::string& suffix) {
        std::vector<std::string> imagePathList;
        cv::glob(path + "/*" + suffix, imagePathList);
        for (auto& imgPath : imagePathList) {
            cv::Mat img = cv::imread(imgPath);
            if (img.empty()) continue;
            image = img.clone();
            cvImagesCameraOri.push_back(img.clone());
            const std::string filename = extractFilename(imgPath);
            const auto it = captureYAxisMap.find(filename);
            auto encoderVal = (it != captureYAxisMap.end()) ? it->second : std::numeric_limits<double>::quiet_NaN();
            inferSingleImage(img, encoderVal);
        }
    };
    inferImagesFromPaths(".jpg");
    inferImagesFromPaths(".bmp");

    emit sendAppendInferLog(QString(u8"共检测工件数量: %1").arg(detectedWp));
    qDebug() << "=== emit sendSignalTocalculate, workpiece count =" ;
    emit sendSignalTocalculate();

    // 遍历 allSegResults 发焊缝框
    std::vector<std::vector<std::array<double, 4>>> allBoxes;
    for (const auto& imgSegRes : allSegResults) {
        std::vector<std::array<double, 4>> perImgBoxes;
        for (const auto& seg : imgSegRes) {
            if (seg.classId == 0) {
                double cx = (seg.topLeftX + seg.bottomRightX) / 2.0;
                double cy = (seg.topLeftY + seg.bottomRightY) / 2.0;
                double w  = seg.bottomRightX - seg.topLeftX;
                double h  = seg.bottomRightY - seg.topLeftY;
                perImgBoxes.push_back({cx, cy, w, h});
            }
        }
        allBoxes.push_back(perImgBoxes);
    }
    qDebug() << "=== allBoxes size=" << allBoxes.size();
    emit sendWeldBoxInfo(allBoxes);
}

void YoloSegInference::whenImageNeedToInfer(std::vector<cv::Mat> cvImages) {
    imgNum = 0;
    detectedWp = 0;
    allSegResults.clear();
    workpieceFinalInfoInWorld = workpieceBoxInWorld{};
    cvImagesCameraOri.clear();
    cvImagesWorkpieceSeg.clear();
    for (auto& cvimage : cvImages) {
        cvImagesCameraOri.emplace_back(cvimage.clone());
        image = cvimage.clone();
        inferSingleImage(cvimage);
    }
    emit sendAppendInferLog(QString(u8"共检测工件数量:%1").arg(detectedWp));
    emit sendSignalTocalculate();

    // 遍历 allSegResults 发焊缝框
    std::vector<std::vector<std::array<double, 4>>> allBoxes;
    for (const auto& imgSegRes : allSegResults) {
        std::vector<std::array<double, 4>> perImgBoxes;
        for (const auto& seg : imgSegRes) {
            if (seg.classId == 0) {
                double cx = (seg.topLeftX + seg.bottomRightX) / 2.0;
                double cy = (seg.topLeftY + seg.bottomRightY) / 2.0;
                double w  = seg.bottomRightX - seg.topLeftX;
                double h  = seg.bottomRightY - seg.topLeftY;
                perImgBoxes.push_back({cx, cy, w, h});
            }
        }
        allBoxes.push_back(perImgBoxes);
    }
    emit sendWeldBoxInfo(allBoxes);
}

void YoloSegInference::whenImageNeedToSave(const cv::Mat& inferResult, const std::string& savePrefix) {
    if (savePrefix.empty()) {
        return;
    }
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream dateTimeStream;
    dateTimeStream << std::put_time(localTime, "%Y%m%d_%H%M%S");

    std::string savePath = savePrefix + "/camera" + std::to_string(imgNum) + "_" + dateTimeStream.str() + "_result.bmp";

    cv::imwrite(savePath, inferResult);
}

void YoloSegInference::parseSegResults(const std::vector<SegResult>& segResults, std::vector<segYolo11::ObjectYolo11Seg>& objs, cv::Mat& res) {
    objs.clear();
    res.release();
    for (const auto& result : segResults) {
        segYolo11::ObjectYolo11Seg obj;
        obj.rect = cv::Rect_<float>(result.topLeftX, result.topLeftY, result.bottomRightX - result.topLeftX, result.bottomRightY - result.topLeftY);
        obj.label = result.classId;
        obj.prob = result.score;
        obj.boxMask = result.maskOnly.clone();  // 深拷贝，防止引用错误

        objs.push_back(obj);
    }
    if (segResults.empty()) {
        res = image.clone();
    } else {
        for (const auto& seg : segResults) {
            if (!seg.segRes.empty()) {
                res = seg.segRes.clone();
                break;
            }
        }
    }
}

void YoloSegInference::sortSegObjects(std::vector<segYolo11::ObjectYolo11Seg>& objs, const std::string& axis, const std::string& order) {
    auto getCoord = [&](const segYolo11::ObjectYolo11Seg& obj) -> int {
        if (axis == "X" || axis == "x")
            return obj.rect.x;
        else if (axis == "Y" || axis == "y")
            return obj.rect.y;
        else
            throw std::invalid_argument("Invalid axis: must be 'X' or 'Y'");
    };

    bool ascending = (order == "up" || order == "UP");

    std::sort(objs.begin(), objs.end(), [&](const segYolo11::ObjectYolo11Seg& a, const segYolo11::ObjectYolo11Seg& b) {
        return ascending ? getCoord(a) < getCoord(b) : getCoord(a) > getCoord(b);
    });
}

void YoloSegInference::colorizeAndDisplayConnectedComponents(cv::Mat& mask) {
    cv::Mat binarizedMask;
    if (mask.channels() > 1) {
        cv::cvtColor(mask, binarizedMask, cv::COLOR_BGR2GRAY);  // 转为灰度图
    } else {
        binarizedMask = mask;
    }
    cv::threshold(binarizedMask, binarizedMask, 127, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(binarizedMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<int> externalArea(contours.size(), 0);
    for (int i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] == -1) {                         // 如果父轮廓为 -1，表示这是一个外部轮廓
            externalArea[i] = cv::contourArea(contours[i]);  // 计算外部连通域的像素数

            // 查找该外部轮廓内部的所有孔洞，并减去这些孔洞的面积
            for (int j = 0; j < contours.size(); ++j) {
                if (hierarchy[j][3] == i) {                           // 如果是外部连通域的子轮廓，即孔洞
                    externalArea[i] -= cv::contourArea(contours[j]);  // 减去孔洞的像素数
                }
            }
        }
    }
    // 找到面积最大的连通域的索引
    int maxAreaIndex = -1;
    int maxArea = 0;
    for (int i = 0; i < externalArea.size(); ++i) {
        if (externalArea[i] > maxArea) {
            maxArea = externalArea[i];
            maxAreaIndex = i;
        }
    }
    cv::Mat filteredMask = mask.clone();  // 先克隆原始掩膜图像
    for (int i = 0; i < contours.size(); ++i) {
        if (i != maxAreaIndex && hierarchy[i][3] == -1) {                            // 如果不是最大面积的外部连通域
            cv::drawContours(filteredMask, contours, i, cv::Scalar(0), cv::FILLED);  // 删除该连通域
        }
    }
    mask = filteredMask;
}

//----------------------------------------------------------------------------------------目标检测---------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void YoloDetInference::whenRecieveWpMaskInWorld(std::vector<cv::Point3d> worldCenters, std::vector<cv::Mat> worldMaskImages) {
    if (worldCenters.size() != worldMaskImages.size()) {
        std::cerr << "Error: worldCenters and worldMaskImages sizes do not match!" << std::endl;
        return;
    }
    maskWorldCenters = worldCenters;
    maskCanvasSizes.clear();
    std::vector<cv::Mat> expandedImages;

    for (size_t i = 0; i < worldMaskImages.size(); ++i) {
        cv::Mat image = worldMaskImages[i];
        int imageWidth = image.cols;
        int imageHeight = image.rows;
        cv::Mat resizedImage;
        if (!image.empty()) {
            cv::resize(image, resizedImage, cv::Size(imageWidth * AdjustWorkpieceResolution, imageHeight * AdjustWorkpieceResolution), 0, 0,
                       cv::INTER_CUBIC);
        }
        imageWidth = resizedImage.cols;
        imageHeight = resizedImage.rows;
        int canvasWidth = expandedWidth;
        int canvasHeight = expandedHeight;

        if (imageWidth > canvasWidth) {
            canvasWidth = static_cast<int>(imageWidth * expendAdaptability);
        }
        if (imageHeight > canvasHeight) {
            canvasHeight = static_cast<int>(imageHeight * expendAdaptability);
        }

        cv::Mat canvas = cv::Mat(canvasHeight, canvasWidth, CV_8UC3, cv::Scalar(255, 255, 255));  // 白底

        // 计算将原图放置到画布中心的起始位置
        int xOffset = (canvas.cols - imageWidth) / 2;
        int yOffset = (canvas.rows - imageHeight) / 2;
        resizedImage.copyTo(canvas(cv::Rect(xOffset, yOffset, imageWidth, imageHeight)));
        maskCanvasSizes.emplace_back(canvasWidth, canvasHeight);
        rectRotationAngleYolo = rectRotationAngle;
        if (workbenchInsertGroup == "negative") {
            rectRotationAngleYolo = rectRotationAngle + 180;
        }
        switch (rectRotationAngleYolo) {
            case 0:
                break;
            case 90:
                cv::rotate(canvas, canvas, cv::ROTATE_90_CLOCKWISE);
                break;
            case 180:
                cv::rotate(canvas, canvas, cv::ROTATE_180);
                break;
            case 270:
                cv::rotate(canvas, canvas, cv::ROTATE_90_COUNTERCLOCKWISE);
                break;
            default:
                PLOGE << "Unsupported rotation angle: " << rectRotationAngleYolo << ". Must be 0, 90, 180 or 270." << std::endl;
                break;
        }
        expandedImages.push_back(canvas);
    }

    // 调用 whenImageNeedToInfer()，传递扩展后的图像
    whenImageNeedToInfer(expandedImages);
}

void YoloDetInference::whenImageNeedToInfer(std::vector<cv::Mat> cvImages) {
    imgNum = 0;
    workpieceNum = 0;
    cvImagesWpMaskOri.clear();
    cvImagesWpMaskDet.clear();
    std::vector<std::vector<cv::Rect_<float>>> rect_Dets;
    for (auto& cvimage : cvImages) {
        std::vector<cv::Rect_<float>> rect_Det;
        image = cvimage.clone();
        inferSingleImage(cvimage);
        for (auto& obj : objs) {
            rect_Det.push_back(obj.rect);
        }
        rect_Dets.push_back(rect_Det);
        // 把当前原图和结果掩膜图一起保存
        workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld[workpieceNum++].workpiece_weld_Mask = std::make_pair(cvimage.clone(), res.clone());
        emit sendInferResultToMainWindow(res);
    }
    whenCoordinatesNeedToProceed(rect_Dets, maskWorldCenters);
}

void YoloDetInference::inferSingleImage(cv::Mat& inputImage) {
    if (inputImage.empty()) {
        PLOGE << "inputImage is empty, cannot infer!";
        return;
    }
    if (saveEveryImg) {
        whenImageNeedToSave(inputImage.clone(), detSavePath, "_Ori");
    }
    cvImagesWpMaskOri.emplace_back(inputImage.clone());

    if (workpieceNum < 0 || workpieceNum >= workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.size()) {
        PLOGE << "Invalid workpieceNum:" << workpieceNum;
        return;
    }

    weldsDetection->inference(inputImage, detRes);  // 深度学习推理
    parseDetResults(detRes, objs, res);
    if (saveEveryImg) {
        whenImageNeedToSave(res.clone(), detSavePath, "_Result");
    }
    cvImagesWpMaskDet.emplace_back(res.clone());

    //  按顺序将 det 对象分配到对应 pair 的第二项中
    workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld[workpieceNum].workpiece_weld_Obj.second = objs;
    imgNum++;
}
void YoloDetInference::whenImageNeedToSave(const cv::Mat& inferResult, const std::string& savePrefix, const std::string& tag) {
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    std::ostringstream dateTimeStream;
    dateTimeStream << std::put_time(localTime, "%Y%m%d_%H%M%S");

    std::string savePath = savePrefix + "/workpiece" + std::to_string(imgNum) + "_" + dateTimeStream.str() + "_" + tag + ".bmp";

    cv::imwrite(savePath, inferResult);
}
void YoloDetInference::parseDetResults(const std::vector<DetResult>& detResult, std::vector<det::Object>& objs, cv::Mat& res) {
    objs.clear();
    for (const auto& det : detResult) {
        det::Object obj;
        obj.label = det.classId;
        obj.prob = det.score;
        obj.rect = cv::Rect_<float>(det.topLeftX, det.topLeftY, det.bottomRightX - det.topLeftX, det.bottomRightY - det.topLeftY);
        objs.push_back(obj);
    }
    if (detResult.empty()) {
        res = image.clone();
    } else {
        for (const auto& det : detResult) {
            if (!det.detRes.empty()) {
                res = det.detRes.clone();  // 深拷贝
                break;
            } else {
                std::cerr << "emptyDetRes" << std::endl;
            }
        }
    }
}

void YoloDetInference::whenCoordinatesNeedToProceed(std::vector<std::vector<cv::Rect_<float>>> rect_Dets, std::vector<cv::Point3d> worldCenters) {
    std::vector<std::vector<std::array<double, 4>>> boxInfos;

    for (size_t i = 0; i < rect_Dets.size(); ++i) {
        const auto& rects = rect_Dets[i];
        std::vector<std::array<double, 4>> infos;
        const cv::Size canvasSize = i < maskCanvasSizes.size() ? maskCanvasSizes[i] : cv::Size(expandedWidth, expandedHeight);
        const double canvasW = static_cast<double>(canvasSize.width);
        const double canvasH = static_cast<double>(canvasSize.height);
        const cv::Point2d canvasCenter(canvasW / 2.0, canvasH / 2.0);

        for (const auto& rect : rects) {
            // 检测框中心点（旋转后图像）
            double cx = static_cast<double>(rect.x) + rect.width * 0.5;
            double cy = static_cast<double>(rect.y) + rect.height * 0.5;

            double origX = 0.0, origY = 0.0;
            double w = rect.width, h = rect.height;

            switch (rectRotationAngleYolo) {
                case 0:
                    origX = cx;
                    origY = cy;
                    break;
                case 90:
                    origX = cy;
                    origY = canvasW - cx;
                    std::swap(w, h);
                    break;
                case 180:
                    origX = canvasW - cx;
                    origY = canvasH - cy;
                    break;
                case 270:
                    origX = canvasH - cy;
                    origY = cx;
                    std::swap(w, h);
                    break;
                default:
                    std::cerr << "Unsupported rotation angle: " << rectRotationAngleYolo << std::endl;
                    continue;
            }

            // 计算相对位置（除以分辨率）
            cv::Point2d rel((origX - canvasCenter.x) / AdjustWorkpieceResolution, (origY - canvasCenter.y) / AdjustWorkpieceResolution);

            cv::Point2d worldCenter2d(worldCenters[i].x, worldCenters[i].y);

            cv::Point2d offset = CoordinateMapper::relativeMapToCoord(rel, worldCenter2d, g_coordMappingType);
            double offsetX = offset.x;
            double offsetY = offset.y;
            double normW = w / AdjustWorkpieceResolution;
            double normH = h / AdjustWorkpieceResolution;

            infos.push_back({offsetX, offsetY, normW, normH});
        }

        boxInfos.push_back(std::move(infos));
    }
    emit sendBoxInfoToDisplay(boxInfos);
}
