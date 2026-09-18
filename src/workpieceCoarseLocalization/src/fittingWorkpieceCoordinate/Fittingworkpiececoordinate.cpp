#include "FittingWorkpieceCoordinate.h"

#include "utils/common/CommonFunc.h"

std::vector<cameraConfig> cameraParameters;  // 相机参数数量

namespace {

bool isTrackAxisDirectionValid(const cv::Mat& dir) {
    return dir.rows == 3 && dir.cols == 1 && cv::norm(dir) > 1e-6;
}

double normalizedTrackAxisDot(const cv::Mat& lhs, const cv::Mat& rhs) {
    cv::Mat lhsNorm = lhs / cv::norm(lhs);
    cv::Mat rhsNorm = rhs / cv::norm(rhs);
    return lhsNorm.dot(rhsNorm);
}

}  // namespace

/**
 * @brief 构造函数，初始化时加载校准参数
 */
FittingWorkpieceCoordinate::FittingWorkpieceCoordinate() {
    loadCalibrationParameters(configFilePath);
    // ViewPlanningConfig::getInstance().initDefaultConfig();
    // ViewPlanningConfig::getInstance().writeConfig();
    ViewPlanningConfig::getInstance().readConfig();
}

/**
 * @brief 处理工件坐标拟合
 * @param objs 检测到的物体列表
 * @param imgNum 当前图像编号
 */
void FittingWorkpieceCoordinate::whenFittingWorkpieceCoordinate(std::vector<segYolo11::ObjectYolo11Seg> objs, int imgNum, double yAxisEncoderValue) {
    std::vector<cv::Point2d> topLeftPt2ds;
    std::vector<cv::Point2d> centerPt2ds;
    std::vector<cv::Point3d> topLeftWorldPoints;
    std::vector<cv::Point3d> centerWorldPoints;
    std::vector<int> validPixels;

    if (objs.empty()) {
        emit appendFittingLog(QString(u8"图片 %1 未检测到工件").arg(imgNum + 1));
        return;
    }

    // 遍历每个 Object
    for (const auto& obj : objs) {
        cv::Point2d topLeftPoint(obj.rect.x, obj.rect.y);  // 左上角坐标
        cv::Point2d centerPoint(obj.rect.x + obj.rect.width / 2.0, obj.rect.y + obj.rect.height / 2.0);
        //  计算有效像素数量（boxMask 中值为 1 的像素）
        int validPixel = cv::countNonZero(obj.boxMask);  // 统计 boxMask 中值为 1 的像素数
        topLeftPt2ds.push_back(topLeftPoint);
        centerPt2ds.push_back(centerPoint);
        validPixels.push_back(validPixel);
    }
    if (topLeftPt2ds.empty()) {
        // 如果没有有效的点，输出日志
        emit appendFittingLog(QString(u8"未在图像 %1 中找到有效点").arg(imgNum + 1));
        return;
    }
    topLeftWorldPoints = pixel2WorldCoordPoint(topLeftPt2ds, yAxisEncoderValue);
    centerWorldPoints = pixel2WorldCoordPoint(centerPt2ds, yAxisEncoderValue);
    if (topLeftWorldPoints.size() != objs.size() || centerWorldPoints.size() != objs.size()) {
        emit appendFittingLog(QString(u8"图片 %1 世界坐标数量异常").arg(imgNum + 1));
        return;
    }
    for (size_t i = 0; i < objs.size(); i++) {
        ObjectInfo objInfo;
        objInfo.object = objs[i];
        objInfo.pt3d = topLeftWorldPoints[i];
        objInfo.centerPt3d = centerWorldPoints[i];
        objInfo.validPixel = validPixels[i];
        objInfo.cameraIndex = imgNum;
        objInfo.yAxisEncoderValue = yAxisEncoderValue;
        allObjects.push_back(objInfo);
    }
}

/**
 * @brief 推理完成后的处理函数
 */
void FittingWorkpieceCoordinate::whenFinishInferrence() {
    categorizedObjects.clear();
    categoryWorldCenters.clear();
    categoryWorldLeftTopCenters.clear();
    filteredWorldCenters.clear();        // 人工筛选后的工件中心点（世界坐标系）
    filteredWorldTopLeftPoints.clear();  // 人工筛选后的工件左上角点（世界坐标系）
    worldMaskImages.clear();
    workpieceROIs.clear();
    selectedWorkpieces.clear();

    categorizedObjects = classifyWorkpieces(allObjects, threshold);
    removeSmallCategories(categorizedObjects);  // 实时拍摄使用，站点拍摄意义不大

    categoryWorldCenters = calculateCategoryCenters(categorizedObjects);
    qDebug() << "=== categorizedObjects=" << categorizedObjects.size() << " centers=" << categoryWorldCenters.size();
    // categoryWorldLeftTopCenters在displayDetectedWorkpieces中存储。
    displayDetectedWorkpieces(categorizedObjects, cvImagesWorkpieceSeg, categoryWorldCenters);
    for (const auto& worldCenter : categoryWorldCenters) {
        // PLOGD << "workbenchInsertGroup:" << workbenchInsertGroup;
        if ((workbenchInsertGroup == "positive" && worldCenter.y < 0) || (workbenchInsertGroup == "negative" && worldCenter.y >= 0)) {
            cv::Mat worldPt = (cv::Mat_<double>(3, 1) << worldCenter.x, worldCenter.y, 1);
            cv::Mat pixelPt = canvasMat * worldPt;  // 变换回像素坐标系
            double x = pixelPt.at<double>(0, 0);
            double y = pixelPt.at<double>(1, 0);
            handleClickEvent(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
        }
    }
    emit sendUpdateInferedCameraImgNum();

    allObjects.clear();
}

std::vector<cv::Point3d> FittingWorkpieceCoordinate::pixel2WorldCoordPoint(std::vector<cv::Point2d>& Pt2ds, double yAxisEncoderValue) {
    if (Pt2ds.empty()) return {};
    std::vector<cv::Point3d> cameraPointsXYZ;

    Point2dto3d(cameraParameters[0].globalPlane, cameraParameters[0].cameraMatrix, cameraParameters[0].distCoeffs,
                Pt2ds, cameraPointsXYZ);  // 输出相机坐标系下在对应平面上的映射
    // std::cout<<"cameraPointsXYZ"<<cameraPointsXYZ<<std::endl;
    std::vector<cv::Point3d> worldPoints = transformCameraToBase(cameraPointsXYZ, cameraParameters[0].cameraToBaseMatrix);
    for (size_t i = 0; i < worldPoints.size(); ++i) {
        cv::Point3d before = worldPoints[i];
        worldPoints[i] = applyXAxisEncoderOffset(worldPoints[i], yAxisEncoderValue);
        std::cout << "补偿前: " << before << " 补偿后: " << worldPoints[i]
                  << " 编码器:" << yAxisEncoderValue << " ref:" << cameraParameters[0].yAxisReferenceEncoderValue <<
            std::endl;
    }
    // std::cout<<"cameraParameters[cameraNumber].extrinsicMatrix"<<cameraParameters[cameraNumber].extrinsicMatrix<<std::endl;
    return worldPoints;
}

cv::Point3d FittingWorkpieceCoordinate::applyXAxisEncoderOffset(const cv::Point3d& pt, double yAxisEncoderValue) {
    std::cout << "applyXAxisEncoderOffset called, encoder=" << yAxisEncoderValue << " ref=" << cameraParameters[0].yAxisReferenceEncoderValue << std::endl;
    if (!std::isfinite(yAxisEncoderValue)) {
        return pt;
    }
    std::cout << "输入配对编码器的点：" << "X:" << pt.x << "Y:" << pt.y << "Z:" << pt.z << std::endl;

    const double delta = cameraParameters[0].yAxisReferenceEncoderValue - yAxisEncoderValue;
    if (std::abs(delta) < 1e-9) {
        return pt;
    }

    if (cameraParameters[0].yAxisTrackDirection.rows == 3 && cameraParameters[0].yAxisTrackDirection.cols == 1) {
        cv::Mat direction = cameraParameters[0].yAxisTrackDirection.clone();
        const double norm = cv::norm(direction);
        if (norm > 1e-9) {
            direction /= norm;
            // 透视修正：编码器每走1mm，rawY实际变化1.794mm
            // 先预估一个值，跑一次看残留漂移再调整
            const double perspectiveScale = 1.8;
            cv::Point3d compensated(
                pt.x + delta * direction.at<double>(0, 0),
                pt.y + delta * perspectiveScale,      // Y用透视比例
                pt.z + delta * direction.at<double>(2, 0));
            return compensated;
        }
    }
    return cv::Point3d(pt.x + delta, pt.y, pt.z);
}

/**
 * @brief 将2D点转换为3D点
 * @param plane 平面方程参数
 * @param cameraMatrix 相机内参矩阵
 * @param distCoeffs 畸变系数
 * @param Pt2ds 输入2D点集
 * @param Pt3ds 输出3D点集
 */
void FittingWorkpieceCoordinate::Point2dto3d(std::vector<double> plane, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Point2d>& Pt2ds,
                                             std::vector<cv::Point3d>& Pt3ds) {
    // Q_UNUSED(distCoeffs)

    double A = -(plane[0] / plane[3]), B = -(plane[1] / plane[3]), C = -(plane[2] / plane[3]);
    std::vector<cv::Point2d> undistortedPts;
    // 使用cv::undistortPoints进行去畸变

    cv::undistortPoints(Pt2ds, undistortedPts, cameraMatrix, distCoeffs);

    for (int i = 0; i < undistortedPts.size(); ++i) {
        double x1 = undistortedPts[i].x, y1 = undistortedPts[i].y;
        cv::Point3d pt;
        pt.z = (1 / (A * x1 + B * y1 + C));
        pt.x = x1 * pt.z;
        pt.y = y1 * pt.z;
        std::cout << "相机坐标系下3D点:" << pt << std::endl;
        Pt3ds.push_back(pt);
    }
    // for (const auto &a :Pt3ds ){
    //     std::cout<<"Point2dto3d Camera:"<<a<<std::endl;
    // }
}

std::vector<cv::Point3d> FittingWorkpieceCoordinate::transformCameraToBase(const std::vector<cv::Point3d>& cameraPoints,
                                                                           const cv::Mat& transformationMatrix) {
    std::vector<cv::Point3d> basePoints;

    // 确保变换矩阵是 4x4 的矩阵
    if (transformationMatrix.rows != 4 || transformationMatrix.cols != 4) {
        std::cerr << "Error: Transformation matrix must be 4x4!" << std::endl;
        return basePoints;
    }

    // 对每个相机坐标系下的点进行转换
    for (const auto& cameraPoint : cameraPoints) {
        // 将相机点转换为 4x1 向量，使用齐次坐标表示
        std::cout << "输入转换的相机坐标系下3D点：" << cameraPoint << std::endl;
        cv::Mat cameraPointMat = (cv::Mat_<double>(4, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z, 1);

        // 使用 4x4 变换矩阵进行转换
        cv::Mat transformedPointMat = transformationMatrix * cameraPointMat;
        std::cout << "手眼转换矩阵:" << transformationMatrix << std::endl;

        // 提取变换后的 3D 点 (x, y, z)，忽略齐次坐标的 w 分量
        cv::Point3d basePoint(transformedPointMat.at<double>(0), transformedPointMat.at<double>(1), transformedPointMat.at<double>(2));
        std::cout << "转换到基坐标系下的3D点:" << basePoint << std::endl;

        // 将变换后的点添加到结果中
        basePoints.push_back(basePoint);
    }

    return basePoints;
}
void FittingWorkpieceCoordinate::saveAllObjectsToFile(std::string filePath) {
    std::ofstream outputFile(filePath, std::ios::trunc);  // 覆盖模式trunc打开文件，追加模式为app
    if (outputFile.is_open()) {
        outputFile << "Classification Results:\n\n";

        for (size_t i = 0; i < categorizedObjects.size(); ++i) {
            outputFile << "Category " << i + 1 << ":\n";
            for (size_t j = 0; j < categorizedObjects[i].size(); ++j) {
                outputFile << "ID:" << j << " x " << categorizedObjects[i][j].pt3d.x << "y " << categorizedObjects[i][j].pt3d.y << "z "
                           << categorizedObjects[i][j].pt3d.z << "Image X:" << categorizedObjects[i][j].object.rect.x
                           << "Image y:" << categorizedObjects[i][j].object.rect.y;

                // 确保 validPixels 长度匹配
                outputFile << "Valid Pixels: " << categorizedObjects[i][j].validPixel << std::endl;
            }
        }
        outputFile.close();
    } else {
        std::cerr << "Failed to open file for writing!" << std::endl;
    }
}

// 计算两点之间的欧几里得距离
double FittingWorkpieceCoordinate::calculateDistance(const cv::Point3d& p1, const cv::Point3d& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// 计算类别的平均坐标（质心）
cv::Point3d FittingWorkpieceCoordinate::computeCentroid(const std::vector<ObjectInfo>& group) {
    double sumX = 0, sumY = 0, sumZ = 0;
    for (const auto& obj : group) {
        sumX += obj.pt3d.x;
        sumY += obj.pt3d.y;
        sumZ += obj.pt3d.z;
    }
    int n = group.size();
    return cv::Point3d(sumX / n, sumY / n, sumZ / n);
}

// 进行工件分类
std::vector<std::vector<ObjectInfo>> FittingWorkpieceCoordinate::classifyWorkpieces(const std::vector<ObjectInfo>&
                                                                                        allObjects, double threshold) {
    std::vector<cv::Point3d> categoryLeftPts;

    for (const auto& obj : allObjects) {
        bool foundGroup = false;
        for (size_t i = 0; i < categoryLeftPts.size(); ++i) {
            double dist = calculateDistance(obj.pt3d, categoryLeftPts[i]);
            if (dist < threshold) {
                categorizedObjects[i].push_back(obj);
                categoryLeftPts[i] = computeCentroid(categorizedObjects[i]);
                foundGroup = true;
                break;
            }
        }
        if (!foundGroup) {
            categorizedObjects.push_back({obj});
            categoryLeftPts.push_back(obj.pt3d);
        }
    }
    return categorizedObjects;
}

void FittingWorkpieceCoordinate::removeSmallCategories(std::vector<std::vector<ObjectInfo>>& categorizedObjects) {
    // 使用 std::remove_if 和 erase 移除大小小于 一定数量的 的类别
    categorizedObjects.erase(std::remove_if(categorizedObjects.begin(), categorizedObjects.end(),
                                            [](const std::vector<ObjectInfo>& category) {
                                                return category.size() < 1;  // “1”为一定数量，此处为站点拍摄，取1无实际价值
                                            }),
                             categorizedObjects.end());
    // std::string filePath = "./data/result/FittingWorkpieceCoordinate/classified_workpiecesRemove.txt";
    // saveAllObjectsToFile(filePath);
}

std::vector<cv::Point3d> FittingWorkpieceCoordinate::calculateCategoryCenters(std::vector<std::vector<ObjectInfo>>& categorizedObjects) {
    for (auto& category : categorizedObjects) {
        // 按 validPixel 从大到小排序，选出 validPixel 最多的三项
        std::sort(category.begin(), category.end(), [](const ObjectInfo& a, const ObjectInfo& b) {
            return a.validPixel > b.validPixel;  // 降序排列
        });
        // std::string filePath ="./data/result/FittingWorkpieceCoordinate/object_testWorldPoints2.txt";
        // saveAllObjectsToFile(filePath);
        // 选择 validPixel 最多的前三项
        int count = std::min(maxPixelCount, (int)category.size());  // 至少选择 1 项，但不超过 3 项
        cv::Point3d sumCenter(0, 0, 0);                             // 用来累加平均中心坐标

        for (int i = 0; i < count; ++i) {
            sumCenter += category[i].centerPt3d;  // 直接使用带位姿补偿的中心点
        }

        // 计算该类别的平均中心坐标
        cv::Point3d averageCenter = sumCenter / count;
        std::cout<<"averageCenter:"<<averageCenter<<std::endl;

        categoryWorldCenters.push_back(averageCenter);  // 将结果加入类别中心坐标集合
    }
    // std::cout<<"test size "<<categoryCenters3d.size();

    // std::string filePath = "./data/result/FittingWorkpieceCoordinate/object_finalWorldPointsInAuto.txt";
    // std::ofstream outputFile(filePath, std::ios::trunc);  // 覆盖模式trunc打开文件，追加模式为app
    // if (outputFile.is_open()) {
    //     // 遍历 worldPoints
    //     for (size_t i = 0; i < categoryWorldCenters.size(); i++) {
    //         outputFile << "workpiece " << i + 1 << ": ";  // 工件编号从 1 开始
    //         outputFile << "categoryCenters3d: ("
    //                    << categoryWorldCenters[i].x/*+1300*/  << " , "
    //                    << categoryWorldCenters[i].y << " , "
    //                    << categoryWorldCenters[i].z << "), "
    //                    <<std::endl;
    //     }
    //     outputFile.close();  // 关闭文件
    //     std::cout << "工件信息已保存至 " << filePath << std::endl;
    // } else {
    //     std::cerr << "Failed to open file for writing!" << std::endl;
    // }
    // for (const auto& pt : finalWorldPoints) {
    //     std::cout <<"平面方程拟合"<< "(" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
    // }
    return categoryWorldCenters;
}

/**
 * @brief 加载校准参数
 * @param filename 参数文件路径
 */
void FittingWorkpieceCoordinate::loadCalibrationParameters(const std::string& filename) {
    cv::Mat cameraMatrixRead = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distCoeffsRead = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<double> planeRead;
    cv::Mat cameraToBaseMatrixRead;
    cv::Mat xAxisTrackDir, yAxisTrackDir, zAxisTrackDir;

    std::ifstream file(filename);
    if (!file.is_open() || file.peek() == std::ifstream::traits_type::eof()) {
        std::cerr << "File is empty or failed to open." << std::endl;
        return;
    }

    cereal::JSONInputArchive inputArchive(file);

    try {
        CoarseLocalizationMatrix calibResult;
        inputArchive(cereal::make_nvp("CalibrationResult", calibResult));

        calibResult.transferData(cameraMatrixRead, distCoeffsRead, planeRead,
                                 cameraToBaseMatrixRead,
                                 xAxisTrackDir, yAxisTrackDir, zAxisTrackDir);

        cameraParameters.resize(1);
        cameraParameters[0].cameraMatrix      = cameraMatrixRead;
        cameraParameters[0].distCoeffs         = distCoeffsRead;
        cameraParameters[0].globalPlane        = planeRead;
        cameraParameters[0].cameraToBaseMatrix = cameraToBaseMatrixRead;
        cameraParameters[0].xAxisTrackDirection = xAxisTrackDir;
        cameraParameters[0].yAxisTrackDirection = yAxisTrackDir;
        cameraParameters[0].zAxisTrackDirection = zAxisTrackDir;
        cameraParameters[0].xAxisReferenceEncoderValue = calibResult.xAxisReferenceEncoderValue;
        cameraParameters[0].yAxisReferenceEncoderValue = calibResult.yAxisReferenceEncoderValue;
        applyTrackCompensation = true;
        const bool xValid = isTrackAxisDirectionValid(xAxisTrackDir);
        const bool yValid = isTrackAxisDirectionValid(yAxisTrackDir);
        const bool zValid = isTrackAxisDirectionValid(zAxisTrackDir);
        bool axesIndependent = false;
        if (xValid && yValid && zValid) {
            const double xyDot = std::abs(normalizedTrackAxisDot(xAxisTrackDir, yAxisTrackDir));
            const double xzDot = std::abs(normalizedTrackAxisDot(xAxisTrackDir, zAxisTrackDir));
            const double yzDot = std::abs(normalizedTrackAxisDot(yAxisTrackDir, zAxisTrackDir));
            axesIndependent = xyDot < 0.95 && xzDot < 0.95 && yzDot < 0.95;
        }
        if (!(xValid && yValid && zValid && axesIndependent)) {
            applyTrackCompensation = false;
            PLOGW << "Track compensation disabled because axis directions are invalid or nearly parallel.";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading calibration: " << e.what() << std::endl;
    }

    // 调试信息
    QString message = "Complete to load data/config/workpiece_localization_calib.json";
    emit appendFittingLog(message);

}

void FittingWorkpieceCoordinate::drawGridAndAxes(cv::Mat& railMap) {
    railMapRotated(railMap, railMapRotationAngle);
    int roPixelRow = railMap.rows;
    int roPixelCol = railMap.cols;
    int originX = 0;
    int originY = 0;
    int scaleX = static_cast<int>(canvasMat.at<double>(0, 0));  // a
    int scaleY = static_cast<int>(canvasMat.at<double>(1, 1));  // d

    if (scaleX == 0 || scaleY == 0) {
        scaleX = canvasMat.at<double>(0, 1);
        scaleY = canvasMat.at<double>(1, 0);
    }
    // 设置 canvasMat 中定义的原点位置
    switch (railMapRotationAngle) {
        case 0:
            originX = static_cast<int>(canvasMat.at<double>(0, 2));  // tx
            originY = static_cast<int>(canvasMat.at<double>(1, 2));  // ty
            break;
        case 90:
            originX = roPixelCol - static_cast<int>(canvasMat.at<double>(1, 2)) - 1;
            originY = static_cast<int>(canvasMat.at<double>(0, 2));
            scaleY *= -1;
            break;
        case 180:
            originX = roPixelCol - static_cast<int>(canvasMat.at<double>(0, 2)) - 1;
            originY = roPixelRow - static_cast<int>(canvasMat.at<double>(1, 2)) - 1;
            scaleX *= -1;
            scaleY *= -1;
            break;
        case 270:
            originX = static_cast<int>(canvasMat.at<double>(1, 2));
            originY = roPixelRow - static_cast<int>(canvasMat.at<double>(0, 2)) - 1;
            scaleX *= -1;
    }

    // 绘制 X 轴（水平线）
    cv::line(railMap, cv::Point(0, originY), cv::Point(roPixelCol, originY), axisColor, axisThickness);
    // 绘制 Y 轴
    cv::line(railMap, cv::Point(originX, 0), cv::Point(originX, roPixelRow), axisColor, axisThickness);

    // 绘制竖直网格线（x方向）
    for (int x = originX % gridSpacingX; x < roPixelCol; x += gridSpacingX) {
        cv::line(railMap, cv::Point(x, 0), cv::Point(x, roPixelRow), cv::Scalar(200, 200, 200), 1);
    }

    // 绘制水平网格线
    for (int y = originY % gridSpacingY; y < roPixelRow; y += gridSpacingY) {
        cv::line(railMap, cv::Point(0, y), cv::Point(roPixelCol, y), cv::Scalar(200, 200, 200), 1);
    }

    // 添加坐标标签
    for (int x = originX % gridSpacingX; x < roPixelCol; x += gridSpacingX * 3) {
        int label = ((x - originX) / scaleX);
        cv::putText(railMap, std::to_string(label), cv::Point(x + 2, originY - 5), cv::FONT_HERSHEY_SIMPLEX, 2, axisColor, 7);
    }

    for (int y = originY % gridSpacingY; y < roPixelRow; y += gridSpacingY) {
        int label = ((y - originY) / scaleY);
        cv::putText(railMap, std::to_string(label), cv::Point(originX + 5, y), cv::FONT_HERSHEY_SIMPLEX, 2, axisColor, 7);
    }
    railMapRotated(railMap, 360 - railMapRotationAngle);
}
// 绘制工件图像
void FittingWorkpieceCoordinate::drawDetectedWorkpieces(cv::Mat& railMap, const cv::Mat& resizedImage, cv::Point3d& worldCenter, int categoryIdx) {
    // 1300
    cv::Mat centerPoint = (cv::Mat_<double>(3, 1) << worldCenter.x, worldCenter.y, 1);
    std::cout<<"转换成像素坐标前的世界坐标："<<centerPoint<<std::endl;
    cv::Mat centerResult = canvasMat * centerPoint;
    std::cout<<"转换成像素坐标后的世界坐标："<<centerResult<<std::endl;
    double railMapX = centerResult.at<double>(0, 0);
    double railMapY = centerResult.at<double>(1, 0);
    // 确保抠出来的图像中心与 worldCenter 对齐
    //找到左上角坐标以确定有没有超出边界
    int offsetX = static_cast<int>(railMapX - resizedImage.cols / 2);
    int offsetY = static_cast<int>(railMapY - resizedImage.rows / 2);
    //看目标矩形框和画布上的有效范围有没有重合
    const cv::Rect targetRect(offsetX, offsetY, resizedImage.cols, resizedImage.rows);
    const cv::Rect canvasRect(0, 0, railMap.cols, railMap.rows);
    //计算重合矩形框
    cv::Rect roi = targetRect & canvasRect;
    //如果有重合就把重合部分copyto长画布上
    if (roi.area() > 0) {
        //把重合部分的矩形框算出来
        const cv::Rect srcRoi(roi.x - targetRect.x, roi.y - targetRect.y, roi.width, roi.height);
        resizedImage(srcRoi).copyTo(railMap(roi));
        //在长画布上画出可视化矩形框
        cv::rectangle(railMap, roi, roi == targetRect ? correctColor : warningColor,
                      roi == targetRect ? correctLineThickness : warningLineThickness);
    }
    // 将 roi 和类别索引存入全局变量
    if (roi.area() > 0) {
        workpieceROIs.push_back(std::make_pair(roi, categoryIdx));
    }
    // 标注工件中心点
    cv::circle(railMap, cv::Point(static_cast<int>(railMapX), static_cast<int>(railMapY)), 4, cv::Scalar(255, 0, 0), -1);
    // std::string text = "(" + std::to_string(worldCenter.x) + ", " + std::to_string(worldCenter.y) + ", " +
    // std::to_string(worldCenter.z) + ")"; cv::Point textPosition = cv::Point(static_cast<int>(railMapX) - resizedImage.cols
    // / 2, static_cast<int>(railMapY) + resizedImage.rows / 2 + 30); cv::putText(railMap, text, textPosition,
    // cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    if (roi.area() > 0 && std::find(selectedWorkpieces.begin(), selectedWorkpieces.end(), categoryIdx) != selectedWorkpieces.end()) {
        cv::rectangle(railMap, roi, deleteColor, deleteLineThickness);
    }
}

void FittingWorkpieceCoordinate::displayDetectedWorkpieces(const std::vector<std::vector<ObjectInfo>>& categorizedObjects,
                                                           const std::vector<cv::Mat>& cvImagesToDisplay,
                                                           const std::vector<cv::Point3d>& categoryCenters3d) {
    railMap = cv::Mat(pixelRow, pixelCol, CV_8UC3, cv::Scalar(255, 255, 255));  // 确保白色背景
    // 1300

    // cv::imshow("Initial White Canvas", railMap); // 显示初始白色画布
    // cv::waitKey(0);

    // 获取图像路径列表
    // std::vector<std::string> imagePathList;
    // cv::glob(imagePath + "/*.bmp", imagePathList);  // 获取所有图像路径
    // 遍历所有类别
    for (size_t categoryIdx = 0; categoryIdx < categorizedObjects.size(); ++categoryIdx) {
        const auto& category = categorizedObjects[categoryIdx];
        if (category.empty()) continue;

        // 选取前个 pt3d 计算均值作为左上角坐标
        cv::Point3d worldTopLeft(0, 0, 0);
        int count = std::min(maxPixelCount, static_cast<int>(category.size()));
        for (int i = 0; i < count; ++i) {
            worldTopLeft += category[i].pt3d;
        }
        worldTopLeft *= (1.0 / count);

        // 将左上角坐标存入类变量中
        categoryWorldLeftTopCenters.push_back(worldTopLeft);
        std::cout<<"categoryWorldLeftTopCenters"<<std::endl<<worldTopLeft<<std::endl;

        // 使用 categoryCenters3d 作为中心坐标
        cv::Point3d worldCenter = categoryCenters3d[categoryIdx];

        // 选取 validPixel 最大的对象
        const ObjectInfo& bestObject = category.front();
        int imagNum = bestObject.cameraIndex;

        if (imagNum >= 0 && imagNum < cvImagesToDisplay.size()) {
            cv::Mat image = cvImagesToDisplay[imagNum];

            cv::Rect rect = bestObject.object.rect;
            cv::Mat boxMask = bestObject.object.boxMask;  // 掩膜矩阵
            // 扩大区域显示母材
            const int expandPixels = 50;
            int ex1 = std::max(0, rect.x - expandPixels);
            int ey1 = std::max(0, rect.y - expandPixels);
            int ex2 = std::min(image.cols - 1, rect.x + rect.width + expandPixels);
            int ey2 = std::min(image.rows - 1, rect.y + rect.height + expandPixels);
            rect = cv::Rect(ex1, ey1, ex2 - ex1, ey2 - ey1);

            cv::Mat maskRegion = image(rect).clone();  // 扩大后的原图区域（含母材）

            // cv::Mat objectRegion = image(rect).clone();  // 原图rect区域
            // objectRegion.copyTo(maskRegion, boxMask);    // 将原图rect区域中的掩膜部分复制给maskRegion
            // cv::imshow("Extracted Workpiece", maskRegion); // 显示掩膜下的工件图像（像素坐标系）
            // cv::waitKey(0);

            // // 计算缩放比例
            // double worldWidth = 2 * std::abs(worldCenter.x - worldTopLeft.x);
            // double worldHeight = 2 * std::abs(worldCenter.y - worldTopLeft.y);

            // 计算缩放后的大小
            // double scaleX = worldWidth / rect.width;
            // double scaleY = worldHeight / rect.height;
            // 计算世界坐标系中的对角线长度
            double worldDiagonal = std::sqrt(std::pow(worldCenter.x - worldTopLeft.x, 2) + std::pow(worldCenter.y - worldTopLeft.y, 2));
            // // 计算图像的对角线长度
            double imageDiagonal = std::sqrt(std::pow(rect.width, 2) + std::pow(rect.height, 2)) / 2.0;

            // 计算缩放比例
            double scaleX = worldDiagonal / imageDiagonal;
            double scaleY = worldDiagonal / imageDiagonal;
            cv::Size newSize(maskRegion.cols * scaleX, maskRegion.rows * scaleY);
            // 缩放图像
            cv::Mat resizedImage;
            // 插值放缩掩膜
            // cv::resize(rotatedImage, resizedImage, newSize);//使用双线性插值
            // cv::resize(rotatedImage, resizedImage, newSize, 0, 0, cv::INTER_CUBIC);  // 使用立方插值
            cv::resize(maskRegion, resizedImage, newSize, 0, 0, cv::INTER_LANCZOS4);  // Lanczos

            // 计算原图 rect 的角度
            cv::Mat R = cameraParameters[0].cameraToBaseMatrix(cv::Range(0, 3), cv::Range(0, 3));
            double angleDifference;
            // 计算偏航角（yaw）
            angleDifference = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            angleDifference = angleDifference * 180.0 / CV_PI;
            angleDifference += 270.0;
            // angleDifference = 225;
            // 获取旋转矩阵
            cv::Point2f center(resizedImage.cols / 2, resizedImage.rows / 2);                // 图像的中心
            cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angleDifference, 1.0);  // 获取旋转矩阵
            // 计算旋转后的图像尺寸
            cv::Rect bbox = cv::RotatedRect(center, resizedImage.size(), angleDifference).boundingRect();

            // 创建一个足够大的白色背景，以容纳旋转后的图像
            cv::Mat rotatedImage = cv::Mat(bbox.size(), CV_8UC3, cv::Scalar(255, 255, 255));  // 新背景
            rotationMatrix.at<double>(0, 2) += (bbox.width - resizedImage.cols) / 2;          // 水平平移
            rotationMatrix.at<double>(1, 2) += (bbox.height - resizedImage.rows) / 2;         // 垂直平移
            // https://blog.csdn.net/u013105205/article/details/78826789

            // 将旋转后的图像放入新的背景中，确保图像居中
            cv::warpAffine(resizedImage, rotatedImage, rotationMatrix, bbox.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT,
                           cv::Scalar(255, 255, 255));  // 旋转图像

            // // 显示旋转后的图像
            // cv::imshow("maskRegion Image", maskRegion);
            // cv::waitKey(0);
            // cv::imshow("resizedImage Image", resizedImage);
            // cv::waitKey(0);
            // cv::imshow("Rotated Image", rotatedImage);
            // cv::waitKey(0);
            worldMaskImages.push_back(rotatedImage);  // 世界坐标系下的掩模图下
            drawDetectedWorkpieces(railMap, rotatedImage, worldCenter, categoryIdx);
        }
    }
    drawGridAndAxes(railMap);
    cv::Mat railMapDisplay = railMap.clone();
    railMapRotated(railMapDisplay, railMapRotationAngle);

    emit sendWorkpieceResultToMainWindow(railMapDisplay);
    // emit sendWorkpieceMaskImageInWorld(categoryCenters3d,worldMaskImages);
    // //发送为筛选前的工件在世界坐标系下的中心点和工件掩膜图像
    cv::Mat railMapRGB;
    cv::cvtColor(railMap, railMapRGB, cv::COLOR_BGR2RGB);
    cv::imwrite("./data/workpieceCoaLoc/FinalRailMap/detected_workpieces.jpg", railMapRGB);

    // 显示最终的画布
    // cv::imshow("Final Workpiece Map", railMap);
    // cv::waitKey(0);
}
void FittingWorkpieceCoordinate::handleClickEvent(int x, int y) {
    cv::Point clickPoint(x, y);
    cv::Rect closestROI;
    int closestCategoryIdx = -1;
    double minDistance = std::numeric_limits<double>::max();

    // 遍历所有 ROI，找出包含点击点且中心最近的那个
    for (const auto& roiPair : workpieceROIs) {
        const cv::Rect& roi = roiPair.first;
        int categoryIdx = roiPair.second;

        if (roi.contains(clickPoint)) {
            // 计算矩形中心
            cv::Point roiCenter(roi.x + roi.width / 2, roi.y + roi.height / 2);
            double distance = cv::norm(clickPoint - roiCenter);

            if (distance < minDistance) {
                minDistance = distance;
                closestROI = roi;
                closestCategoryIdx = categoryIdx;
            }
        }
    }

    // 如果有匹配 ROI，进行标记或取消标记
    if (closestCategoryIdx != -1) {
        auto it = std::find(selectedWorkpieces.begin(), selectedWorkpieces.end(), closestCategoryIdx);
        if (it == selectedWorkpieces.end()) {
            selectedWorkpieces.push_back(closestCategoryIdx);  // 添加选中
        } else {
            selectedWorkpieces.erase(std::remove(selectedWorkpieces.begin(), selectedWorkpieces.end(), closestCategoryIdx), selectedWorkpieces.end());
        }

        // 更新显示
        displayDetectedWorkpieces(categorizedObjects, cvImagesWorkpieceSeg, categoryWorldCenters);
    }
}

void FittingWorkpieceCoordinate::whenVerifyWorkpieceCoordinates() {
    filteredWorldCenters.clear();
    filteredWorldTopLeftPoints.clear();
    std::vector<cv::Mat> resultWorkpieceMasks;
    // 筛选结构体中的工件信息
    std::vector<workpieceInfo> filteredWorkpieces;
    if (categorizedObjects.size() == 0) {
        PLOGE << "未进行粗定位检测";
        return;
    }

    for (size_t i = 0; i < categorizedObjects.size(); ++i) {
        if (std::find(selectedWorkpieces.begin(), selectedWorkpieces.end(), i) == selectedWorkpieces.end()) {
            filteredWorldCenters.push_back(categoryWorldCenters[i]);
            filteredWorldTopLeftPoints.push_back(categoryWorldLeftTopCenters[i]);
            resultWorkpieceMasks.push_back(worldMaskImages[i]);
            if (i < workpieceFinalInfoInWorld.workpieceInfoInWorld.size()) {
                filteredWorkpieces.push_back(workpieceFinalInfoInWorld.workpieceInfoInWorld[i]);
            }
        }
    }

    workpieceFinalInfoInWorldAfterVerify = workpieceFinalInfoInWorld;
    workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld = std::move(filteredWorkpieces);
    whenGetResultInfo(filteredWorldCenters, filteredWorldTopLeftPoints);
    emit sendWorkpieceMaskImageInWorld(filteredWorldCenters, resultWorkpieceMasks);  // 发送筛选后的

    // std::string filePath = "./data/result/FittingWorkpieceCoordinate/object_finalWorldPointsInManual.txt";
    // std::ofstream outputFile(filePath, std::ios::trunc);  // 覆盖模式trunc打开文件，追加模式为app
    // if (outputFile.is_open()) {
    //     // 遍历 worldPoints
    //     for (size_t i = 0; i < resultCenters.size(); i++) {
    //         outputFile << "workpiece " << i + 1 << ": ";  // 工件编号从 1 开始
    //         outputFile << "resultCenters: ("
    //                    << resultCenters[i].x  << " , "
    //                    << resultCenters[i].y << " , "
    //                    << resultCenters[i].z << "), "
    //                    <<std::endl;
    //     }
    //     outputFile.close();  // 关闭文件
    //     std::cout << "saved to " << filePath << std::endl;
    // } else {
    //     std::cerr << "Failed to open file for writing!" << std::endl;
    // }
}

void FittingWorkpieceCoordinate::whenDisplayWeldSeamArea(workpieceBoxInWorld& boxInfo) {
    if (boxInfo.workpieceInfoInWorld.empty()) return;
    const auto& workpieces = boxInfo.workpieceInfoInWorld;

    for (const auto& info : workpieces) {
        const cv::Point3d& center3d = info.workpieceAreaRect.first;
        const auto& weldRects = info.weldAreaRect;

        cv::Point2d drawCenter2d = projectAndRotateCenter(center3d, canvasMat, railMap.rows, railMap.cols, 0);

        for (const auto& rect : weldRects) {
            double width = rect.width;
            double height = rect.height;

            double offsetX = rect.x + width / 2.0;
            double offsetY = rect.y + height / 2.0;

            cv::Point2d rel(offsetX - center3d.x, offsetY - center3d.y);

            cv::Point2d railMapWeldXY = CoordinateMapper::relativeMapToCoord(rel, drawCenter2d, g_coordMappingType);
            double railMapX = railMapWeldXY.x;
            double railMapY = railMapWeldXY.y;

            cv::Point topLeft(static_cast<int>(railMapX - width / 2), static_cast<int>(railMapY - height / 2));
            cv::Point bottomRight(static_cast<int>(railMapX + width / 2), static_cast<int>(railMapY + height / 2));
            cv::Point center(static_cast<int>(railMapX), static_cast<int>(railMapY));
            cv::rectangle(railMap, topLeft, bottomRight, weldSeamColor, weldSeamLineThickness);
            cv::circle(railMap, center, 5, cv::Scalar(0, 255, 0), -1);
        }
    }

    // 显示和保存图像
    cv::Mat railMapDisplay = railMap.clone();
    railMapRotated(railMapDisplay, railMapRotationAngle);

    for (const auto& info : workpieces) {
        const cv::Point3d& center3d = info.workpieceAreaRect.first;
        const cv::Point3d& rectTopLeft = info.workpieceAreaRect.second;

        double width = std::abs(rectTopLeft.x - center3d.x) * 2;
        double height = std::abs(rectTopLeft.y - center3d.y) * 2;

        cv::Point2d drawCenterRotated = projectAndRotateCenter(center3d, canvasMat, railMap.rows, railMap.cols, railMapRotationAngle);
        cv::circle(railMapDisplay, drawCenterRotated, 5, cv::Scalar(255, 0, 0), -1);

        std::ostringstream oss;
        oss << "(" << std::fixed << std::setprecision(1) << center3d.x << ", " << center3d.y << ", " << center3d.z << ")";
        std::string text = oss.str();

        cv::Point textPosition(static_cast<int>(drawCenterRotated.x - height / 2), static_cast<int>(drawCenterRotated.y + width / 2 + 30));
        cv::putText(railMapDisplay, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
    }

    emit sendWorkpieceResultToMainWindow(railMapDisplay);

    cv::Mat railMapRGB;
    cv::cvtColor(railMap, railMapRGB, cv::COLOR_BGR2RGB);
    boxInfo.finalRailMap = railMapRGB.clone();
    cv::imwrite("./data/workpieceCoaLoc/FinalRailMap/detected_weldSeam.jpg", railMapRGB);
}

cv::Point2d FittingWorkpieceCoordinate::projectAndRotateCenter(const cv::Point3d& center3d, const cv::Mat& canvasMat, int imageRows, int imageCols,
                                                               int rotationAngle) {
    // 将 3D 点投影到 2D
    cv::Mat centerPointOrigin = (cv::Mat_<double>(3, 1) << center3d.x, center3d.y, 1);
    cv::Mat drawresultCenter = canvasMat * centerPointOrigin;
    cv::Point2d drawCenter2d(drawresultCenter.at<double>(0, 0), drawresultCenter.at<double>(1, 0));

    // 将投影点根据角度进行旋转
    return CoordinateMapper::originalToRotated(drawCenter2d, imageRows, imageCols, rotationAngle);
}
void FittingWorkpieceCoordinate::railMapRotated(cv::Mat& image, int angle) {
    switch (angle % 360) {
        case 90:
            cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
            break;
        case 180:
            cv::rotate(image, image, cv::ROTATE_180);
            break;
        case 270:
            cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        case 0:
            break;
        default:
            throw std::invalid_argument("Unsupported rotation angle. Use 0, 90, 180, or 270.");
    }
}

//修改成三轴投影
cv::Point3d FittingWorkpieceCoordinate::computeTrackCompensation(const cv::Point3d &pt, const cv::Mat &xDir, const cv::Mat &yDir, const cv::Mat &zDir)
{
    // 三个轴方向向量可能不正交，解三元一次方程求分量
    cv::Mat T = (cv::Mat_<double>(3, 3) <<
                 xDir.at<double>(0,0), yDir.at<double>(0,0), zDir.at<double>(0,0),
                 xDir.at<double>(1,0), yDir.at<double>(1,0), zDir.at<double>(1,0),
                 xDir.at<double>(2,0), yDir.at<double>(2,0), zDir.at<double>(2,0));
    std::cout<<"三轴方向向量组成的转换矩阵:"<<T<<std::endl;
    cv::Mat P = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
    std::cout<<"准备投影到三轴上的点："<<std::endl<<"X:"<<pt.x<<std::endl<<"Y:"<<pt.y<<std::endl<<"Z:"<<pt.z<<std::endl;
    cv::Mat k;
    if (!cv::solve(T, P, k, cv::DECOMP_LU)) {
        return pt;  // 求解失败，返回原值
    }

    // k = (k1, k2, k3) 分别是三个轴的分量
    // 补偿后只保留各轴自身方向上的贡献
    cv::Point3d result(
        k.at<double>(0,0) * xDir.at<double>(0,0) +
            k.at<double>(1,0) * yDir.at<double>(0,0) +
            k.at<double>(2,0) * zDir.at<double>(0,0),

        k.at<double>(0,0) * xDir.at<double>(1,0) +
            k.at<double>(1,0) * yDir.at<double>(1,0) +
            k.at<double>(2,0) * zDir.at<double>(1,0),

        k.at<double>(0,0) * xDir.at<double>(2,0) +
            k.at<double>(1,0) * yDir.at<double>(2,0) +
            k.at<double>(2,0) * zDir.at<double>(2,0));
    std::cout<<"投影到三轴上的点："<<std::endl<<"X:"<<result.x<<std::endl<<"Y:"<<result.y<<std::endl<<"Z:"<<result.z<<std::endl;

    return result;
}
//-----------------------------------------------获取结果----------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
void FittingWorkpieceCoordinate::sortWorkpieceBoxInfo(workpieceBoxInWorld& boxInfo, const std::string& axis, const std::string& order) {
    auto& infos = boxInfo.workpieceInfoInWorld;
    if (infos.empty()) return;
    PLOGD << "railPosition" << railPosition;
    if (railPosition >= 2500 && order == "up") {
        sortOrder = "down";
    } else if (railPosition >= 2500 && order != "up") {
        sortOrder = "up";
    } else {
        sortOrder = order;
    }

    // 获取当前排序依据值（X/Y/Z）
    auto getValue = [&](const workpieceInfo& info) -> double {
        if (axis == "X")
            return info.workpieceAreaRect.first.x;
        else if (axis == "Y")
            return info.workpieceAreaRect.first.y;
        else if (axis == "Z")
            return info.workpieceAreaRect.first.z;
        else
            throw std::invalid_argument("无效的排序轴: " + axis);
    };

    // 获取分组依据（非排序轴的正负）
    auto getGroupingValue = [&](const workpieceInfo& info) -> double {
        if (axis == "X")
            return info.workpieceAreaRect.first.y;  // 分组看 Y
        else if (axis == "Y" || axis == "Z")
            return info.workpieceAreaRect.first.x;  // 分组看 X（Z 也默认用 X 来分）
        else
            throw std::invalid_argument("无效的分组轴: " + axis);
    };

    bool ascending = (sortOrder == "up");

    // 排序函数
    auto sorter = [&](std::vector<workpieceInfo>& vec) {
        std::sort(vec.begin(), vec.end(), [&](const workpieceInfo& a, const workpieceInfo& b) {
            double va = getValue(a);
            double vb = getValue(b);
            return ascending ? (va < vb) : (va > vb);
        });
    };

    // 根据分组轴（非排序轴）正负分组
    std::vector<workpieceInfo> positiveGroup, negativeGroup;
    for (const auto& info : infos) {
        if (getGroupingValue(info) >= 0)
            positiveGroup.push_back(info);
        else
            negativeGroup.push_back(info);
    }

    // 分别排序
    sorter(positiveGroup);
    sorter(negativeGroup);

    infos.clear();
    if (standardOutput) {
        if (workbenchInsertGroup == "positive") {
            infos.insert(infos.end(), positiveGroup.begin(), positiveGroup.end());
        } else if (workbenchInsertGroup == "negative") {
            infos.insert(infos.end(), negativeGroup.begin(), negativeGroup.end());
        } else if (workbenchInsertGroup == "all_negative_first") {
            infos.insert(infos.end(), negativeGroup.begin(), negativeGroup.end());
            infos.insert(infos.end(), positiveGroup.begin(), positiveGroup.end());
        } else {
            // 默认：positiveGroup 在前（对应 "all_positive_first" 或其他无效值）
            infos.insert(infos.end(), positiveGroup.begin(), positiveGroup.end());
            infos.insert(infos.end(), negativeGroup.begin(), negativeGroup.end());
        }
    } else {
        infos.insert(infos.end(), positiveGroup.begin(), positiveGroup.end());
        infos.insert(infos.end(), negativeGroup.begin(), negativeGroup.end());
    }
}

void FittingWorkpieceCoordinate::computeIOUsWithOverlap(workpieceBoxInWorld& boxInfo) {
    if (boxInfo.workpieceInfoInWorld.empty()) return;
    auto& originalInfos = boxInfo.workpieceInfoInWorld;
    size_t n = originalInfos.size();
    std::vector<bool> visited(n, false);
    std::vector<workpieceInfo> mergedInfos;

    for (size_t i = 0; i < n; ++i) {
        if (visited[i]) continue;

        // 广度优先搜索策略
        std::vector<size_t> toMergeIndices;
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            size_t current = q.front();
            q.pop();
            toMergeIndices.push_back(current);

            const auto& infoA = originalInfos[current];
            const auto& boxA = infoA.workpieceAreaRect;
            double xA = boxA.first.x - std::abs(boxA.first.x - boxA.second.x);
            double yA = boxA.first.y - std::abs(boxA.first.y - boxA.second.y);
            double wA = std::abs(boxA.first.x - boxA.second.x) * 2.0;
            double hA = std::abs(boxA.first.y - boxA.second.y) * 2.0;
            cv::Rect2d rectA(xA, yA, wA, hA);
            size_t startSearch = (current >= wpIOUSearchNum) ? current - wpIOUSearchNum : 0;
            size_t endSearch = std::min(current + wpIOUSearchNum + 1, n);

            for (size_t j = startSearch; j < endSearch; ++j) {
                if (visited[j] || j == current) continue;

                const auto& infoB = originalInfos[j];
                const auto& boxB = infoB.workpieceAreaRect;
                double xB = boxB.first.x - std::abs(boxB.first.x - boxB.second.x);
                double yB = boxB.first.y - std::abs(boxB.first.y - boxB.second.y);
                double wB = std::abs(boxB.first.x - boxB.second.x) * 2.0;
                double hB = std::abs(boxB.first.y - boxB.second.y) * 2.0;
                cv::Rect2d rectB(xB, yB, wB, hB);

                double interArea = (rectA & rectB).area();
                // PLOGD << "interArea" << interArea;
                double unionArea = rectA.area() + rectB.area() - interArea;
                // std::cout << "iou" << (interArea / unionArea) << std::endl;

                if (unionArea > 0.0 && (interArea / unionArea) > iouThreshold) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }

        // 找掩膜最大为主体
        int mainIdx = -1;
        double maxMaskArea = 0.0;
        for (size_t idx : toMergeIndices) {
            double area = cv::countNonZero(originalInfos[idx].workpiece_weld_Obj.first.boxMask);
            if (area > maxMaskArea) {
                maxMaskArea = area;
                mainIdx = static_cast<int>(idx);
            }
        }

        if (mainIdx == -1) continue;
        const auto& mainInfo = originalInfos[mainIdx];

        // 合并区域范围
        cv::Rect2d unionRect;
        for (size_t k = 0; k < toMergeIndices.size(); ++k) {
            const auto& box = originalInfos[toMergeIndices[k]].workpieceAreaRect;
            double x = box.first.x - std::abs(box.first.x - box.second.x);
            double y = box.first.y - std::abs(box.first.y - box.second.y);
            double w = std::abs(box.first.x - box.second.x) * 2.0;
            double h = std::abs(box.first.y - box.second.y) * 2.0;
            cv::Rect2d rect(x, y, w, h);
            if (k == 0)
                unionRect = rect;
            else
                unionRect |= rect;
        }

        cv::Point3d newCenter(unionRect.x + unionRect.width / 2.0, unionRect.y + unionRect.height / 2.0, mainInfo.workpieceAreaRect.first.z);
        cv::Point3d newTopLeft(unionRect.x, unionRect.y, mainInfo.workpieceAreaRect.second.z);

        workpieceInfo newInfo = mainInfo;
        newInfo.workpieceAreaRect = {newCenter, newTopLeft};

        int rotationAngle = 0;
        cv::Point2d ptMain = projectAndRotateCenter(mainInfo.workpieceAreaRect.first, canvasMat, railMap.rows, railMap.cols, rotationAngle);
        cv::Point2d ptNewCenter = projectAndRotateCenter(newCenter, canvasMat, railMap.rows, railMap.cols, rotationAngle);
        cv::line(railMap, ptMain, ptNewCenter, cv::Scalar(255, 0, 0), 2);

        for (size_t idx : toMergeIndices) {
            if (idx == static_cast<size_t>(mainIdx)) continue;

            const auto& info = originalInfos[idx];
            newInfo.weldAreaRect.insert(newInfo.weldAreaRect.end(), info.weldAreaRect.begin(), info.weldAreaRect.end());

            workpieceIOUInfo iou;
            iou.cameraOriginalMat = info.cameraOriginalMat;
            iou.cameraSegMat = info.cameraSegMat;
            iou.workpiece_weld_Mask = info.workpiece_weld_Mask;
            iou.workpiece_weld_Obj = info.workpiece_weld_Obj;
            newInfo.workpieceIouInfos.emplace_back(std::move(iou));

            cv::Point2d ptSrc = projectAndRotateCenter(info.workpieceAreaRect.first, canvasMat, railMap.rows, railMap.cols, rotationAngle);
            cv::line(railMap, ptSrc, ptNewCenter, cv::Scalar(255, 0, 0), 1);
        }
        mergedInfos.push_back(std::move(newInfo));
    }

    boxInfo.workpieceInfoInWorld = std::move(mergedInfos);
}

void FittingWorkpieceCoordinate::whenGetResultInfo(const std::vector<cv::Point3d> resultCenters, const std::vector<cv::Point3d> resultLeftTop) {
    // 保证数量一致并不超过已有工件数量
    size_t count = std::min({resultCenters.size(), resultLeftTop.size(), workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.size()});
    for (size_t i = 0; i < count; ++i) {
        workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld[i].workpieceAreaRect = std::make_pair(resultCenters[i], resultLeftTop[i]);
    }
}
void FittingWorkpieceCoordinate::applyTrackOffsetCompensation(workpieceBoxInWorld& info) {
    qDebug() << "applyTrackComp:" << applyTrackCompensation << "xDirEmpty:" << info.xAxisTrackDirection.empty();
    if (!applyTrackCompensation || info.xAxisTrackDirection.empty()) return;

    for (auto& wp : info.workpieceInfoInWorld) {
        auto& center = wp.workpieceAreaRect.first;
        auto& topLeft = wp.workpieceAreaRect.second;

        center = computeTrackCompensation(center, info.xAxisTrackDirection,
                                          info.yAxisTrackDirection, info.zAxisTrackDirection);
        topLeft = computeTrackCompensation(topLeft, info.xAxisTrackDirection,
                                           info.yAxisTrackDirection, info.zAxisTrackDirection);

        for (auto& rect : wp.weldAreaRect) {
            cv::Point3d pt3d(rect.x, rect.y, 0.0);
            cv::Point3d newPt = computeTrackCompensation(pt3d, info.xAxisTrackDirection,
                                                         info.yAxisTrackDirection, info.zAxisTrackDirection);
            rect.x = newPt.x;
            rect.y = newPt.y;
        }
    }
}
//-------------计算基座位置和视点位姿-------------------

namespace {

cv::Point2d normalize2d(const cv::Point2d& v) {
    double n = std::sqrt(v.x * v.x + v.y * v.y);
    if (n < 1e-9) {
        return cv::Point2d(0.0, 0.0);
    }
    return cv::Point2d(v.x / n, v.y / n);
}

double dot2d(const cv::Point2d& a, const cv::Point2d& b) { return a.x * b.x + a.y * b.y; }

double cross2d(const cv::Point2d& a, const cv::Point2d& b) { return a.x * b.y - a.y * b.x; }

void drawCoordAxis(cv::Mat& img, const cv::Point2d& origin, const cv::Point2d& xDir, const cv::Point2d& yDir, double len) {
    cv::Point o(cvRound(origin.x), cvRound(origin.y));

    cv::Point xEnd(cvRound(origin.x + xDir.x * len), cvRound(origin.y + xDir.y * len));

    cv::Point yEnd(cvRound(origin.x + yDir.x * len), cvRound(origin.y + yDir.y * len));

    // X轴：红色
    cv::arrowedLine(img, o, xEnd, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    // Y轴：绿色
    cv::arrowedLine(img, o, yEnd, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // 原点：蓝色
    cv::circle(img, o, 5, cv::Scalar(255, 0, 0), -1);
}

int countSkeletonNeighbors(const cv::Mat& skeleton, int x, int y) {
    int count = 0;

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }

            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= skeleton.cols || ny < 0 || ny >= skeleton.rows) {
                continue;
            }

            if (skeleton.at<uchar>(ny, nx) > 0) {
                ++count;
            }
        }
    }

    return count;
}

std::vector<cv::Point> findSkeletonEndpoints(const cv::Mat& skeleton) {
    std::vector<cv::Point> endpoints;

    for (int y = 1; y < skeleton.rows - 1; ++y) {
        for (int x = 1; x < skeleton.cols - 1; ++x) {
            if (skeleton.at<uchar>(y, x) == 0) {
                continue;
            }

            int neighbors = countSkeletonNeighbors(skeleton, x, y);

            if (neighbors == 1) {
                endpoints.emplace_back(x, y);
            }
        }
    }

    return endpoints;
}

std::pair<cv::Point, cv::Point> findFarthestPair(const std::vector<cv::Point>& pts) {
    double maxDist2 = -1.0;
    std::pair<cv::Point, cv::Point> result;

    for (size_t i = 0; i < pts.size(); ++i) {
        for (size_t j = i + 1; j < pts.size(); ++j) {
            double dx = pts[i].x - pts[j].x;
            double dy = pts[i].y - pts[j].y;
            double d2 = dx * dx + dy * dy;

            if (d2 > maxDist2) {
                maxDist2 = d2;
                result = std::make_pair(pts[i], pts[j]);
            }
        }
    }

    return result;
}

cv::Point2d chooseNearestImageAxis(const cv::Point2d& dir) {
    std::vector<cv::Point2d> axes = {cv::Point2d(1.0, 0.0), cv::Point2d(-1.0, 0.0), cv::Point2d(0.0, 1.0), cv::Point2d(0.0, -1.0)};

    cv::Point2d bestAxis = axes[0];
    double bestDot = -1.0;

    cv::Point2d ndir = normalize2d(dir);

    for (const auto& axis : axes) {
        double d = dot2d(ndir, axis);
        if (d > bestDot) {
            bestDot = d;
            bestAxis = axis;
        }
    }

    return bestAxis;
}

}  // namespace

// 这里传入的掩膜图像如果是与三轴1：1的尺度是最好
// 即输入的图像为像素坐标系转换到机器人基座坐标系在三轴原点处的坐标系中；
// 目前默认位于地面高度为2m，后续数值不定，所以目前计划不以三轴坐标系，聚焦于焊缝坐标系
// 旋转矩阵转换欧拉角abc函数为： MyToolFunc::extractEulerZYX(R_ref, currentABC);
void FittingWorkpieceCoordinate::computeBaseOffsetAndViewpointsFromMask(workpieceBoxInWorld& info) {
    QDir().mkpath("./data/test/output");

    // ===== 第一步：获取单例配置对象 =====
    ViewPlanningConfig& viewConfig = ViewPlanningConfig::getInstance();

    for (size_t i = 0; i < info.workpieceInfoInWorld.size(); ++i) {
        auto& wp = info.workpieceInfoInWorld[i];
        auto& segObj = wp.workpiece_weld_Obj.first;

        const int classId = segObj.label;

        if (segObj.boxMask.empty()) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: mask empty, index = " << i;
            continue;
        }

        // ===== 第二步：从单例配置中获取该类别的视点配置 =====
        ViewClassConfig* classConfig = nullptr;
        if (viewConfig.classConfigs.find(classId) != viewConfig.classConfigs.end()) {
            classConfig = &viewConfig.classConfigs[classId];
        } else {
            PLOGW << "computeBaseOffsetAndViewpointsFromMask: No view config found for classId " << classId;
            // 若无配置，使用默认偏移0
        }

        // ================= 1. 获取标准二值mask =================
        cv::Mat grayMask;

        if (segObj.boxMask.channels() == 1) {
            grayMask = segObj.boxMask.clone();
        } else {
            cv::cvtColor(segObj.boxMask, grayMask, cv::COLOR_BGR2GRAY);
        }

        cv::Mat binMask;
        cv::threshold(grayMask, binMask, 127, 255, cv::THRESH_BINARY);

        std::vector<cv::Point> maskPts;
        cv::findNonZero(binMask, maskPts);

        if (maskPts.size() < 10) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: mask point too few, index = " << i;
            continue;
        }

        // ================= 2. PCA计算mask质心和主方向 =================
        cv::Mat data(static_cast<int>(maskPts.size()), 2, CV_32F);

        for (int k = 0; k < static_cast<int>(maskPts.size()); ++k) {
            data.at<float>(k, 0) = static_cast<float>(maskPts[k].x);
            data.at<float>(k, 1) = static_cast<float>(maskPts[k].y);
        }

        cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

        cv::Point2d origin(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

        cv::Point2d pcaDir(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

        pcaDir = normalize2d(pcaDir);

        if (std::abs(pcaDir.x) < 1e-9 && std::abs(pcaDir.y) < 1e-9) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: invalid PCA dir, index = " << i;
            continue;
        }

        wp.photoPos.clear();

        // ================= 3. 类别0和类别2：暂用PCA建立焊缝坐标系 =================
        // Plate_Plate_F0
        // TubeSide_Plate_F2
        if (classId == 0 || classId == 2) {
            /*
             * 目前暂用方案，后面根据实际工件在具体修改：
             * 1. PCA主方向作为焊缝坐标系Y轴；
             * 2. X轴为Y轴垂直方向；
             * 3. X/Y正方向先按照掩膜默认像素坐标系方向投影确定；
             * 4. 坐标系原点使用PCA求出的mask质心点；
             * 5. 后续在该焊缝坐标系下根据不同类别做固定XYZ偏移。
             */

            cv::Point2d yDir = pcaDir;

            // Y轴大方向按照图像默认坐标系方向，优先让Y朝图像右侧或下侧
            if (std::abs(yDir.x) >= std::abs(yDir.y)) {
                if (yDir.x < 0.0) {
                    yDir = -yDir;
                }
            } else {
                if (yDir.y < 0.0) {
                    yDir = -yDir;
                }
            }

            cv::Point2d xDir(-yDir.y, yDir.x);
            xDir = normalize2d(xDir);

            // 类别0有两个坐标系，分别为X朝向两边
            if (classId == 0) {
                cv::Point2d xDirA = xDir;
                cv::Point2d yDirA = yDir;

                cv::Point2d xDirB = -xDir;
                cv::Point2d yDirB = yDir;

                // ===== 从配置中获取偏移值（如果存在），否则使用默认值 =====
                double offsetXA = 0.0, offsetYA = 0.0, offsetZA = 0.0;
                double offsetXB = 0.0, offsetYB = 0.0, offsetZB = 0.0;

                if (classConfig != nullptr && classConfig->transforms.size() >= 2) {
                    // 第一个变换对应A方向
                    offsetXA = classConfig->transforms[0].offsetXYZ[0];
                    offsetYA = classConfig->transforms[0].offsetXYZ[1];
                    offsetZA = classConfig->transforms[0].offsetXYZ[2];

                    PLOGD << "Class 0 Transform A: offset=(" << offsetXA << ", " << offsetYA << ", " << offsetZA << ")";

                    // 第二个变换对应B方向
                    offsetXB = classConfig->transforms[1].offsetXYZ[0];
                    offsetYB = classConfig->transforms[1].offsetXYZ[1];
                    offsetZB = classConfig->transforms[1].offsetXYZ[2];

                    PLOGD << "Class 0 Transform B: offset=(" << offsetXB << ", " << offsetYB << ", " << offsetZB << ")";
                }

                cv::Point2d photoA = origin + xDirA * offsetXA + yDirA * offsetYA;

                cv::Point2d photoB = origin + xDirB * offsetXB + yDirB * offsetYB;

                wp.photoPos.emplace_back(photoA.x, photoA.y, offsetZA);
                wp.photoPos.emplace_back(photoB.x, photoB.y, offsetZB);

                cv::Mat pcaMaskColor;
                cv::cvtColor(binMask, pcaMaskColor, cv::COLOR_GRAY2BGR);
                pcaMaskColor.setTo(cv::Scalar(180, 180, 180), binMask > 0);

                drawCoordAxis(pcaMaskColor, origin, xDirA, yDirA, 120.0);
                drawCoordAxis(pcaMaskColor, origin, xDirB, yDirB, 80.0);

                std::string pcaSavePath =
                    "./data/test/output/workpiece_" + std::to_string(i) + "_cls_" + std::to_string(classId) + "_mask_pca_two_axis.png";

                cv::imwrite(pcaSavePath, pcaMaskColor);
            }
            // 类别2只有一个坐标系
            else if (classId == 2) {
                // ===== 从配置中获取偏移值（如果存在），否则使用默认值 =====
                double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

                if (classConfig != nullptr && classConfig->transforms.size() >= 1) {
                    offsetX = classConfig->transforms[0].offsetXYZ[0];
                    offsetY = classConfig->transforms[0].offsetXYZ[1];
                    offsetZ = classConfig->transforms[0].offsetXYZ[2];

                    PLOGD << "Class 2 Transform: offset=(" << offsetX << ", " << offsetY << ", " << offsetZ << ")";
                }

                cv::Point2d photo = origin + xDir * offsetX + yDir * offsetY;

                wp.photoPos.emplace_back(photo.x, photo.y, offsetZ);

                cv::Mat pcaMaskColor;
                cv::cvtColor(binMask, pcaMaskColor, cv::COLOR_GRAY2BGR);
                pcaMaskColor.setTo(cv::Scalar(180, 180, 180), binMask > 0);

                drawCoordAxis(pcaMaskColor, origin, xDir, yDir, 120.0);

                std::string pcaSavePath =
                    "./data/test/output/workpiece_" + std::to_string(i) + "_cls_" + std::to_string(classId) + "_mask_pca_axis.png";

                cv::imwrite(pcaSavePath, pcaMaskColor);
            }
        }

        // ================= 4. 类别1和类别3：骨架线、端点、开口方向、坐标系 =================
        // Tube_Plate_F1
        // Tube_Tube_F3
        else if (classId == 1 || classId == 3) {
            cv::Mat skeleton;
            MyToolFunc::thinning(binMask, skeleton, MyToolFunc::THINNING_GUOHALL);

            std::vector<cv::Point> skeletonPts;
            cv::findNonZero(skeleton, skeletonPts);

            if (skeletonPts.size() < 2) {
                PLOGE << "computeBaseOffsetAndViewpointsFromMask: skeleton point too few, index = " << i;
                continue;
            }

            std::vector<cv::Point> endpoints = findSkeletonEndpoints(skeleton);

            cv::Point endPt0;
            cv::Point endPt1;

            if (endpoints.size() >= 2) {
                auto farPair = findFarthestPair(endpoints);
                endPt0 = farPair.first;
                endPt1 = farPair.second;
            } else {
                auto farPair = findFarthestPair(skeletonPts);
                endPt0 = farPair.first;
                endPt1 = farPair.second;
            }

            cv::Point2d p0(endPt0.x, endPt0.y);
            cv::Point2d p1(endPt1.x, endPt1.y);

            cv::Point2d chordDir = normalize2d(p1 - p0);

            if (std::abs(chordDir.x) < 1e-9 && std::abs(chordDir.y) < 1e-9) {
                PLOGE << "computeBaseOffsetAndViewpointsFromMask: invalid chord dir, index = " << i;
                continue;
            }

            cv::Point2d normalDir(-chordDir.y, chordDir.x);

            int positiveCount = 0;
            int negativeCount = 0;

            for (const auto& sp : skeletonPts) {
                cv::Point2d q(sp.x, sp.y);

                double dist = cross2d(chordDir, q - p0);

                if (dist > 1.0) {
                    ++positiveCount;
                } else if (dist < -1.0) {
                    ++negativeCount;
                }
            }

            cv::Point2d skeletonSideDir;

            if (positiveCount >= negativeCount) {
                skeletonSideDir = normalDir;
            } else {
                skeletonSideDir = -normalDir;
            }

            // 大多数骨架线所在方向的反方向，作为焊缝开口方向
            cv::Point2d openingDir = -skeletonSideDir;
            openingDir = normalize2d(openingDir);

            /*
             * 1. 类别1/3先通过骨架线端点连接形成弦方向；
             * 2. 判断大多数骨架点在弦的哪一侧；
             * 3. 另一侧作为焊缝开口方向；
             * 4. 坐标系原点使用PCA质心；
             * 5. 坐标系轴方向先按默认图像坐标系轴向选择；
             * 6. 看开口方向离哪个图像轴最近，就把哪个方向作为焊缝坐标系X轴；
             * 7. 另一个轴按右手/垂直关系旋转得到。
             */

            cv::Point2d xDir = chooseNearestImageAxis(openingDir);
            cv::Point2d yDir(-xDir.y, xDir.x);

            // 保证Y方向大致与弦方向同向，方便后续统一
            if (dot2d(yDir, chordDir) < 0.0) {
                yDir = -yDir;
            }

            // ===== 从配置中获取偏移值（如果存在），否则使用默认值 =====
            double offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

            if (classConfig != nullptr && classConfig->transforms.size() >= 1) {
                offsetX = classConfig->transforms[0].offsetXYZ[0];
                offsetY = classConfig->transforms[0].offsetXYZ[1];
                offsetZ = classConfig->transforms[0].offsetXYZ[2];

                PLOGD << "Class " << classId << " Transform: offset=(" << offsetX << ", " << offsetY << ", " << offsetZ << ")";
            }

            cv::Point2d photo = origin + xDir * offsetX + yDir * offsetY;

            wp.photoPos.emplace_back(photo.x, photo.y, offsetZ);

            cv::Mat skeletonColor;
            cv::cvtColor(binMask, skeletonColor, cv::COLOR_GRAY2BGR);
            skeletonColor.setTo(cv::Scalar(180, 180, 180), binMask > 0);

            // 骨架线：红色
            skeletonColor.setTo(cv::Scalar(0, 0, 255), skeleton > 0);

            // 端点连线：黄色
            cv::line(skeletonColor, endPt0, endPt1, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

            // 端点：蓝色、绿色
            cv::circle(skeletonColor, endPt0, 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(skeletonColor, endPt1, 5, cv::Scalar(0, 255, 0), -1);

            // 开口方向：紫色
            cv::arrowedLine(skeletonColor, cv::Point(cvRound(origin.x), cvRound(origin.y)),
                            cv::Point(cvRound(origin.x + openingDir.x * 100.0), cvRound(origin.y + openingDir.y * 100.0)), cv::Scalar(255, 0, 255), 2,
                            cv::LINE_AA);

            // 焊缝坐标系
            drawCoordAxis(skeletonColor, origin, xDir, yDir, 120.0);

            std::string skeletonSavePath =
                "./data/test/output/workpiece_" + std::to_string(i) + "_cls_" + std::to_string(classId) + "_skeleton_endpoint_axis.png";

            cv::imwrite(skeletonSavePath, skeletonColor);
        }

        else {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: unknown classId = " << classId;
            continue;
        }

        // ================= 5. 从配置中获取视点姿态（4x4变换矩阵） =================
        // 位姿通过建立的焊缝坐标系和变换矩阵来确立
        if (classConfig != nullptr && classConfig->transforms.size() >= 1) {
            // 获取第一个变换的4x4矩阵
            const ViewTransformConfig& transform = classConfig->transforms[0];
            cv::Mat transformMat = transform.getTransformMatrix();

            // 从4x4矩阵中提取旋转部分 (3x3)
            cv::Mat R = transformMat(cv::Rect(0, 0, 3, 3)).clone();

            // 使用你项目中的函数将旋转矩阵转换为欧拉角 (ABC)
            // 注意：这里假设你有 MyToolFunc::extractEulerZYX 函数
            cv::Mat eularABC;

            if (R.rows == 3 && R.cols == 3) {
                // MyToolFunc::extractEulerZYX(R, eularABC);  // 取消注释后使用
                // wp.robotViewPose = robotPose(eularABC.at<double>(0, 0), eularABC.at<double>(1, 0), eularABC.at<double>(2, 0), 0.0, 0.0, 0.0);

                PLOGD << "Transform matrix for class " << classId << ":\n" << transformMat;
            }
        }

        // 如果无配置或不需要从矩阵提取，使用默认值
        if (wp.robotViewPose.a_ == 0.0 && wp.robotViewPose.b_ == 0.0 && wp.robotViewPose.c_ == 0.0) {
            wp.robotViewPose = robotPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    }
}

void FittingWorkpieceCoordinate::whenGetWeldBoxInfo(const std::vector<std::vector<std::array<double, 4>>>& boxInfos) {
    // workpieceFinalInfoInWorldAfterVerify = workpieceFinalInfoInWorld;
    // whenVerifyWorkpieceCoordinates();

    size_t numWp = std::min(categorizedObjects.size(), categoryWorldCenters.size());
    numWp = std::min(numWp, categoryWorldLeftTopCenters.size());
    numWp = std::min(numWp, workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.size());
    for (size_t i = 0; i < numWp; ++i) {
        workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld[i].workpieceAreaRect =
            std::make_pair(categoryWorldCenters[i], categoryWorldLeftTopCenters[i]);
        workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.resize(numWp);
    }

    PLOGD << "=== whenGetWeldBoxInfo numWp=" << numWp << " total=" << workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.size();

    // 检查 workpieceAreaRect 是否有效
    for (auto& wp : workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld) {
        if (wp.workpieceAreaRect.first.x == 0 && wp.workpieceAreaRect.first.y == 0) {
            PLOGE << "workpieceAreaRect not initialized for workpiece";
        }
    }

    size_t count = std::min(boxInfos.size(), workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld.size());

    for (size_t i = 0; i < count; ++i) {
        std::vector<cv::Rect_<double>> weldRectsForOneObj;

        for (const auto& box : boxInfos[i]) {
            // box 格式为 {centerX, centerY, width, height}
            double cx = box[0];
            double cy = box[1];
            double w = box[2];
            double h = box[3];

            double x = cx - w / 2.0;
            double y = cy - h / 2.0;

            weldRectsForOneObj.emplace_back(x, y, w, h);
        }

        // 将结果写入对应工件信息中
        workpieceFinalInfoInWorldAfterVerify.workpieceInfoInWorld[i].weldAreaRect = weldRectsForOneObj;
    }

    // 同时读取并设置地轨方向向量
    if (!cameraParameters.empty()) {
        workpieceFinalInfoInWorldAfterVerify.xAxisTrackDirection = cameraParameters[0].xAxisTrackDirection;
        workpieceFinalInfoInWorldAfterVerify.yAxisTrackDirection = cameraParameters[0].yAxisTrackDirection;
        workpieceFinalInfoInWorldAfterVerify.zAxisTrackDirection = cameraParameters[0].zAxisTrackDirection;
    }
    // std::cout <<    "TrackDirection"
    //           <<    workpieceBoxInfoInWorld.TrackDirection  <<std::endl;
    // if (workpieceFinalInfoInWorldAfterVerify.workpieceAreaRect.size() != 0) {
    //     // 排序
    //     sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterVerify);
    // }
    // emit sendFinalInfoToMain(workpieceFinalInfoInWorldAfterVerify);
    workpieceFinalInfoInWorldAfterIOU = workpieceFinalInfoInWorldAfterVerify;
    sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterIOU, sortWorldAxis, sortWorldOrder);
    computeIOUsWithOverlap(workpieceFinalInfoInWorldAfterIOU);
    applyTrackOffsetCompensation(workpieceFinalInfoInWorldAfterIOU);
    sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterIOU, sortWorldAxis, sortWorldOrder);
    whenDisplayWeldSeamArea(workpieceFinalInfoInWorldAfterIOU);
    // TODO 基座位置选解
    // computeBaseOffsetAndViewpointsFromMask(workpieceFinalInfoInWorldAfterIOU);

    emit sendFinalInfoToMain(workpieceFinalInfoInWorldAfterIOU);
}
