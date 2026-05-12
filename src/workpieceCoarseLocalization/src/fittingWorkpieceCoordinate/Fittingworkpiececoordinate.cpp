#include "FittingWorkpieceCoordinate.h"

#include "utils/common/CommonFunc.h"
std::array<cameraConfig, 6> cameraParameters;  // 相机参数数量

/**
 * @brief 构造函数，初始化时加载校准参数
 */
FittingWorkpieceCoordinate::FittingWorkpieceCoordinate() { loadCalibrationParameters(configFilePath); }

/**
 * @brief 处理工件坐标拟合
 * @param objs 检测到的物体列表
 * @param imgNum 当前图像编号
 */
void FittingWorkpieceCoordinate::whenFittingWorkpieceCoordinate(std::vector<segYolo11::ObjectYolo11Seg> objs, int imgNum) {
    std::vector<cv::Point2d> Pt2ds;
    std::vector<cv::Point3d> worldPoints;
    std::vector<int> validPixels;

    // 遍历每个 Object
    for (const auto& obj : objs) {
        // 计算矩形的中心坐标
        // cv::Point2d center = cv::Point(obj.rect.x + obj.rect.width / 2, obj.rect.y + obj.rect.height / 2);
        cv::Point2d point = cv::Point(obj.rect.x, obj.rect.y);  // 左上角坐标
        // std::cout<<"point"<<point<<std::endl;
        //  计算有效像素数量（boxMask 中值为 1 的像素）
        int validPixel = cv::countNonZero(obj.boxMask);  // 统计 boxMask 中值为 1 的像素数
        Pt2ds.push_back(point);
        validPixels.push_back(validPixel);
    }
    if (Pt2ds.empty()) {
        // 如果没有有效的点，输出日志
        emit appendFittingLog(QString(u8"未在图像 %1 中找到有效点").arg(imgNum + 1));
        return;
    }
    worldPoints = pixel2WorldCoordPoint(Pt2ds, imgNum);  // 计算世界坐标系下的坐标
    for (size_t i = 0; i < objs.size(); i++) {
        ObjectInfo objInfo;
        objInfo.object = objs[i];
        objInfo.pt3d = worldPoints[i];
        objInfo.validPixel = validPixels[i];
        objInfo.cameraIndex = imgNum;
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
    // categoryWorldLeftTopCenters在displayDetectedWorkpieces中存储。
    displayDetectedWorkpieces(categorizedObjects, cvImagesCameraOri, categoryWorldCenters);
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

std::vector<cv::Point3d> FittingWorkpieceCoordinate::pixel2WorldCoordPoint(std::vector<cv::Point2d>& Pt2ds, int cameraNumber) {
    std::vector<cv::Point3d> cameraPointsXYZ;

    Point2dto3d(cameraParameters[cameraNumber].globalPlane, cameraParameters[cameraNumber].cameraMatrix, cameraParameters[cameraNumber].distCoeffs,
                Pt2ds, cameraPointsXYZ);  // 输出相机坐标系下在对应平面上的映射
    // std::cout<<"cameraPointsXYZ"<<cameraPointsXYZ<<std::endl;
    std::vector<cv::Point3d> worldPoints = transformCameraToBase(cameraPointsXYZ, cameraParameters[cameraNumber].extrinsicMatrix);
    // std::cout<<"cameraParameters[cameraNumber].extrinsicMatrix"<<cameraParameters[cameraNumber].extrinsicMatrix<<std::endl;
    return worldPoints;
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
        cv::Mat cameraPointMat = (cv::Mat_<double>(4, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z, 1);

        // 使用 4x4 变换矩阵进行转换
        cv::Mat transformedPointMat = transformationMatrix * cameraPointMat;

        // 提取变换后的 3D 点 (x, y, z)，忽略齐次坐标的 w 分量
        cv::Point3d basePoint(transformedPointMat.at<double>(0), transformedPointMat.at<double>(1), transformedPointMat.at<double>(2));

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
std::vector<std::vector<ObjectInfo>> FittingWorkpieceCoordinate::classifyWorkpieces(const std::vector<ObjectInfo>& allObjects, double threshold) {
    std::vector<cv::Point3d> categoryLeftPts;  // 记录每个类别的左上角代表点（动态更新）
    for (const auto& obj : allObjects) {
        bool foundGroup = false;

        // 只和每个类别的代表点比较
        for (size_t i = 0; i < categoryLeftPts.size(); ++i) {
            if (calculateDistance(obj.pt3d, categoryLeftPts[i]) < threshold) {
                categorizedObjects[i].push_back(obj);  // 归入该类别
                // 计算新的类别中心点
                categoryLeftPts[i] = computeCentroid(categorizedObjects[i]);
                foundGroup = true;
                break;
            }
        }
        // 如果没有找到合适的类别，创建新类别
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
            // 计算每个 Object 对应的矩形框的中心点
            cv::Rect rect = category[i].object.rect;
            // std::cout<<"rect:"<<rect<<std::endl;
            // cv::Point2d center(rect.x , rect.y);
            cv::Point2d center(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
            // std::cout<<"beforecenter: "<<rect.x<<", "<<rect.y<<std::endl;
            // std::cout<<"center: "<<center<<std::endl;
            std::vector<cv::Point2d> categoryCenters = {};  // 存储图像坐标系下的中心坐标
            categoryCenters.push_back(center);
            std::vector<cv::Point3d> finalWorldPoints = pixel2WorldCoordPoint(categoryCenters, category[i].cameraIndex);
            // std::cout<<"finalWorldPoints:"<<finalWorldPoints<<std::endl;
            sumCenter += finalWorldPoints[0];  // 累加
        }

        // 计算该类别的平均中心坐标
        cv::Point3d averageCenter = sumCenter / count;
        // std::cout<<"averageCenter:"<<averageCenter<<std::endl;

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
    cv::Mat cameraMatrixRead = cv::Mat::zeros(3, 3, CV_64F);  // 清零相机矩阵
    cv::Mat distCoeffsRead = cv::Mat::zeros(1, 5, CV_64F);    // 清零畸变系数
    std::vector<double> planeRead;                            // 清空全局平面参数
    cv::Mat extrinsicMatrixRead;
    int cameraNumber = 0;
    // 打开 JSON 文件
    std::ifstream file(filename);
    if (!file.is_open() || file.peek() == std::ifstream::traits_type::eof()) {
        // 文件为空或无法打开，初始化一个包含多个相机的默认值
        std::cerr << "File is empty or failed to open. Initializing default values." << std::endl;

        // 创建多个默认的相机配置
        std::map<std::string, CoarseLocalizationMatrix> cameraConfigMap;

        // 假设有多个相机配置，名称为 Camera1, Camera2, Camera3
        for (int i = 1; i <= 6; ++i) {
            cv::Mat defaultCameraMatrix = cv::Mat::eye(3, 3, CV_64F);     // 默认相机矩阵
            cv::Mat defaultDistCoeffs = cv::Mat::zeros(1, 5, CV_64F);     // 默认畸变系数
            std::vector<double> defaultPlane = {0.0, 0.0, 0.0};           // 默认平面参数
            cv::Mat defaultExtrinsicMatrix = cv::Mat::eye(4, 4, CV_64F);  // 默认外参矩阵

            CoarseLocalizationMatrix defaultMatrix(defaultCameraMatrix, defaultDistCoeffs, defaultPlane, defaultExtrinsicMatrix);
            cameraConfigMap["Camera" + std::to_string(i)] = defaultMatrix;  // 使用 Camera1, Camera2, Camera3 为键
        }

        // 保存默认相机配置到文件
        std::ofstream outFile(filename);
        cereal::JSONOutputArchive outputArchive(outFile);
        outputArchive(cereal::make_nvp("Cameras", cameraConfigMap));

        std::cerr << "Default values written to file." << std::endl;
        return;
    }

    // 创建 Cereal JSON 输入归档
    cereal::JSONInputArchive inputArchive(file);

    try {
        // 反序列化所有相机的数据，假设每个相机的名称是 Camera1, Camera2, 等等
        std::map<std::string, CoarseLocalizationMatrix> cameraConfigMap;
        inputArchive(cereal::make_nvp("Cameras", cameraConfigMap));

        // 遍历所有相机并输出其参数
        for (const auto& cameraConfigPara : cameraConfigMap) {
            std::string cameraName = cameraConfigPara.first;
            CoarseLocalizationMatrix cameraData = cameraConfigPara.second;

            // 将反序列化后的数据传递到对应的变量
            cameraData.transferData(cameraMatrixRead, distCoeffsRead, planeRead, extrinsicMatrixRead);
            cameraParameters[cameraNumber].cameraMatrix = cameraMatrixRead;
            cameraParameters[cameraNumber].distCoeffs = distCoeffsRead;
            cameraParameters[cameraNumber].globalPlane = planeRead;
            cameraParameters[cameraNumber].extrinsicMatrix = extrinsicMatrixRead;
            cameraNumber++;
        }

        // cameraMatrix = cameraParameters[3].cameraMatrix;
        // distCoeffs = cameraParameters[3].distCoeffs;
        // plane = cameraParameters[3].globalPlane;
        // extrinsicMatrix = cameraParameters[3].extrinsicMatrix;
        // // 打印相机的参数
        // std::cout << "Calibration for cameraName" << 3 << ":\n";
        // std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
        // std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
        // std::cout << "ExtrinsicMatrix:\n" << extrinsicMatrix << std::endl;
        // std::cout << "Global Plane Parameters: ";
        // for (const auto& p : plane) {
        //     std::cout << p << " ";
        // }
        // std::cout << std::endl;
    } catch (const cereal::Exception& e) {
        std::cerr << "Error reading calibration parameters: " << e.what() << std::endl;
        return;
    }

    // 可以记录调试信息
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
    cv::Mat centerResult = canvasMat * centerPoint;
    double railMapX = centerResult.at<double>(0, 0);
    double railMapY = centerResult.at<double>(1, 0);
    // 确保抠出来的图像中心与 worldCenter 对齐
    int offsetX = static_cast<int>(railMapX - resizedImage.cols / 2);
    int offsetY = static_cast<int>(railMapY - resizedImage.rows / 2);
    cv::Rect roi;  // 定义一个空的矩形

    // 适配到画布
    if (offsetX >= 0 && offsetX + resizedImage.cols < railMap.cols && offsetY >= 0 && offsetY + resizedImage.rows < railMap.rows) {
        roi = cv::Rect(offsetX, offsetY, resizedImage.cols, resizedImage.rows);  // 正确设置 roi
        cv::rectangle(railMap, roi, correctColor, correctLineThickness);
        resizedImage.copyTo(railMap(roi));  // 将图像绘制到画布中
    } else {
        // 如果超出范围，调整到边界
        offsetX = std::max(0, std::min(offsetX, railMap.cols - resizedImage.cols));
        offsetY = std::max(0, std::min(offsetY, railMap.rows - resizedImage.rows));
        roi = cv::Rect(offsetX, offsetY, resizedImage.cols, resizedImage.rows);  // 正确设置 roi
        resizedImage.copyTo(railMap(roi));

        // 绘制包裹工件的警告线
        cv::rectangle(railMap, roi, warningColor, warningLineThickness);
    }
    // 将 roi 和类别索引存入全局变量
    workpieceROIs.push_back(std::make_pair(roi, categoryIdx));
    // 标注工件中心点
    cv::circle(railMap, cv::Point(static_cast<int>(railMapX), static_cast<int>(railMapY)), 4, cv::Scalar(255, 0, 0), -1);
    // std::string text = "(" + std::to_string(worldCenter.x) + ", " + std::to_string(worldCenter.y) + ", " +
    // std::to_string(worldCenter.z) + ")"; cv::Point textPosition = cv::Point(static_cast<int>(railMapX) - resizedImage.cols
    // / 2, static_cast<int>(railMapY) + resizedImage.rows / 2 + 30); cv::putText(railMap, text, textPosition,
    // cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    if (std::find(selectedWorkpieces.begin(), selectedWorkpieces.end(), categoryIdx) != selectedWorkpieces.end()) {
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

        // 使用 categoryCenters3d 作为中心坐标
        cv::Point3d worldCenter = categoryCenters3d[categoryIdx];

        // 选取 validPixel 最大的对象
        const ObjectInfo& bestObject = category.front();
        int imagNum = bestObject.cameraIndex;

        if (imagNum >= 0 && imagNum < cvImagesToDisplay.size()) {
            cv::Mat image = cvImagesToDisplay[imagNum];

            cv::Rect rect = bestObject.object.rect;
            cv::Mat boxMask = bestObject.object.boxMask;  // 掩膜矩阵
            // 生成白色背景的掩膜图像
            cv::Mat maskRegion = cv::Mat(rect.size(), CV_8UC3, cv::Scalar(255, 255, 255));  // 白色画布

            cv::Mat objectRegion = image(rect).clone();  // 原图rect区域
            objectRegion.copyTo(maskRegion, boxMask);    // 将原图rect区域中的掩膜部分复制给maskRegion
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
            cv::Mat R = cameraParameters[imagNum].extrinsicMatrix(cv::Range(0, 3), cv::Range(0, 3));
            double angleDifference;
            // 计算偏航角（yaw）
            angleDifference = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            angleDifference = angleDifference * 180.0 / CV_PI;
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
        displayDetectedWorkpieces(categorizedObjects, cvImagesCameraOri, categoryWorldCenters);
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
cv::Point3d FittingWorkpieceCoordinate::computeProjectedOffset(const cv::Point3d& pt, const cv::Mat& trackDirection, const std::string& axis) {
    if (trackDirection.empty() || trackDirection.rows != 3 || trackDirection.cols != 1) {
        std::cerr << "Invalid trackDirection vector!" << std::endl;
        return pt;
    }

    double dx = trackDirection.at<double>(0, 0);
    double dy = trackDirection.at<double>(1, 0);
    double dz = trackDirection.at<double>(2, 0);

    double t = 0.0;
    if (axis == "x") {
        if (dx == 0) return pt;
        t = pt.x / dx;
    } else if (axis == "y") {
        if (dy == 0) return pt;
        t = pt.y / dy;
    } else if (axis == "z") {
        if (dz == 0) return pt;
        t = pt.z / dz;
    } else {
        std::cerr << "Invalid axis!" << std::endl;
        return pt;
    }

    double offset_x = t * dx;
    double offset_y = t * dy;
    double offset_z = t * dz;
    cv::Point3d projected_pt(pt.x, pt.y - offset_y, pt.z - offset_z);

    // 新投影点到原点的距离
    double distance = std::sqrt(projected_pt.x * projected_pt.x + offset_y * offset_y + offset_z * offset_z);
    projected_pt = cv::Point3d(distance, projected_pt.y, projected_pt.z);
    // std::cout << "Original Point: " << pt << "\n";
    // std::cout << "Track Direction: (" << dx << ", " << dy << ", " << dz << ")\n";
    // std::cout << "Track chazhi: (" << offset_x << ", " << offset_y << ", " << offset_z << ")\n";
    // std::cout << "Projection Axis: '" << axis << "' => t = " << t << "\n";
    // std::cout << "Projected Point = pt + t * dir = " << projected_pt << "\n";
    // std::cout << "Distance from projected point to origin: " << distance << "\n";
    // std::cout << "-----------------------------\n";

    return projected_pt;
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
        if (newInfo.workpiece_weld_Obj.second.size() != 0) {
            mergedInfos.push_back(std::move(newInfo));
        }
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
    if (!applyTrackCompensation || info.trackDirection.empty()) {
        return;
    }

    for (auto& wp : info.workpieceInfoInWorld) {
        auto& center = wp.workpieceAreaRect.first;
        auto& topLeft = wp.workpieceAreaRect.second;

        center = computeProjectedOffset(center, info.trackDirection, "x");
        topLeft = computeProjectedOffset(topLeft, info.trackDirection, "x");

        // === 对每个焊缝区域 weldAreaRect 的左上角和中心做补偿（这里只对左上角做） ===
        for (auto& rect : wp.weldAreaRect) {
            cv::Point3d topLeft3d(rect.x, rect.y, 0.0);  // 默认 z=0
            cv::Point3d newTopLeft = computeProjectedOffset(topLeft3d, info.trackDirection, "x");

            rect.x = newTopLeft.x;
            rect.y = newTopLeft.y;
        }
    }
}
// 这里传入的掩膜图像如果是与三轴1：1的尺度是最好
void FittingWorkpieceCoordinate::computeBaseOffsetAndViewpointsFromMask(workpieceBoxInWorld& info) {
    for (size_t i = 0; i < info.workpieceInfoInWorld.size(); ++i) {
        auto& wp = info.workpieceInfoInWorld[i];
        auto& segObj = wp.workpiece_weld_Obj.first;

        const int classId = segObj.label;
        cv::Mat mask = segObj.boxMask.clone();

        if (mask.empty()) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: mask empty, index = " << i;
            continue;
        }
        // ================= 1. 获取标准二值mask =================
        cv::Mat grayMask;

        if (segObj.boxMask.channels() == 1) {
            grayMask = segObj.boxMask;
        } else {
            cv::cvtColor(segObj.boxMask, grayMask, cv::COLOR_BGR2GRAY);
        }

        // 强制转标准二值图
        cv::Mat binMask;
        cv::threshold(grayMask, binMask, 127, 255, cv::THRESH_BINARY);

        std::vector<cv::Point> maskPts;
        cv::findNonZero(binMask, maskPts);

        if (maskPts.size() < 10) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: mask point too few, index = " << i;
            continue;
        }

        // ================= 2. 根据mask PCA建立子焊缝坐标系 =================
        cv::Mat data(static_cast<int>(maskPts.size()), 2, CV_32F);

        for (int k = 0; k < static_cast<int>(maskPts.size()); ++k) {
            data.at<float>(k, 0) = static_cast<float>(maskPts[k].x);
            data.at<float>(k, 1) = static_cast<float>(maskPts[k].y);
        }

        cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

        cv::Point2d origin(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

        cv::Point2d xDir(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

        double xNorm = std::sqrt(xDir.x * xDir.x + xDir.y * xDir.y);
        if (xNorm < 1e-6) {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: invalid PCA xDir, index = " << i;
            continue;
        }

        xDir.x /= xNorm;
        xDir.y /= xNorm;

        // PCA主方向为X，垂直方向为Y
        cv::Point2d yDir(-xDir.y, xDir.x);

        // ================= 3. 保存PCA主方向调试图：画在mask图上 =================
        cv::Mat pcaMaskColor;
        cv::cvtColor(binMask, pcaMaskColor, cv::COLOR_GRAY2BGR);

        // mask区域显示为浅灰色
        pcaMaskColor.setTo(cv::Scalar(180, 180, 180), binMask > 0);

        const double lineLen = 200.0;

        cv::Point p0(static_cast<int>(origin.x - xDir.x * lineLen), static_cast<int>(origin.y - xDir.y * lineLen));

        cv::Point p1(static_cast<int>(origin.x + xDir.x * lineLen), static_cast<int>(origin.y + xDir.y * lineLen));

        // X方向：红色
        cv::line(pcaMaskColor, p0, p1, cv::Scalar(0, 0, 255), 2);

        // 原点：蓝色
        cv::circle(pcaMaskColor, cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)), 5, cv::Scalar(255, 0, 0), -1);

        std::string pcaSavePath = "./data/test/output/workpiece_" + std::to_string(i) + "_cls_" + std::to_string(classId) + "_mask_pca_axis.png";

        cv::imwrite(pcaSavePath, pcaMaskColor);

        // ================= 4. 类别1和类别3额外保存骨架线效果 =================
        // Tube_Plate_F1
        // Tube_Tube_F3
        if (classId == 1 || classId == 3) {
            cv::Mat skeleton;
            MyToolFunc::thinning(binMask, skeleton, MyToolFunc::THINNING_ZHANGSUEN);

            cv::Mat skeletonColor;
            cv::cvtColor(binMask, skeletonColor, cv::COLOR_GRAY2BGR);
            skeletonColor.setTo(cv::Scalar(0, 0, 255), skeleton > 0);

            std::string skeletonSavePath = "./data/test/output/workpiece_" + std::to_string(i) + "_cls_" + std::to_string(classId) + "_skeleton.png";

            cv::imwrite(skeletonSavePath, skeletonColor);
        }

        // ================= 5. 根据类别分支规划偏移和视点 =================

        double offsetX = 0.0;
        double offsetY = 0.0;
        double offsetZ = 0.0;

        // Plate_Plate_F0
        if (classId == 0) {
            offsetX = 0.0;
            offsetY = 0.0;
            offsetZ = 0.0;
        }
        // Tube_Plate_F1
        else if (classId == 1) {
            offsetX = 0.0;
            offsetY = 0.0;
            offsetZ = 0.0;
        }
        // TubeSide_Plate_F2
        else if (classId == 2) {
            offsetX = 0.0;
            offsetY = 0.0;
            offsetZ = 0.0;
        }
        // Tube_Tube_F3
        else if (classId == 3) {
            offsetX = 0.0;
            offsetY = 0.0;
            offsetZ = 0.0;
        } else {
            PLOGE << "computeBaseOffsetAndViewpointsFromMask: unknown classId = " << classId;
            continue;
        }

        // ================= 6. 当前阶段先全部填0 =================
        wp.robotViewPose = robotPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}

void FittingWorkpieceCoordinate::whenGetWeldBoxInfo(const std::vector<std::vector<std::array<double, 4>>>& boxInfos) {
    // 安全处理：避免超过已有工件数量
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
    MyMatrixTrackDirection Track;
    workpieceFinalInfoInWorldAfterVerify.trackDirection = Track.readTrackDirectionFromJsonFile(trackFilePath);
    // std::cout <<    "TrackDirection"
    //           <<    workpieceBoxInfoInWorld.TrackDirection  <<std::endl;
    // if (workpieceFinalInfoInWorldAfterVerify.workpieceAreaRect.size() != 0) {
    //     // 排序
    //     sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterVerify);
    // }
    // emit sendFinalInfoToMain(workpieceFinalInfoInWorldAfterVerify);
    workpieceFinalInfoInWorldAfterIOU = workpieceFinalInfoInWorldAfterVerify;
    // sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterIOU, sortWorldAxis, sortWorldOrder);
    computeIOUsWithOverlap(workpieceFinalInfoInWorldAfterIOU);
    sortWorkpieceBoxInfo(workpieceFinalInfoInWorldAfterIOU, sortWorldAxis, sortWorldOrder);
    whenDisplayWeldSeamArea(workpieceFinalInfoInWorldAfterIOU);
    // TODO 基座位置选解
    // computeBaseOffsetAndViewpointsFromMask(workpieceFinalInfoInWorldAfterIOU);
    // TODO 下面的要进行修改，原逻辑是垂直投影，现在为求解三元一次方程；
    applyTrackOffsetCompensation(workpieceFinalInfoInWorldAfterIOU);

    emit sendFinalInfoToMain(workpieceFinalInfoInWorldAfterIOU);
}
