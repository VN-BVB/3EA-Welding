#include "CameraAndLaserPlaneCalibration.h"

#include <algorithm>
#include <direct.h>
#include <sys/stat.h>

namespace {

bool directoryExists(const std::string& path) {
    struct _stat info;
    if (_stat(path.c_str(), &info) != 0) {
        return false;
    }
    return (info.st_mode & _S_IFDIR) != 0;
}

std::string normalizedPath(std::string path) {
    std::replace(path.begin(), path.end(), '/', '\\');
    return path;
}

std::string parentDirectoryPath(const std::string& filename) {
    const std::string path = normalizedPath(filename);
    const size_t slashPos = path.find_last_of('\\');
    if (slashPos == std::string::npos) {
        return std::string();
    }
    return path.substr(0, slashPos);
}

void createDirectoryRecursive(const std::string& path) {
    if (path.empty() || directoryExists(path)) {
        return;
    }

    const std::string parentPath = parentDirectoryPath(path);
    if (!parentPath.empty() && parentPath != path) {
        createDirectoryRecursive(parentPath);
    }

    _mkdir(path.c_str());
}

bool hasSuffix(const std::string& str, const std::string& suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }
    return str.substr(str.length() - suffix.length()) == suffix;
}

std::string resolvePlanePointCloudPath() {
    return "./data/plane/planeData.pcd";
}

}  // namespace

/**
 * @brief CamraCalibration  相机标定
 * @param files             文件名
 * @param cameraMatrix      内参矩阵
 * @param distCoeffs        畸变系数
 * @param tvecsMat          平移矩阵
 * @param rvecsMat          旋转矩阵
 */
void CameraAndLaserPlaneCalibration::cameraCalibration(std::vector<std::string>& files, cv::Mat& cameraMatrix,
                                                       cv::Mat& distCoeffs, std::vector<cv::Mat>& tvecsMat,
                                                       std::vector<cv::Mat>& rvecsMat, int& imageCount, double& totalErr) {
    // 读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
    imageCount = 0;  // 图像数量
    tvecsMat.clear();
    rvecsMat.clear();
    cv::Size imageSize;                                        // 图像的尺寸
    cv::Size boardSize = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);  // 标定板上每行、列的角点数
    std::vector<cv::Point2f> imagePointsBuf;                   // 缓存每幅图像上检测到的角点
    std::vector<std::vector<cv::Point2f>> imagePointsSeq;      // 保存检测到的所有角点

    for (int i = 0; i < files.size(); i++) {
        cv::Mat imageInput = cv::imread(files[i]);
        /* 提取角点 */
        // imageInput:输入图片, board_size:角点尺寸, image_points_buf:检测结果
        // cv::imshow("Input Image", imageInput);
        // cv::waitKey(100);
        // if (0 == cv::findChessboardCorners(imageInput, boardSize, imagePointsBuf)) {
        //     std::cout << "Num " << i << " can not find chessboard corners!\n";  // 找不到角点
        //     continue;
        // }
        // 求图像特征点
        whenCalculateImagePoints(i, imageInput, boardSize, imagePointsBuf);
        imageCount++;
        if (imageCount == 1) {  // 读入第一张图片时获取图像宽高信息
            imageSize.width = imageInput.cols;
            imageSize.height = imageInput.rows;
        }
        imagePointsSeq.push_back(imagePointsBuf);  // 保存角点
    }

    // int total = imagePointsSeq.size();
    // std::cout << "Total num is" << total << std::endl;
    // std::cout << "get corner suc";

    /* 棋盘三维信息 */
    // cv::Size2f squareSize = cv::Size2f(BOARD_SCALE, BOARD_SCALE);  // 实际测量得到的标定板上每个棋盘格的大小
    cv::Size2f squareSize = cv::Size2f(BOARD_SCALE, BOARD_SCALE);  // 实际测量得到的标定板上每个棋盘格的大小
    std::vector<std::vector<cv::Point3f>> objectPoints;            // 保存标定板上角点的三维坐标
    cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));    // 摄像机内参数矩阵
    std::vector<int> pointCounts;                                // 每幅图像中角点的数量
    distCoeffs = cv::Mat(1, 5, CV_64FC1,
                         cv::Scalar::all(0));  // 摄像机的畸变系数：k1,k2,p1,p2,k3

    /* 初始化标定板上角点的三维坐标 */
    int i, j, t;
    for (t = 0; t < imageCount; t++) {
        std::vector<cv::Point3f> tempPointSet;
        //        qDebug() << "board_size.height:" << board_size.height;
        //        qDebug() << "board_size.width:" << board_size.width;
        for (i = 0; i < boardSize.height; i++) {
            for (j = 0; j < boardSize.width; j++) {
                cv::Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.y = i * squareSize.width;
                realPoint.x = j * squareSize.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
                //                qDebug() << realPoint.x << "," << realPoint.y;
            }
        }
        objectPoints.push_back(tempPointSet);
    }

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i = 0; i < imageCount; i++) {
        pointCounts.push_back(boardSize.width * boardSize.height);
    }

    /* 开始标定 */
    /* double cv::calibrateCamera(
     * InputArrayOfArrays objectPoints,  // 三维物体空间点的集合，类型为 std::vector<std::vector<cv::Point3f>>
     * InputArrayOfArrays imagePoints,   // 二维图像空间点的集合，类型为 std::vector<std::vector<cv::Point2f>>
     * Size imageSize,                   // 图像尺寸，类型为 cv::Size(width, height)
     * InputOutputArray cameraMatrix,    // 输出的相机内参矩阵，类型为 cv::Mat
     * InputOutputArray distCoeffs,      // 输出的畸变系数，类型为 cv::Mat
     * OutputArrayOfArrays rvecs,        // 输出的旋转向量，类型为 std::vector<cv::Mat>
     * OutputArrayOfArrays tvecs,        // 输出的平移向量，类型为 std::vector<cv::Mat>
     * int flags = 0,                    // 标定选项，如是否固定某些参数
     * TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON)  // 迭代终止准则
     * );
     */
    cv::calibrateCamera(objectPoints, imagePointsSeq, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
                        cv::CALIB_FIX_K3);  //

    // std::cout << "calibration succ" << std::endl;

    /* 对标定结果进行评价 */
    /*
     * 说明：在进行标定时，相机是相对固定的，通过移动标定板并拍摄图片来得到不同位置姿态的标定板图片，
     * 但实际上，我们是认为标定板在空间中的三维坐标是固定不变的，通过移动相机来拍摄不同角度下的标定板。
     * 得到每张标定板角点在图像中的二维坐标后，我们进行了相机的标定，得到的是相机的内参（内参矩阵和畸变系数），
     * 以及相机的外参（假设标定板不动，相机移动前提下的相机的旋转平移矩阵）。
     * 通过得到的内外参矩阵以及固定的真实棋盘格三维空间角点坐标，我们可以得到这些三维点重新投影于对应的二维图片中应当的位置，
     * 从而将其与之前检测到的角点坐标进行比较，得到之前检测角点时的误差。
     * 存在的问题：我们需要使用角点检测的结果来计算相机的内外参数，然后又使用这些内外参数来进行重投影，最终用重投影结果来评价角点检测的质量。
     */
    totalErr = 0.0;                         // 所有图像的平均误差的总和
    double err = 0.0;                       // 每幅图像的平均误差
    std::vector<cv::Point2f> imagePoints2;  // 保存重新计算得到的投影点
    for (i = 0; i < imageCount; i++) {      // imageCount
        std::vector<cv::Point3f> tempPointSet = objectPoints[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
         */

        // 打印三维点坐标
        // std::cout << "tempPointSet for image " << i << ":\n";
        // for (const auto& pt : tempPointSet) {
        //     std::cout << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
        //}
        cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, imagePoints2);

        // std::cout << "imagePoints2 for image " << i << ":\n";
        // for (const auto& pt : imagePoints2) {
        //     std::cout << "(" << pt.x << ", " << pt.y << ")\n";
        // }

        /* 计算新的投影点和旧的投影点之间的误差 */
        std::vector<cv::Point2f> tempImagePoint = imagePointsSeq[i];  // 亚像素集合
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat imagePoints2Mat = cv::Mat(1, imagePoints2.size(), CV_32FC2);
        for (int j = 0; j < tempImagePoint.size(); j++) {
            imagePoints2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(imagePoints2[j].x, imagePoints2[j].y);
            tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = cv::norm(imagePoints2Mat, tempImagePointMat, cv::NORM_L2);
        totalErr += err /= pointCounts[i];
    }
}
void CameraAndLaserPlaneCalibration::planeCalibration(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                                                      std::vector<double>& globalPlane,
                                                      CameraAndLaserPlaneCalibration::ErrorMetrics& errorMetrics) {
    cv::Size boardSize = cv::Size(6, 9);  // 标定板上每行、列的角点数
    cv::Size2f squareSize = cv::Size2f(60, 60);
    // cv::Size boardSize = cv::Size(BOARD_HEIGHT, BOARD_WIDTH);  // 标定板上每行、列的角点数
    // cv::Size2f squareSize = cv::Size2f(BOARD_SCALE, BOARD_SCALE);  // 实际测量得到的标定板上每个棋盘格的大小
    std::vector<cv::Point2f> imagePointsBuf;               // 缓存每幅图像上检测到的角点
    std::vector<std::vector<cv::Point2f>> imagePointsSeq;  // 保存检测到的所有角点;
    std::vector<cv::Point3f> objectCornerPoints;           // 标定板坐标系中的世界坐标点
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            cv::Point3f realPoint;
            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.y = i * squareSize.width;
            realPoint.x = j * squareSize.height;
            realPoint.z = 0;
            objectCornerPoints.push_back(realPoint);
        }
    }
    int i;
    for (i = 0; i < files.size(); i++) {
        cv::Mat imageInput = cv::imread(files[i]);
        imagePointsBuf.clear();

        if (imageInput.empty()) {
            std::cout << "Num " << i << " can not read image!\n";
            continue;
        }
        // 求图像特征点
        //  whenCalculateImagePoints(i,imageInput,boardSize,imagePointsBuf);
        //  if (i == files.size() - 1) {  // 处理最后一张图片后
        //      cv::waitKey(500);
        //      cv::destroyAllWindows();
        //  }
        //-------------------------------------------------------------------
#ifdef findChessboardCorner
        if (0 == cv::findChessboardCornersSB(imageInput, boardSize, imagePointsBuf, cv::CALIB_CB_NORMALIZE_IMAGE)) {
            std::cout << "Num " << i << " can not find chessboard corners!\n";  // 找不到角点
            continue;
        }
        cv::Mat viewGray;
        cv::cvtColor(imageInput, viewGray, cv::COLOR_RGB2GRAY);

        /* 亚像素精确化 */
        // 对已经检测到的角点进行亚像素优化，提供已经检测到的角点，搜索区域大小5*5，不适用零区域，迭代30次或精度达到0.1后停止
        cv::cornerSubPix(viewGray, imagePointsBuf, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
#else
        cv::bitwise_not(imageInput, imageInput);  // 反转灰度图像
        if (0 == calculate_Image_Points(imageInput, boardSize, imagePointsBuf)) {
            std::cout << "Num " << i << " can not  find CirclesGrid !\n";  // 找不到角点
            continue;
        }

#endif

        // 统一角点顺序
        if (imagePointsBuf.empty()) {
            std::cout << "Num " << i << " has empty image points!\n";
            continue;
        }

        if (imagePointsBuf[0].x > imagePointsBuf[imagePointsBuf.size() - 1].x) {
            std::vector<cv::Point2f> buf;
            for (int i = 0; i < imagePointsBuf.size(); i++) {
                buf.push_back(imagePointsBuf[imagePointsBuf.size() - 1 - i]);
            }
            imagePointsBuf.clear();
            imagePointsBuf = buf;
        }

        //-------------------------------------------------------------------
        imagePointsSeq.push_back(imagePointsBuf);  // 标定板-特征点-坐标
        int pointIndex = 0;
        for (int j = 0; j < imagePointsBuf.size(); j++) {
            cv::circle(imageInput, cv::Point(imagePointsBuf[j].x, imagePointsBuf[j].y), 1, cv::Scalar(0, 0, 255),
                       -1);  // 画圆
            cv::Point textPosition(imagePointsBuf[j].x, imagePointsBuf[j].y);
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.5;
            cv::Scalar fontColor(255, 0, 0);  // 文本颜色，BGR格式
            int fontThickness = 1;
            cv::putText(imageInput, std::to_string(pointIndex++), textPosition, fontFace, fontScale, fontColor, fontThickness);
        }
        // // 打印当前图像的角点坐标
        // std::cout << "Image " << i << " corner points:\n";
        // for (int j = 0; j < imagePointsBuf.size(); j++) {
        //     std::cout << "Point " << j << ": ("
        //               << imagePointsBuf[j].x << ", "
        //               << imagePointsBuf[j].y << ")\n";
        // }

    }
    if (imagePointsSeq.empty()) {
        globalPlane.clear();
        errorMetrics = {0.0, 0.0, 0.0, 0.0};
        return;
    }

    std::vector<cv::Mat> extrinsicMatrices;
    calibrationSolveExtrinsics(cameraMatrix, distCoeffs, objectCornerPoints, imagePointsSeq,
                               extrinsicMatrices);  // 已知内参求外参
    if (extrinsicMatrices.empty()) {
        globalPlane.clear();
        errorMetrics = {0.0, 0.0, 0.0, 0.0};
        return;
    }

    if (!extrinsicMatrices.empty()) {
        std::cout << "extrinsicMatrices[0] =\n"
                  << extrinsicMatrices[0] << std::endl;
    }
    CameraAndLaserPlaneCalibration::computePlaneEquations(extrinsicMatrices, objectCornerPoints, globalPlane);
    // 验证平面拟合是否能正确反应到世界坐标系
    errorMetrics = CameraAndLaserPlaneCalibration::evaluatePlaneFittingError(
        imagePointsSeq, objectCornerPoints, extrinsicMatrices, cameraMatrix, distCoeffs, globalPlane);
}

void CameraAndLaserPlaneCalibration::whenCalculateImagePoints(int i, const cv::Mat& imageInput, cv::Size boardSize,
                                                              std::vector<cv::Point2f>& imagePointsBuf) {
#ifdef findChessboardCorner
    if (0 == cv::findChessboardCornersSB(imageInput, boardSize, imagePointsBuf)) {
        std::cout << "Num " << i << " can not find chessboard corners!\n";  // 找不到角点
        return;
    }
    cv::Mat viewGray;
    cv::cvtColor(imageInput, viewGray, cv::COLOR_RGB2GRAY);

    /* 亚像素精确化 */
    // 对已经检测到的角点进行亚像素优化，提供已经检测到的角点，搜索区域大小5*5，不适用零区域，迭代30次或精度达到0.1后停止
    cv::cornerSubPix(viewGray, imagePointsBuf, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

#else
    cv::bitwise_not(imageInput, imageInput);  // 反转灰度图像
    if (0 == calculate_Image_Points(imageInput, boardSize, imagePointsBuf)) {
        std::cout << "Num " << i << " can not  find CirclesGrid !\n";  // 找不到角点
        return;
    }

#endif
    // 统一角点顺序
    if (imagePointsBuf[0].x > imagePointsBuf[imagePointsBuf.size() - 1].x) {
        std::vector<cv::Point2f> buf;
        for (int i = 0; i < imagePointsBuf.size(); i++) {
            buf.push_back(imagePointsBuf[imagePointsBuf.size() - 1 - i]);
        }
        imagePointsBuf.clear();
        imagePointsBuf = buf;
    }
    int pointIndex = 0;
    for (int j = 0; j < imagePointsBuf.size(); j++) {
        cv::circle(imageInput, cv::Point(imagePointsBuf[j].x, imagePointsBuf[j].y), 1, cv::Scalar(0, 0, 255),
                   -1);  // 画圆
        cv::Point textPosition(imagePointsBuf[j].x, imagePointsBuf[j].y);
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        cv::Scalar fontColor(255, 0, 0);  // 文本颜色，BGR格式
        int fontThickness = 1;
        cv::putText(imageInput, std::to_string(pointIndex++), textPosition, fontFace, fontScale, fontColor, fontThickness);
    }
    // // 打印当前图像的角点坐标
    // std::cout << "Image " << i << " corner points:\n";
    // for (int j = 0; j < imagePointsBuf.size(); j++) {
    //     std::cout << "Point " << j << ": ("
    //               << imagePointsBuf[j].x << ", "
    //               << imagePointsBuf[j].y << ")\n";
    // }
    // 保存角点图片
    //            std::string outputFilename = std::to_string(i) + "output_image.bmp";
    //            bool result = cv::imwrite(outputFilename, imageInput);
    //            if (!result) {
    //                std::cerr << "Failed to save image." << std::endl;
    //                return;
    //            }
    return;
}
/**
 * @brief GetImage  取得原图
 * @param imgSrc   图像名称
 * @param srcImage 原始图像
 */
void CameraAndLaserPlaneCalibration::GetImage(std::string imgSrc, cv::Mat& srcImage) { srcImage = cv::imread(imgSrc, 0); }

/**
 * @brief Correction   畸变校正
 * @param srcImage     原始图像
 * @param dstImage     结果图像
 * @param cameraMatrix 内参矩阵
 * @param distCoeffs   畸变系数
 */
void CameraAndLaserPlaneCalibration::Correction(cv::Mat& srcImage, cv::Mat& dstImage, cv::Mat& cameraMatrix,
                                                cv::Mat& distCoeffs) {
    // distCoeffs.at<double>(0, 4) = 0;
    undistort(srcImage, dstImage, cameraMatrix, distCoeffs);
}

/**
 * @brief RemoveSmallRegion 去除面积小于给定值的连通域
 * @param InputImage        输入图像
 * @param OutputImage       输出图像
 * @param pixel             像素值
 */
void CameraAndLaserPlaneCalibration::RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage, int pixel) {
    InputImage.copyTo(OutputImage);
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(InputImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // 计算每个轮廓的面积
    double temp = 0;
    for (int i = 0; i < contours.size(); ++i) {
        temp = fabs(cv::contourArea(contours[i]));
    }

    // 去除小轮廓，保留大轮廓
    for (int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
        if (fabs(cv::contourArea(contours[idx])) < pixel) {
            cv::drawContours(OutputImage, contours, idx, cv::Scalar(0), cv::FILLED, 8, hierarchy);
        }
    }
}

/**
 * @brief CameraAndLaserPlaneCalibration::PointSortRule 对点的坐标按 x从小到大排序，若 x值相同则按 y从小到大排序
 * @param pt1 第一个点
 * @param pt2 第二个点
 * @return    第一个点小，则返回 1，反之返回 0
 */
bool CameraAndLaserPlaneCalibration::PointSortRule(const cv::Point2d pt1, const cv::Point2d pt2) {
    if (pt1.x != pt2.x) {
        return pt1.x < pt2.x;
    } else {
        return pt1.y < pt2.y;
    }
}

/**
 * @brief Point2dSperate 分离坐标点
 * @param P              中心点坐标值
 * @param pPlane         平面上的中心点
 * @param pObject        物体上的中心点
 */
void CameraAndLaserPlaneCalibration::Point2dSperate(std::vector<cv::Point2d>& P, std::vector<cv::Point2d>& pPlane,
                                                    std::vector<cv::Point2d>& pObject) {
    pPlane.clear();
    pObject.clear();

    int objectBegin1, objectEnd1;
    for (int i = 1; i < P.size(); ++i) {
        if (P[i - 1].y - P[i].y > 10) objectBegin1 = i;
        if (P[i].y - P[i - 1].y > 10) objectEnd1 = i;
    }

    int objectBegin2, objectEnd2;
    for (int i = 1; i < P.size(); ++i) {
        if (P[i - 1].y - P[i].y > 10) {
            objectBegin2 = i;
            break;
        }
    }
    for (int i = objectBegin1; i < P.size(); ++i) {
        if (P[i].y - P[i - 1].y > 10) {
            objectEnd2 = i;
            break;
        }
    }
    // std::cout << object_begin1 << " " << object_begin2 << " " << object_end1
    // << " " << object_end2 << std::endl;

    for (int i = 0; i < objectBegin2; ++i) pPlane.push_back(P[i]);
    for (int i = objectEnd1; i < P.size(); ++i) pPlane.push_back(P[i]);
    for (int i = objectBegin1; i < objectEnd2; ++i) pObject.push_back(P[i]);
}

/**
 * @brief Point2dto3d  二维像素坐标转三维图像坐标
 * @param plane        平面方程系数
 * @param cameraMatrix 内参矩阵
 * @param distCoeffs   畸变系数
 * @param Pt2ds        二维点集
 * @param Pt3ds        三维点集
 */
void CameraAndLaserPlaneCalibration::Point2dto3d(const std::vector<double> plane, const cv::Mat& cameraMatrix,
                                                 const cv::Mat& distCoeffs, const std::vector<cv::Point2d>& Pt2ds,
                                                 std::vector<cv::Point3d>& Pt3ds) {
    if (plane.size() < 4) {
        std::cerr << "Error: plane has fewer than 4 coefficients!" << std::endl;
        return;
    }
    if (std::abs(plane[3]) < 1e-12) {
        std::cerr << "Error: plane[3] is too close to zero!" << std::endl;
        return;
    }
    // Q_UNUSED(distCoeffs)
    double A = -(plane[0] / plane[3]), B = -(plane[1] / plane[3]), C = -(plane[2] / plane[3]);
    double u0 = cameraMatrix.at<double>(0, 2), v0 = cameraMatrix.at<double>(1, 2);  // 相机主点
    double fx = cameraMatrix.at<double>(0, 0), fy = cameraMatrix.at<double>(1, 1);  // 尺度因子

    //------------------------像素点去畸变后在相机坐标系下的归一化坐标---------------------

    std::vector<cv::Point2d> undistortedPts;
    // 使用cv::undistortPoints进行去畸变
    cv::undistortPoints(Pt2ds, undistortedPts, cameraMatrix, distCoeffs);
    // for (const auto& pt : undistortedPts) {
    //     std::cout << "Undistorted Point: (" << pt.x << ", " << pt.y << ")" << std::endl;
    // }
    for (int i = 0; i < undistortedPts.size(); ++i) {
        double x1 = undistortedPts[i].x, y1 = undistortedPts[i].y;

        //----------------------------像素点不去畸变情况下的运算------------------------------
        // for (int i = 0; i < Pt2ds.size(); ++i) {
        //     double u = Pt2ds[i].x, v = Pt2ds[i].y;
        //     /*
        //      * 图像坐标系转换到相机坐标系，得到的是归一化坐标值，也就是在 z = 1
        //      * 平面上的投影点， 所以求得的相机坐标系下的点其实是(x1, y1, 1)，
        //      * 那么由原点通过点(x1, y1, 1)的直线方程为 x/x1=y/y1=z/1，
        //      * 其中任一点x坐标满足 x=x1*z，y坐标满足y=y1*z。
        //      * 带入平面方程 Ax + By + Cz - 1 = 0可得：A*x1*z + B*y1*z + Cz - 1 = 0，
        //      * 即(A*x1 + B*y1 + C)*z - 1 = 0，所以 z = 1 / (A*x1 + B*y1 + C)，
        //      * 根据相似三角形，放缩比例为 z1 / 1 = z，
        //      *
        //      * 所以平面上的点的x和y坐标为x1*z1和y1*z1，推导完成。
        //      */
        //     double x1 = ((u - u0) / fx), y1 = ((v - v0) / fy);

        // std::cout<<  " x1 "<<x1 << "y1 " << y1 << " " << std::endl;
        cv::Point3d pt;
        pt.z = (1 / (A * x1 + B * y1 + C));
        pt.x = x1 * pt.z;
        pt.y = y1 * pt.z;
        Pt3ds.push_back(pt);
        // fout << std::setprecision(16) << pt.x << " " << pt.y << " " << pt.z << std::endl;
        // std::cout<< pt.x << " " << pt.y << " " << pt.z << std::endl;
    }
    // fout.close();
}

/**
 * @brief PointtoPlaneEvaluation 点到面精度评价
 * @param Pt3ds                  三维点集
 * @param plane                  平面方程系数
 */
void CameraAndLaserPlaneCalibration::PointtoPlaneEvaluation(const std::vector<cv::Point3d>& Pt3ds, std::vector<double> plane) {
    double a = plane[0], b = plane[1], c = plane[2];  // 平面方程系数
    std::vector<double> distance;                     // 存储每个点到平面的距离

    // 计算所有点到平面的平均距离
    double distanceMean = 0;
    for (int i = 0; i < Pt3ds.size(); ++i) {
        // 计算点到平面的距离公式：|ax + by + cz - 1| / sqrt(a^2 + b^2 + c^2)
        double dis = abs(a * Pt3ds[i].x + b * Pt3ds[i].y + c * Pt3ds[i].z - 1) / sqrt(a * a + b * b + c * c);
        distance.push_back(dis);
        distanceMean += dis;
    }
    distanceMean /= Pt3ds.size();  // 计算平均距离
    std::cout << "\n距离平均值：" << distanceMean << std::endl;

    // 计算距离的标准差
    double sigma = 0;
    for (int i = 0; i < Pt3ds.size(); ++i) {
        sigma += (distance[i] - distanceMean) * (distance[i] - distanceMean);
    }
    sigma /= Pt3ds.size();
    std::cout << "距离标准差：" << sqrt(sigma) << std::endl;
}
// 相机坐标系下的平面方程求解
void CameraAndLaserPlaneCalibration::computePlaneEquations(std::vector<cv::Mat>& extrinsicMatrices,
                                                           std::vector<cv::Point3f>& objectPoints,
                                                           std::vector<double>& globalPlane) {
    // 遍历每张图像
    std::vector<cv::Point3d> allPoints;  // 存储所有的相机坐标系下的点
    std::vector<std::vector<double>> planeEquations;
    std::vector<std::vector<cv::Point3d>> allCameraCoordinates;  // 存储所有点

    for (int i = 0; i < extrinsicMatrices.size(); i++) {  // extrinsicMatrices.size()

        // // 添加边界检查
        // if (i >= objectPoints.size() || objectPoints[i].empty()) {
        //     std::cerr << "Error: objectPoints[" << i << "] is out of range or empty!" << std::endl;
        //     continue;
        // }
        // 获取外参矩阵（旋转矩阵 + 平移向量）
        cv::Mat extrinsicMatrix = extrinsicMatrices[i];
        cv::Mat rotationMatrix = extrinsicMatrix(cv::Rect(0, 0, 3, 3));     // 3x3旋转矩阵
        cv::Mat translationVector = extrinsicMatrix(cv::Rect(3, 0, 1, 3));  // 3x1平移向量

        // 转换为相机坐标系下的点
        std::vector<cv::Point3d> cameraCoordinates;  // 相机坐标系下的点
        for (auto& point : objectPoints) {
            // 将世界坐标点转换为相机坐标系下的点
            cv::Mat worldPoint = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
            cv::Mat cameraPoint = rotationMatrix * worldPoint + translationVector;
            // std::cout << "//worldPoint: \n" << worldPoint << std::endl;
            // std::cout << "//rotationMatrix: \n" << rotationMatrix << std::endl;
            // std::cout << "//tvecsMat: \n" << translationVector << std::endl;
            // std::cout << "//cameraPoint: \n" << cameraPoint << std::endl;
            allPoints.push_back(cv::Point3d(cameraPoint.at<double>(0), cameraPoint.at<double>(1), cameraPoint.at<double>(2)));
            cameraCoordinates.push_back(
                cv::Point3d(cameraPoint.at<double>(0), cameraPoint.at<double>(1), cameraPoint.at<double>(2)));
        }
        // 存储相机坐标系下的标定板的点
        allCameraCoordinates.push_back(cameraCoordinates);
    }
    globalPlane = planeLeastSquareFitting(allPoints);

    // 获取平面方程系数
    // std::vector<double> planeEquation = planeLeastSquareFitting(cameraCoordinates);
    // planeEquations.push_back(planeEquation);

    // //打印平面方程系数
    // std::cout << "plane equation in camera coordinates: "
    //           << planeEquation[0] << " * X + " << planeEquation[1] << " * Y + " << planeEquation[2] << " * Z + " <<
    //           planeEquation[3]
    //           << " = 0" << std::endl;
    //}
    // globalPlane = fitPlaneToMultipleEquations(planeEquations);//这个是系数平均拟合
    // 对第一张图相机转世界（检查矩阵运算正确性，其应与worldPoint一样）
    // std::vector<cv::Point3d> basePoints = transformCameraToBase(allCameraCoordinates[0], extrinsicMatrices[0]);
    // for (const auto& pt : basePoints) {

    //     std::cout <<"原来反向拟合"<< "(" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
    // }
    // std::cout << "globalPlane equation in camera coordinates: "
    //            << globalPlane[0] << " * X + " << globalPlane[1] << " * Y + " << globalPlane[2] << " * Z + " << globalPlane[3]
    //            << " = 0" << std::endl;
    // for (const auto& cameraCoordinates : allCameraCoordinates) {
    //     PointtoPlaneEvaluation(cameraCoordinates, globalPlane);  // 直接调用，输出误差
    // }

    std::string filename = resolvePlanePointCloudPath();
    savePointCloud(allCameraCoordinates, filename);
}
/**
 * @brief findPlane 最小二乘法拟合平面
 * @param pts       输入三维点集
 * @return          平面方程系数
 */
std::vector<double> CameraAndLaserPlaneCalibration::planeLeastSquareFitting(std::vector<cv::Point3d>& pts) {
    // 最小二乘法
    double A, B, C, D;
    std::vector<double> parameters;
    double meanX = 0, meanY = 0, meanZ = 0;
    double meanXX = 0, meanYY = 0, meanZZ = 0;
    double meanXY = 0, meanXZ = 0, meanYZ = 0;

    for (int i = 0; i < pts.size(); ++i) {
        meanX += pts[i].x;
        meanY += pts[i].y;
        meanZ += pts[i].z;

        meanXX += pts[i].x * pts[i].x;
        meanYY += pts[i].y * pts[i].y;
        meanZZ += pts[i].z * pts[i].z;

        meanXY += pts[i].x * pts[i].y;
        meanXZ += pts[i].x * pts[i].z;
        meanYZ += pts[i].y * pts[i].z;
    }
    meanX /= pts.size();
    meanY /= pts.size();
    meanZ /= pts.size();
    meanXX /= pts.size();
    meanYY /= pts.size();
    meanZZ /= pts.size();
    meanXY /= pts.size();
    meanXZ /= pts.size();
    meanYZ /= pts.size();

    /* eigenvector */
    Eigen::Matrix3d eMat;
    eMat(0, 0) = meanXX - meanX * meanX;
    eMat(0, 1) = meanXY - meanX * meanY;
    eMat(0, 2) = meanXZ - meanX * meanZ;
    eMat(1, 0) = meanXY - meanX * meanY;
    eMat(1, 1) = meanYY - meanY * meanY;
    eMat(1, 2) = meanYZ - meanY * meanZ;
    eMat(2, 0) = meanXZ - meanX * meanZ;
    eMat(2, 1) = meanYZ - meanY * meanZ;
    eMat(2, 2) = meanZZ - meanZ * meanZ;
    Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
    // 得到协方差矩阵的特征值和特征向量
    Eigen::Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
    Eigen::Matrix3d eVector = xjMat.pseudoEigenvectors();
    // 协方差矩阵的特征向量可以被看作是数据集中方差最小（或最不重要）的方向，而对应的特征值则表示了这个方向上的方差大小。
    // 因此，最小特征值对应的特征向量通常被认为是数据集中变化最小的方向，即是拟合平面的法向量。

    /* the eigenvector corresponding to the minimum eigenvalue */
    double v1 = eValue(0, 0);
    double v2 = eValue(1, 1);
    double v3 = eValue(2, 2);
    int minNumber = 0;
    if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3))) {
        minNumber = 1;
    }
    if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2))) {
        minNumber = 2;
    }
    A = eVector(0, minNumber);
    B = eVector(1, minNumber);
    C = eVector(2, minNumber);
    // D = -(A * meanX + B * meanY + C * meanZ);
    D = -(A * meanX + B * meanY + C * meanZ);

    /* result */
    if (C < 0) {
        A *= -1.0;
        B *= -1.0;
        C *= -1.0;
        D *= -1.0;
    }

    // parameters.push_back(-A / D);
    // parameters.push_back(-B / D);
    // parameters.push_back(-C / D);
    parameters.push_back(A);
    parameters.push_back(B);
    parameters.push_back(C);
    parameters.push_back(D);
    return parameters;
}

cv::Vec4d CameraAndLaserPlaneCalibration::fitPlaneToPoints(const std::vector<cv::Point3d>& points) {
    // 将点集转换为OpenCV的Mat格式
    cv::Mat pointsMat(points.size(), 3, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        pointsMat.at<double>(i, 0) = points[i].x;  // 设置x坐标
        pointsMat.at<double>(i, 1) = points[i].y;  // 设置y坐标
        pointsMat.at<double>(i, 2) = points[i].z;  // 设置z坐标
    }

    // 执行PCA（主成分分析）
    cv::PCA pca(pointsMat, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // 平面的法向量是最后一个主成分
    cv::Vec3d normal = pca.eigenvectors.row(2);

    // 计算点集的质心
    cv::Point3d centroid(pca.mean.at<double>(0, 0), pca.mean.at<double>(0, 1), pca.mean.at<double>(0, 2));

    // 平面方程为：ax + by + cz + d = 0
    double d = -normal.dot(centroid);  // 计算d值

    // 返回平面方程的参数 (a, b, c, d)
    return cv::Vec4d(normal[0], normal[1], normal[2], d);
}
/**
 * @brief findPlane 方向向量拟合平面
 * @param plane     世界坐标系下的平面方程系数
 * @return          相机坐标系下平面方程系数
 */
cv::Vec4f CameraAndLaserPlaneCalibration::calculatePlaneSquareWithVecs(std::vector<double> plane, cv::Mat& extrinsicMatrix) {
    cv::Mat rotationMatrix = extrinsicMatrix(cv::Rect(0, 0, 3, 3));  // 3x3旋转矩阵
    cv::Mat normal_w = (cv::Mat_<double>(3, 1) << plane[0], plane[1], plane[2]);
    cv::Mat translationVector = extrinsicMatrix(cv::Rect(3, 0, 1, 3));  // 3x1平移向量

    // 将法向量变换到相机坐标系
    cv::Mat normal_c = rotationMatrix * normal_w;
    // 获取平移向量 t
    cv::Mat t = translationVector;
    // 计算平面方程的常数项 D_c = -(n_c^T * t+D)
    double D_c = -(normal_c.at<double>(0, 0) * t.at<double>(0, 0) + normal_c.at<double>(1, 0) * t.at<double>(1, 0) +
                   normal_c.at<double>(2, 0) * t.at<double>(2, 0) + plane[3]);
    // 保存相机坐标系下的平面方程系数
    cv::Vec4f planeCoeff(normal_c.at<double>(0, 0), normal_c.at<double>(1, 0), normal_c.at<double>(2, 0), D_c);
    return planeCoeff;
}
// 求取外参矩阵
void CameraAndLaserPlaneCalibration::calculateExtrinsicMatrices(int imageCount, std::vector<cv::Mat>& tvecsMat,
                                                                std::vector<cv::Mat>& rvecsMat,
                                                                std::vector<cv::Mat>& extrinsicMatrices) {
    extrinsicMatrices.clear();
    // 保存定标结果
    for (int i = 0; i < imageCount; i++) {
        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));   // 保存每幅图像的旋转矩阵
        cv::Mat extrinsicMatrix = cv::Mat(3, 4, CV_32FC1, cv::Scalar::all(0));  //
        /* 将旋转向量转换为相对应的旋转矩阵 */
        // std::cout << "第 " << i << " 幅图像的旋转向量(rvec):\n" << rvecsMat[i] << std::endl;
        // std::cout << "第 " << i << " 幅图像的平移向量(tvec):\n" << tvecsMat[i] << std::endl;
        cv::Rodrigues(rvecsMat[i], rotationMatrix);
        /*rvecsMat[i],3*1，方向为轴，模长为弧度。
         *使用罗德里格斯（Rodrigues）公式，将 旋转向量（3×1）与 旋转矩阵（3×3）相互转换。
         */
        cv::hconcat(rotationMatrix, tvecsMat[i], extrinsicMatrix);
        extrinsicMatrices.push_back(extrinsicMatrix);
        // std::cout << "第 " << i << " 幅图像的外参矩阵:\n" << extrinsicMatrix << std::endl;
    }
    // std::cout << "\n所有外参矩阵:\n";
    // for (const auto& pt : extrinsicMatrices) {
    //     std::cout << "extrinsicMatrices: \n" << pt << std::endl;
    // }
    // 拟合多个平面方程并计算误差
    // cv::Vec4f fittedPlaneCoeff;
    // double meanError;
    // fitPlaneToMultipleEquations(planeCoefficients, fittedPlaneCoeff, meanError);
}

std::vector<double> CameraAndLaserPlaneCalibration::fitPlaneToMultipleEquations(const std::vector<std::vector<double>>& planes) {
    double A = 0, B = 0, C = 0, D = 0;

    // 计算平均法向量
    for (const auto& plane : planes) {
        A += plane[0];
        B += plane[1];
        C += plane[2];
    }
    A /= planes.size();
    B /= planes.size();
    C /= planes.size();

    // 归一化
    double norm = sqrt(A * A + B * B + C * C);
    A /= norm;
    B /= norm;
    C /= norm;

    // 计算9个平面方程的重心，用于计算D
    double sumX = 0, sumY = 0, sumZ = 0;
    for (const auto& plane : planes) {
        sumX += -plane[3] * plane[0];
        sumY += -plane[3] * plane[1];
        sumZ += -plane[3] * plane[2];
    }
    sumX /= planes.size();
    sumY /= planes.size();
    sumZ /= planes.size();

    // 计算新的D
    D = -(A * sumX + B * sumY + C * sumZ);
    if (C < 0) {
        A *= -1.0;
        B *= -1.0;
        C *= -1.0;
        D *= -1.0;
    }

    return {A, B, C, D};
}
/**
 * @brief 将相机坐标系下的点转换到世界座标系
 * @param cameraPoints 相机坐标系下的点集
 * @param extrinsicMatrix 相机外参矩阵
 * @return 基座标系下的点集
 */
std::vector<cv::Point3d> CameraAndLaserPlaneCalibration::transformCameraToBase(const std::vector<cv::Point3d>& cameraPoints,
                                                                               const cv::Mat& extrinsicMatrix) {
    std::vector<cv::Point3d> basePoints;

    // 从外参矩阵中提取旋转矩阵和平移向量
    cv::Mat rotationMatrix = extrinsicMatrix(cv::Rect(0, 0, 3, 3));     // 3x3旋转矩阵
    cv::Mat translationVector = extrinsicMatrix(cv::Rect(3, 0, 1, 3));  // 3x1平移向量

    // 对每个点进行转换
    for (const auto& cameraPoint : cameraPoints) {
        // 将点转换为Mat格式
        cv::Mat cameraPointMat = (cv::Mat_<double>(3, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z);

        // 转换公式：basePoint = R^T * (cameraPoint - t)
        cv::Mat basePointMat = rotationMatrix.t() * (cameraPointMat - translationVector);

        // 将结果转换回Point3d格式
        basePoints.push_back(cv::Point3d(basePointMat.at<double>(0), basePointMat.at<double>(1), basePointMat.at<double>(2)));
    }

    return basePoints;
}
// 保存点云
void CameraAndLaserPlaneCalibration::savePointCloud(const std::vector<std::vector<cv::Point3d>>& objectPoints,
                                                    const std::string& filename) {
    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.emplace_back(0.0f, 0.0f, 0.0f);  // 原点
        // 遍历 objectPoints 并存入点云
        for (const auto& points : objectPoints) {
            for (const auto& pt : points) {
                cloud->points.emplace_back(pt.x, pt.y, pt.z);
            }
        }

        // 设置点云参数
        cloud->width = cloud->points.size();
        cloud->height = 1;  // 1 表示无序点云
        cloud->is_dense = false;

        createDirectoryRecursive(parentDirectoryPath(filename));

        int saveResult = -1;
        if (hasSuffix(filename, ".pcd")) {
            saveResult = pcl::io::savePCDFileASCII(filename, *cloud);
        } else if (hasSuffix(filename, ".ply")) {
            saveResult = pcl::io::savePLYFileASCII(filename, *cloud);
        } else {
            std::cerr << "Unsupported file format: " << filename << std::endl;
            return;
        }

        if (saveResult == 0) {
            std::cout << "Saved " << cloud->points.size() << " points to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save point cloud: " << filename << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "savePointCloud exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "savePointCloud exception: unknown error" << std::endl;
    }
}

CameraAndLaserPlaneCalibration::ErrorMetrics CameraAndLaserPlaneCalibration::evaluatePlaneFittingError(
    const std::vector<std::vector<cv::Point2f>>& imageCornerPoints, const std::vector<cv::Point3f>& objectCornerPoints,
    const std::vector<cv::Mat>& extrinsicMatrices, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const std::vector<double>& globalPlane) {
    double totalErrorX = 0.0, totalErrorY = 0.0, totalErrorZ = 0.0, totalError = 0.0;
    int totalPoints = 0;

    // 遍历每张图像
    for (size_t i = 0; i < imageCornerPoints.size(); i++) {
        if (i >= extrinsicMatrices.size()) {
            std::cerr << "Warning: extrinsicMatrices size (" << extrinsicMatrices.size()
                      << ") < imageCornerPoints size (" << imageCornerPoints.size() << "), skipping image " << i << std::endl;
            continue;
        }
        std::vector<cv::Point3d> camera3DPoints;
        std::vector<cv::Point3d> world3DPoints;
        // 2D → 相机 3D 坐标
        std::vector<cv::Point2d> imagePoints2d(imageCornerPoints[i].begin(), imageCornerPoints[i].end());
        Point2dto3d(globalPlane, cameraMatrix, distCoeffs, imagePoints2d, camera3DPoints);

        // 相机 3D → 世界 3D 坐标
        world3DPoints = transformCameraToBase(camera3DPoints, extrinsicMatrices[i]);

        // 计算误差
        size_t numPoints = std::min(world3DPoints.size(), objectCornerPoints.size());
        for (size_t j = 0; j < numPoints; j++) {
            double errorX = std::abs(world3DPoints[j].x - objectCornerPoints[j].x);
            double errorY = std::abs(world3DPoints[j].y - objectCornerPoints[j].y);
            double errorZ = std::abs(world3DPoints[j].z - objectCornerPoints[j].z);
            double errorTotal = std::sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ);

            totalErrorX += errorX;
            totalErrorY += errorY;
            totalErrorZ += errorZ;
            totalError += errorTotal;
            totalPoints++;
        }
    }

    if (totalPoints == 0) return {0, 0, 0, 0};  // 避免除零错误

    // 计算平均误差
    return {totalErrorX / totalPoints, totalErrorY / totalPoints, totalErrorZ / totalPoints, totalError / totalPoints};
}
bool CameraAndLaserPlaneCalibration::calculate_Image_Points(cv::Mat imageInput, cv::Size boardSize,
                                                            std::vector<cv::Point2f>& imagePoints) {
    cv::Mat viewGray;
    std::vector<cv::Point2f> pointbuf;
    cvtColor(imageInput, viewGray, cv::COLOR_RGB2GRAY);

    // 实际标定图片，应灰度翻转
    for (int row = 0; row < viewGray.rows; row++) {
        for (int col = 0; col < viewGray.cols; col++) {
            viewGray.at<uchar>(row, col) = 255 - viewGray.at<uchar>(row, col);  // 灰度反转
        }
    }

    //// Blob算子参数
    cv::SimpleBlobDetector::Params params;
    // params.filterByArea = true;
    params.maxArea = 3000;  // 10e4
    params.minArea = 300;    // 30
    params.minDistBetweenBlobs = 10;
    // params.minThreshold = 10;   //默认50
    // params.maxThreshold = 250;  //默认220
    params.filterByInertia = true;  // 斑点惯性率的限制变量  短轴/长轴
    params.minInertiaRatio = 0.5f;  // 斑点的最小惯性率;

    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

    bool found = false;
    found = findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING,
                            blobDetector);  // cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    if (found) {
        imagePoints = pointbuf;
    } else {
        std::cout << "当前图片找圆心出现错误" << std::endl;
        return false;
    }
    return true;
}
// bool CameraAndLaserPlaneCalibration::calculate_Image_Points2(cv::Mat imageInput, cv::Size boardSize,
//                                                              std::vector<cv::Point2f>& imagePoints) {
//     cv::Mat gray;
//     cv::cvtColor(imageInput, gray, cv::COLOR_BGR2GRAY);
//     cv::bitwise_not(gray, gray);

//     cv::SimpleBlobDetector::Params params;
//     params.minArea = 1e4;
//     params.maxArea = 3e5;
//     params.minCircularity = 0.7f;
//     params.filterByCircularity = true;
//     params.filterByColor = false;
//     params.filterByConvexity = false;
//     params.filterByInertia = false;

//     cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

//     std::vector<cv::KeyPoint> keypoints;
//     blobDetector->detect(gray, keypoints);

//     // 生成当前时间字符串（格式：YYYYMMDD_HHMMSS）
//     std::time_t t = std::time(nullptr);
//     std::tm tm;
// #ifdef _WIN32
//     localtime_s(&tm, &t);
// #else
//     localtime_r(&t, &tm);
// #endif
//     std::ostringstream oss;
//     oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
//     std::string timestamp = oss.str();

//     // 保存 Blob 检测结果
//     cv::Mat blobView;
//     cv::drawKeypoints(gray, keypoints, blobView, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//     std::string blobFile = "Detected_Blobs_" + timestamp + ".png";
//     cv::imwrite(blobFile, blobView);

//     // 棋盘圆点检测
//     bool found =
//         cv::findCirclesGrid(gray, boardSize, imagePoints, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);

//     if (!found) {
//         std::cout << "未检测到圆心，结果图已保存为: " << blobFile << std::endl;
//         return false;
//     }

//     // 保存圆心检测结果
//     cv::Mat drawImg = imageInput.clone();
//     cv::drawChessboardCorners(drawImg, boardSize, imagePoints, found);
//     std::string gridFile = "Detected_Grid_" + timestamp + ".png";
//     cv::imwrite(gridFile, drawImg);

//     std::cout << "检测完成，结果文件已保存：\n"
//               << "  - Blob检测结果: " << blobFile << "\n"
//               << "  - 圆心检测结果: " << gridFile << std::endl;
//     return true;
// }
bool CameraAndLaserPlaneCalibration::calculate_Image_Points2(cv::Mat imageInput, cv::Size boardSize,
                                                             std::vector<cv::Point2f>& imagePoints) {
    // ---------- 1. 灰度化与反色 ----------
    cv::Mat gray;
    if (imageInput.channels() == 3)
        cv::cvtColor(imageInput, gray, cv::COLOR_BGR2GRAY);
    else
        gray = imageInput.clone();
    cv::bitwise_not(gray, gray);  // 若圆为黑底白点可省略

    // ---------- 2. Blob 检测参数 ----------
    cv::SimpleBlobDetector::Params params;
    params.minArea = 1e5;
    params.maxArea = 8e5;
    params.minCircularity = 0.7f;
    params.filterByCircularity = true;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

    // ---------- 3. 时间戳 ----------
    std::time_t t = std::time(nullptr);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string timestamp = oss.str();

    // ---------- 4. 圆阵检测 ----------
    bool found =
        cv::findCirclesGrid(gray, boardSize, imagePoints, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);

    if (!found) {
        std::cout << "未检测到圆心，结果图已保存为: " << std::endl;
        // 仍按原逻辑提示，但不保存图（保持与你原代码一致的行为）
        return false;
    }

    // ---------- 5. 亚像素精确化 ----------
    cv::Size winSize(10, 10);
    cv::Size zeroZone(1, 1);
    cv::TermCriteria tc(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-3);
    cv::cornerSubPix(gray, imagePoints, winSize, zeroZone, tc);

    // ---------- 6. 点序规范（保持最终为：上→下、左→右，行优先） ----------
    const int W = boardSize.width;
    const int H = boardSize.height;
    if (imagePoints.size() == static_cast<size_t>(W * H)) {
        auto mean_row_y = [&](int r, const std::vector<cv::Point2f>& P) -> double {
            double s = 0.0;
            for (int c = 0; c < W; ++c) s += P[r * W + c].y;
            return s / W;
        };
        auto row_y_variance_sum = [&](const std::vector<cv::Point2f>& P) -> double {
            double total = 0.0;
            for (int r = 0; r < H; ++r) {
                double mu = mean_row_y(r, P);
                double var = 0.0;
                for (int c = 0; c < W; ++c) {
                    double dy = P[r * W + c].y - mu;
                    var += dy * dy;
                }
                total += var / W;
            }
            return total;
        };

        // 6.1 列优先 → 行优先（必要时做转置）
        double rowVar_A = row_y_variance_sum(imagePoints);
        std::vector<cv::Point2f> transposed(imagePoints.size());
        for (int r = 0; r < H; ++r)
            for (int c = 0; c < W; ++c) transposed[r * W + c] = imagePoints[c * H + r];
        double rowVar_B = row_y_variance_sum(transposed);
        if (rowVar_B + 1e-6 < rowVar_A) {
            imagePoints.swap(transposed);
            std::cout << " 检测到列优先 → 已转置为行优先\n";
        }

        // 6.2 每行保证 x 递增（左→右）
        for (int r = 0; r < H; ++r) {
            int idx = r * W;
            int inc = 0;
            for (int c = 0; c < W - 1; ++c)
                if (imagePoints[idx + c + 1].x > imagePoints[idx + c].x) ++inc;
            if (inc < (W - 1) / 2) {  // 多数递减 → 镜像一行
                for (int c = 0; c < W / 2; ++c) std::swap(imagePoints[idx + c], imagePoints[idx + (W - 1 - c)]);
                std::cout << " 行 " << r << " 发生镜像，已纠正为 x 递增\n";
            }
        }

        // 6.3 整体保证第一行为“更靠上”（上→下）
        double mean_top_y = 0.0, mean_bottom_y = 0.0;
        for (int c = 0; c < W; ++c) {
            mean_top_y += imagePoints[c].y;
            mean_bottom_y += imagePoints[(H - 1) * W + c].y;
        }
        mean_top_y /= W;
        mean_bottom_y /= W;
        if (mean_top_y > mean_bottom_y) {
            // 反转所有行的顺序（上下翻转）
            for (int r = 0; r < H / 2; ++r) {
                for (int c = 0; c < W; ++c) {
                    std::swap(imagePoints[r * W + c], imagePoints[(H - 1 - r) * W + c]);
                }
            }
            std::cout << " 整体做了上下翻转，统一为上→下\n";
        }
    }

    // ---------- 7. 绘制结果（保存路径与命名按你的原程序） ----------
    return true;
}

// 已知内参求外参
void CameraAndLaserPlaneCalibration::calibrationSolveExtrinsics(cv::Mat& Kc, cv::Mat& distCoeffs,
                                                                std::vector<cv::Point3f>& objPoints,
                                                                std::vector<std::vector<cv::Point2f>>& imagePoints,
                                                                std::vector<cv::Mat>& vecHc) {
    std::vector<double> camera_distortion(distCoeffs.begin<double>(), distCoeffs.end<double>());
    for (int i = 0; i < imagePoints.size(); i++) {
        if (imagePoints[i].empty()) {
            continue;
        }
        // 创建旋转矩阵和平移矩阵
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        if (!cv::solvePnP(objPoints, imagePoints[i], Kc, camera_distortion, rvec, tvec)) {
            continue;
        }
        cv::Mat rotM = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(rvec, rotM);  // 将旋转向量变换成旋转矩阵
        cv::Mat RT_Mat_temp;
        hconcat(rotM, tvec, RT_Mat_temp);
        cv::Mat last_line = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);  // 齐次矩阵最后一行
        cv::Mat RT_Mat;
        cv::vconcat(RT_Mat_temp, last_line, RT_Mat);  // 输出外参矩阵
        vecHc.push_back(RT_Mat);
    }
}

