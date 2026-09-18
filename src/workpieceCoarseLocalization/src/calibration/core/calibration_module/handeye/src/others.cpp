
#include "../include/others.h"

std::vector<double> vecWorldX, vecWorldY, vecWorldZ;  // 标定板在机械臂基坐标系下的坐标
void calc_stdev(vector<double> &vecWorld, double &stdev, double &mean) {
    double sum = std::accumulate(std::begin(vecWorld), std::end(vecWorld), 0.0);
    mean = sum / vecWorld.size();  // 均值
    double accum = 0.0;
    std::for_each(std::begin(vecWorld), std::end(vecWorld), [&](const double d) { accum += (d - mean) * (d - mean); });
    stdev = sqrt(accum / (vecWorld.size() - 1));  // 标准差
}
// 将 RT 拆分为旋转矩阵 R 和平移向量 T
void RT2R_T(cv::Mat &RT, cv::Mat &R, cv::Mat &T) {
    cv::Rect R_rect(0, 0, 3, 3);
    cv::Rect T_rect(3, 0, 1, 3);
    R = RT(R_rect);
    T = RT(T_rect);
}
// 将旋转矩阵 R 和平移向量 T 组合为 RT
cv::Mat R_T2RT(cv::Mat &R, cv::Mat &T) {
    cv::Mat RT;
    cv::Mat_<double> R1 =
        (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1),
         R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), 0.0, 0.0, 0.0);
    cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

    cv::hconcat(R1, T1, RT);
    return RT;
}

// 根据标定板信息生成标定板坐标点
void calculate_Object_Points(int board_width, int board_heignt, double circle_distance, vector<cv::Point3f> &objP) {
    for (int i = 0; i < board_heignt; i++) {
        for (int j = 0; j < board_width; j++) {
            objP.push_back(cv::Point3f(j * circle_distance, i * circle_distance, 0));
        }
    }
}
bool calculate_Image_Points_ChessboardCorners(const std::string &path, cv::Size boardSize,
                                              std::vector<std::vector<cv::Point2f>> &imagePoints) {
    // 使用 cv::glob 获取目录中所有 BMP 图片的文件路径
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);  // 根据路径和文件类型（.bmp）获取图片列表

    // 检查是否找到图片文件
    if (imageList.empty()) {
        std::cerr << "No images found in the directory!" << std::endl;
        return false;
    }

    int nframes = (int)imageList.size();
    int imageCount = 0;

    // 遍历所有图片
    for (int i = 0; i < nframes; i++) {
        cv::Mat view, viewGray;

        // 读取当前图片
        if (i < (int)imageList.size()) {
            std::cout << "Processing image: " << imageList[i] << std::endl;
            view = cv::imread(imageList[i], cv::IMREAD_COLOR);  // 读取彩色图片
        }

        if (view.empty()) {
            std::cout << "Could not open or find the image at " << imageList[i] << std::endl;
            continue;
        }

        std::vector<cv::Point2f> imagePointsBuf;

        // 检查图像是否已经是灰度图像
        if (view.channels() == 3) {
            cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);  // 彩色图像转换为灰度图像
        } else {
            viewGray = view;  // 已经是灰度图像，直接使用
        }

        // 使用 cv::findChessboardCornersSB 查找棋盘格角点
        if (cv::findChessboardCornersSB(viewGray, boardSize, imagePointsBuf)) {
            // 对角点进行亚像素级精确化
            cv::cornerSubPix(viewGray, imagePointsBuf, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

            // 统一角点顺序，确保方向一致
            if (imagePointsBuf[0].x > imagePointsBuf[imagePointsBuf.size() - 1].x) {
                std::vector<cv::Point2f> buf;
                for (int i = 0; i < imagePointsBuf.size(); i++) {
                    buf.push_back(imagePointsBuf[imagePointsBuf.size() - 1 - i]);
                }
                imagePointsBuf.clear();
                imagePointsBuf = buf;
            }

            // 可视化角点并标注序号
            int pointIndex = 0;
            for (int j = 0; j < imagePointsBuf.size(); j++) {
                cv::circle(view, cv::Point(imagePointsBuf[j].x, imagePointsBuf[j].y), 1, cv::Scalar(0, 0, 255), -1);
                cv::Point textPosition(imagePointsBuf[j].x, imagePointsBuf[j].y);
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 0.5;
                cv::Scalar fontColor(255, 0, 0);
                int fontThickness = 1;
                cv::putText(view, std::to_string(pointIndex++), textPosition, fontFace, fontScale, fontColor, fontThickness);
            }

            // 将角点保存到 imagePoints 中
            imagePoints.push_back(imagePointsBuf);
        } else {
            std::cout << "Num " << i << " can not find chessboard corners!\n";
        }

        // 记录已处理的图像数量
        imageCount++;
        if (imageCount == 1) {
            // 输出第一张图片的尺寸信息
            std::cout << "Image size: " << view.cols << "x" << view.rows << std::endl;
        }
    }

    // 所有图片处理完成
    return true;
}
// 读取目录中的图片，计算每张图片的圆心集合
bool calculate_Image_Points(std::string &path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>> &imagePoints) {
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);  // 圆点标定板图片为 BMP 格式
    if (imageList.size() == 0) {
        std::cout << "no images." << std::endl;
        return false;
    }
    int nframes = (int)imageList.size();

    for (int i = 0; i < nframes; i++) {
        cv::Mat view, viewGray;

        if (i < (int)imageList.size()) {
            std::cout << "image_file: " << imageList[i];
            view = imread(imageList[i], cv::IMREAD_COLOR);
        }
        std::cout << "\n";
        std::vector<cv::Point2f> pointbuf;
        cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(viewGray, viewGray);  // 反转灰度图像
        // 实际标定图片需要进行灰度反转
        for (int row = 0; row < viewGray.rows; row++) {
            for (int col = 0; col < viewGray.cols; col++) {
                viewGray.at<uchar>(row, col) = 255 - viewGray.at<uchar>(row, col);  // 灰度反转
            }
        }

        //// Blob 算子参数
        cv::SimpleBlobDetector::Params params;
        // params.filterByArea = true;
        params.maxArea = 10e4;  // 10e4
        params.minArea = 30;    // 30
        params.minDistBetweenBlobs = 10;
        // params.minThreshold = 10;   // 默认值为 50
        // params.maxThreshold = 250;  // 默认值为 220
        params.filterByInertia = true;  // 启用斑点惯性率限制（短轴/长轴）
        params.minInertiaRatio = 0.5f;  // 斑点的最小惯性率

        cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

        bool found = false;
        found = findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING,
                                blobDetector);  // cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
        if (found) {
            if (pointbuf[0].x > pointbuf[pointbuf.size() - 1].x) {
                std::vector<cv::Point2f> buf;
                for (int i = 0; i < pointbuf.size(); i++) {
                    buf.push_back(pointbuf[pointbuf.size() - 1 - i]);
                }
                pointbuf.clear();
                pointbuf = buf;
            }
            imagePoints.push_back(pointbuf);
        } else {
            std::cout << "Failed to find circle grid in current image." << std::endl;
            // 获取文件路径和文件名
            std::string newFileName = imageList[i];
            // 在文件扩展名前添加“_unfind”后缀
            size_t lastDot = newFileName.find_last_of(".");
            if (lastDot != std::string::npos) {
                newFileName.insert(lastDot, "_unfind");
            }

            // 重命名文件
            if (rename(imageList[i].c_str(), newFileName.c_str()) != 0) {
                std::cerr << "无法重命名文件：" << imageList[i] << std::endl;
            } else {
                // std::cout << "重命名为：" << newFileName << std::endl;
            }
        }
        // 可视化检测结果
        drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
    }
    return true;
}
// vector
bool calculate_Image_Points_V(std::vector<cv::Mat> &dirImages, cv::Size boardSize,
                              std::vector<std::vector<cv::Point2f>> &imagePoints) {
    int nframes = (int)dirImages.size();

    if (nframes == 0) {
        std::cout << "no images." << std::endl;
        return false;
    }

    for (int i = 0; i < nframes; i++) {
        cv::Mat view = dirImages[i];  // 直接从传入的 dirImages 中取出图像
        cv::Mat viewGray;

        std::cout << "Processing image " << i + 1 << " / " << nframes << std::endl;

        // 转换为灰度图像
        cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(viewGray, viewGray);  // 反转灰度图像
        // 灰度反转
        for (int row = 0; row < viewGray.rows; row++) {
            for (int col = 0; col < viewGray.cols; col++) {
                viewGray.at<uchar>(row, col) = 255 - viewGray.at<uchar>(row, col);  // 灰度反转
            }
        }

        // Blob 算子参数
        cv::SimpleBlobDetector::Params params;
        params.maxArea = 10e4;            // 设置最大面积
        params.minArea = 30;              // 设置最小面积
        params.minDistBetweenBlobs = 10;  // 设置斑点之间的最小距离
        params.filterByInertia = true;    // 启用斑点惯性率限制
        params.minInertiaRatio = 0.5f;    // 设置最小惯性率

        cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

        // 查找圆点标定板上的圆心
        std::vector<cv::Point2f> pointbuf;
        bool found;
        if (1) {
            found =
                findCirclesGrid(viewGray, boardSize, pointbuf, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetector);
        } else {
            found =
                findChessboardCornersSB(viewGray, boardSize, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        }
        if (found) {
            if (pointbuf[0].x > pointbuf[pointbuf.size() - 1].x) {
                std::vector<cv::Point2f> buf;
                for (int i = 0; i < pointbuf.size(); i++) {
                    buf.push_back(pointbuf[pointbuf.size() - 1 - i]);
                }
                pointbuf.clear();
                pointbuf = buf;
            }
            imagePoints.push_back(pointbuf);  // 找到圆心后存入 imagePoints
        } else {
            std::cout << "Failed to find circle grid in current image." << std::endl;
        }

        // 可视化检测结果
        drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
    }

    return true;
}

// 使用已知相机内参求解标定板外参
void Calibration_Solve_Extrinsics(cv::Mat &Kc, cv::Mat &distCoeffs, vector<cv::Point3f> &objPoints,
                                  std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<cv::Mat> &vecHc) {
    std::vector<double> camera_distortion(distCoeffs.begin<double>(), distCoeffs.end<double>());
    for (int i = 0; i < imagePoints.size(); i++) {
        // 创建旋转向量和平移向量
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::solvePnP(objPoints, imagePoints[i], Kc, camera_distortion, rvec, tvec);
        cv::Mat rotM = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(rvec, rotM);  // 将旋转向量转换为旋转矩阵
        cv::Mat RT_Mat_temp;
        hconcat(rotM, tvec, RT_Mat_temp);
        cv::Mat last_line = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);  // 齐次矩阵最后一行
        cv::Mat RT_Mat;
        cv::vconcat(RT_Mat_temp, last_line, RT_Mat);  // 输出外参矩阵
        vecHc.push_back(RT_Mat);
    }
}

void draw_line_chart(vector<double> &X_data, vector<double> &Y_data, string string_title) {
    const char *title = string_title.c_str();
    pcl::visualization::PCLPlotter *plot_(new pcl::visualization::PCLPlotter(title));
    plot_->setBackgroundColor(1, 1, 1);
    plot_->setTitle(title);
    plot_->setXTitle("X");
    plot_->setYTitle("Y");
    plot_->addPlotData(X_data, Y_data, "display", vtkChart::LINE);  // X、Y 均为 double 类型的向量
    plot_->plot();                                                  // 绘制曲线
}

void draw_line_chart_visualization(pcl::visualization::PCLPlotter *plot_, vector<double> &X_data, vector<double> &Y_data,
                                   string string_title) {
    const char *title = string_title.c_str();

    plot_->setBackgroundColor(1, 1, 1);
    plot_->setTitle(title);
    plot_->setXTitle("X");
    plot_->setYTitle("Y");
    plot_->addPlotData(X_data, Y_data, "display", vtkChart::LINE);  // X、Y 均为 double 类型的向量
    plot_->plot();                                                  // 绘制曲线

    while (!plot_->wasStopped()) {
        plot_->spinOnce(100);
    }
}
