
#include "../include/others.h"

std::vector<double> vecWorldX, vecWorldY, vecWorldZ;  // 鏍囧畾鏉垮湪鏈烘鑷傚熀鍧愭爣绯荤殑鍧愭爣
void calc_stdev(vector<double> &vecWorld, double &stdev, double &mean) {
    double sum = std::accumulate(std::begin(vecWorld), std::end(vecWorld), 0.0);
    mean = sum / vecWorld.size();  // 鍧囧€?
    double accum = 0.0;
    std::for_each(std::begin(vecWorld), std::end(vecWorld), [&](const double d) { accum += (d - mean) * (d - mean); });
    stdev = sqrt(accum / (vecWorld.size() - 1));  // 鏍囧噯宸?
}
// RT杞琑鍜孴  浠嶳T涓妸 R鍜孴鏁村嚭鏉?
void RT2R_T(cv::Mat &RT, cv::Mat &R, cv::Mat &T) {
    cv::Rect R_rect(0, 0, 3, 3);
    cv::Rect T_rect(3, 0, 1, 3);
    R = RT(R_rect);
    T = RT(T_rect);
}
// R鍜孴杞琑T
cv::Mat R_T2RT(cv::Mat &R, cv::Mat &T) {
    cv::Mat RT;
    cv::Mat_<double> R1 =
        (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1),
         R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), 0.0, 0.0, 0.0);
    cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

    cv::hconcat(R1, T1, RT);
    return RT;
}

// 鏍规嵁鏍囧畾鏉夸俊鎭紝杈撳嚭鍧愭爣鐐?
void calculate_Object_Points(int board_width, int board_heignt, double circle_distance, vector<cv::Point3f> &objP) {
    for (int i = 0; i < board_heignt; i++) {
        for (int j = 0; j < board_width; j++) {
            objP.push_back(cv::Point3f(j * circle_distance, i * circle_distance, 0));
        }
    }
}
bool calculate_Image_Points_ChessboardCorners(const std::string &path, cv::Size boardSize,
                                              std::vector<std::vector<cv::Point2f>> &imagePoints) {
    // 浣跨敤 cv::glob 鑾峰彇鐩綍涓殑鎵€鏈?bmp鍥剧墖鏂囦欢璺緞
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);  // 鏍规嵁璺緞鍜屾枃浠剁被鍨?.bmp)鑾峰彇鍥剧墖鍒楄〃

    // 妫€鏌ユ槸鍚︽壘鍒板浘鐗囨枃浠?
    if (imageList.empty()) {
        std::cerr << "No images found in the directory!" << std::endl;
        return false;
    }

    int nframes = (int)imageList.size();
    int imageCount = 0;

    // 閬嶅巻鎵€鏈夌殑鍥剧墖
    for (int i = 0; i < nframes; i++) {
        cv::Mat view, viewGray;

        // 璇诲彇褰撳墠鍥剧墖
        if (i < (int)imageList.size()) {
            std::cout << "Processing image: " << imageList[i] << std::endl;
            view = cv::imread(imageList[i], cv::IMREAD_COLOR);  // 璇诲彇褰╄壊鍥剧墖
        }

        if (view.empty()) {
            std::cout << "Could not open or find the image at " << imageList[i] << std::endl;
            continue;
        }

        std::vector<cv::Point2f> imagePointsBuf;

        // 妫€鏌ュ浘鍍忔槸鍚﹀凡缁忔槸鐏板害鍥惧儚
        if (view.channels() == 3) {
            cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);  // 濡傛灉鏄僵鑹插浘鍍忥紝鍒欒浆鎹负鐏板害鍥惧儚
        } else {
            viewGray = view;  // 濡傛灉宸茬粡鏄伆搴﹀浘鍍忥紝鐩存帴浣跨敤
        }

        // 浣跨敤 cv::findChessboardCornersSB 鏌ユ壘妫嬬洏鏍艰鐐?
        if (cv::findChessboardCornersSB(viewGray, boardSize, imagePointsBuf)) {
            // 瀵硅鐐硅繘琛屼簹鍍忕礌绾х簿纭寲
            cv::cornerSubPix(viewGray, imagePointsBuf, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

            // 缁熶竴瑙掔偣椤哄簭锛岀‘淇濋『鏃堕拡椤哄簭
            if (imagePointsBuf[0].x > imagePointsBuf[imagePointsBuf.size() - 1].x) {
                std::vector<cv::Point2f> buf;
                for (int i = 0; i < imagePointsBuf.size(); i++) {
                    buf.push_back(imagePointsBuf[imagePointsBuf.size() - 1 - i]);
                }
                imagePointsBuf.clear();
                imagePointsBuf = buf;
            }

            // 鍙鍖栬鐐瑰苟淇濆瓨
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

            // 灏嗚鐐逛繚瀛樺埌 imagePoints 涓?
            imagePoints.push_back(imagePointsBuf);
        } else {
            std::cout << "Num " << i << " can not find chessboard corners!\n";
        }

        // 鏄剧ず澶勭悊鍚庣殑鍥惧儚
        imageCount++;
        if (imageCount == 1) {
            // 鑾峰彇绗竴寮犲浘鐗囩殑鍥惧儚瀹介珮淇℃伅
            std::cout << "Image size: " << view.cols << "x" << view.rows << std::endl;
        }
    }

    // 绛夊緟鏄剧ず绐楀彛鍏抽棴
    return true;
}
// 鏍规嵁璇诲彇鐨勫浘鐗囷紝璁＄畻鍚勫紶鍥剧墖鐨勫渾蹇冮泦鍚?
bool calculate_Image_Points(std::string &path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>> &imagePoints) {
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);  // 鍦嗙洏涓篵mp
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
        cv::bitwise_not(viewGray, viewGray);  // 鍙嶈浆鐏板害鍥惧儚
        // 瀹為檯鏍囧畾鍥剧墖锛屽簲鐏板害缈昏浆
        for (int row = 0; row < viewGray.rows; row++) {
            for (int col = 0; col < viewGray.cols; col++) {
                viewGray.at<uchar>(row, col) = 255 - viewGray.at<uchar>(row, col);  // 鐏板害鍙嶈浆
            }
        }

        //// Blob绠楀瓙鍙傛暟
        cv::SimpleBlobDetector::Params params;
        // params.filterByArea = true;
        params.maxArea = 10e4;  // 10e4
        params.minArea = 30;    // 30
        params.minDistBetweenBlobs = 10;
        // params.minThreshold = 10;   //榛樿50
        // params.maxThreshold = 250;  //榛樿220
        params.filterByInertia = true;  // 鏂戠偣鎯€х巼鐨勯檺鍒跺彉閲? 鐭酱/闀胯酱
        params.minInertiaRatio = 0.5f;  // 鏂戠偣鐨勬渶灏忔儻鎬х巼;

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
            // 鑾峰彇鏂囦欢璺緞鍜屾枃浠跺悕
            std::string newFileName = imageList[i];
            // 鍦ㄦ枃浠跺悕鏈熬娣诲姞 "_unfind" 鍚庣紑
            size_t lastDot = newFileName.find_last_of(".");
            if (lastDot != std::string::npos) {
                newFileName.insert(lastDot, "_unfind");
            }

            // 閲嶅懡鍚嶆枃浠?
            if (rename(imageList[i].c_str(), newFileName.c_str()) != 0) {
                std::cerr << "鏃犳硶閲嶅懡鍚嶆枃浠? " << imageList[i] << std::endl;
            } else {
                // std::cout << "閲嶅懡鍚嶄负: " << newFileName << std::endl;
            }
        }
        // 鍙鍖?
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
        cv::Mat view = dirImages[i];  // 鐩存帴浠庝紶鍏ョ殑 dirImages 涓彇鍑哄浘鍍?
        cv::Mat viewGray;

        std::cout << "Processing image " << i + 1 << " / " << nframes << std::endl;

        // 杞崲涓虹伆搴﹀浘鍍?
        cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(viewGray, viewGray);  // 鍙嶈浆鐏板害鍥惧儚
        // 鐏板害鍙嶈浆
        for (int row = 0; row < viewGray.rows; row++) {
            for (int col = 0; col < viewGray.cols; col++) {
                viewGray.at<uchar>(row, col) = 255 - viewGray.at<uchar>(row, col);  // 鐏板害鍙嶈浆
            }
        }

        // Blob绠楀瓙鍙傛暟
        cv::SimpleBlobDetector::Params params;
        params.maxArea = 10e4;            // 璁剧疆鏈€澶ч潰绉?
        params.minArea = 30;              // 璁剧疆鏈€灏忛潰绉?
        params.minDistBetweenBlobs = 10;  // 璁剧疆鏂戠偣涔嬮棿鐨勬渶灏忚窛绂?
        params.filterByInertia = true;    // 鍚敤鏂戠偣鎯€х巼鐨勯檺鍒?
        params.minInertiaRatio = 0.5f;    // 璁剧疆鏈€灏忔儻鎬х巼

        cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

        // 鏌ユ壘妫嬬洏鏍兼爣瀹氭澘涓婄殑鍦嗙偣
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
            imagePoints.push_back(pointbuf);  // 濡傛灉鎵惧埌浜嗗渾蹇冿紝瀛樺叆 imagePoints
        } else {
            std::cout << "Failed to find circle grid in current image." << std::endl;
        }

        // 鍙鍖栫粨鏋?
        drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
    }

    return true;
}

// 宸茬煡鍐呭弬鏍囧畾澶栧弬
void Calibration_Solve_Extrinsics(cv::Mat &Kc, cv::Mat &distCoeffs, vector<cv::Point3f> &objPoints,
                                  std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<cv::Mat> &vecHc) {
    std::vector<double> camera_distortion(distCoeffs.begin<double>(), distCoeffs.end<double>());
    for (int i = 0; i < imagePoints.size(); i++) {
        // 鍒涘缓鏃嬭浆鐭╅樀鍜屽钩绉荤煩闃?
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::solvePnP(objPoints, imagePoints[i], Kc, camera_distortion, rvec, tvec);
        cv::Mat rotM = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(rvec, rotM);  // 灏嗘棆杞悜閲忓彉鎹㈡垚鏃嬭浆鐭╅樀
        cv::Mat RT_Mat_temp;
        hconcat(rotM, tvec, RT_Mat_temp);
        cv::Mat last_line = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);  // 榻愭鐭╅樀鏈€鍚庝竴琛?
        cv::Mat RT_Mat;
        cv::vconcat(RT_Mat_temp, last_line, RT_Mat);  // 杈撳嚭澶栧弬鐭╅樀
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
    plot_->addPlotData(X_data, Y_data, "display", vtkChart::LINE);  // X,Y鍧囦负double鍨嬬殑鍚戦噺
    plot_->plot();                                                  // 缁樺埗鏇茬嚎
}

void draw_line_chart_visualization(pcl::visualization::PCLPlotter *plot_, vector<double> &X_data, vector<double> &Y_data,
                                   string string_title) {
    const char *title = string_title.c_str();

    plot_->setBackgroundColor(1, 1, 1);
    plot_->setTitle(title);
    plot_->setXTitle("X");
    plot_->setYTitle("Y");
    plot_->addPlotData(X_data, Y_data, "display", vtkChart::LINE);  // X,Y鍧囦负double鍨嬬殑鍚戦噺
    plot_->plot();                                                  // 缁樺埗鏇茬嚎

    while (!plot_->wasStopped()) {
        plot_->spinOnce(100);
    }
}
