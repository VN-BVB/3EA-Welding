#include "PointCloudReconstruction.h"

#include "settingPara/SettingPara.h"
#include "structLightCamera/config/StructLightConfig.h"
#include "utils/common/CommonFunc.h"
#include "utils/pointCloud/PointCloudFunc.h"
#ifdef SMART_CAMERA
#include "deepLearning/objectDetect/AbstractObjectDetect.h"
#include "deepLearning/objectDetect/yolo11/yolo11objdetInference.h"
#include "utils/common/WeldSeamInfo.h"
#endif

PointCloudReconstruction::PointCloudReconstruction() : structLightConfig(StructLightConfig::getInstance()) {
    initPara();  // 初始化需要用到的参数
#ifdef SMART_CAMERA
    initObjectDetect();  // 初始化目标检测类
#endif
}

// 初始化需要用到的参数
void PointCloudReconstruction::initPara() {
    // 同步调制度阈值
    modulationThreshold = SettingPara::getInstance().modulation_threshold;

    // 初始化用于全局重建的mask
    for (int i = 0; i < cameraHeight; i++) {     // 高 对应y
        for (int j = 0; j < cameraWidth; j++) {  // 宽 对应x
            maskForGlobal.at<uchar>(i, j) = 100;
        }
    }

    // 初始化sin表和cos表
    for (int k = 0; k < phaseShiftImgNum; ++k) {
        double angle = 2.0 * k * myPi / phaseShiftImgNum;
        sinTable[k] = sin(angle);
        cosTable[k] = cos(angle);
    }

    // 畸变系数集合  Opencv：k1, k2, p1, p2, k3
    cameraDistortion = (cv::Mat_<double>(5, 1) << structLightConfig.camera_distortion[0], structLightConfig.camera_distortion[1],
                        structLightConfig.camera_distortion[3], structLightConfig.camera_distortion[4], structLightConfig.camera_distortion[2]);
    projectorDistortion = (cv::Mat_<double>(5, 1) << structLightConfig.project_distortion[0], structLightConfig.project_distortion[1],
                           structLightConfig.project_distortion[3], structLightConfig.project_distortion[4], structLightConfig.project_distortion[2]);
    // 初始化畸变表
    QtConcurrent::run([this]() { initDistortionMap(); });
}
void PointCloudReconstruction::initDistortionMap() {
    cameraMapX = cv::Mat(cameraHeight, cameraWidth, CV_32F);
    cameraMapY = cv::Mat(cameraHeight, cameraWidth, CV_32F);

    for (int v = 0; v < cameraHeight; v++) {
        for (int u = 0; u < cameraWidth; u++) {
            std::vector<cv::Point2f> src(1);
            std::vector<cv::Point2f> dst;

            src[0] = cv::Point2f(u, v);

            cv::undistortPoints(src, dst, structLightConfig.Kc, cameraDistortion, cv::Mat(), structLightConfig.Kc);

            cameraMapX.at<float>(v, u) = dst[0].x;
            cameraMapY.at<float>(v, u) = dst[0].y;
        }
    }
}
// 初始化目标检测类
#ifdef SMART_CAMERA
void PointCloudReconstruction::initObjectDetect() {
    weldsCoarsePosition = std::make_shared<Yolo11ObjDetInference>();

    weldsCoarsePosition->setEngine_path(objDetEnginePath);
    weldsCoarsePosition->setClassNames(classNames);
    weldsCoarsePosition->setColors(colors);
    weldsCoarsePosition->setScore_thres(scoreThreshold);
    weldsCoarsePosition->setIou_thres(iouThreshold);
    weldsCoarsePosition->setLabelsNum(labelNum);
    weldsCoarsePosition->setSize(cv::Size{imgSize, imgSize});

    weldsCoarsePosition->initialization();  // 完成类的初始化 (读取模型文件, 移入显卡等)
}
#endif

// 局部点云重建
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudReconstruction::localReconstruct(int minU, int maxU, int minV, int maxV) {
    PLOGD << "重建开始...";

    // 变量重新初始化
    reInitialize();
    phaseShiftImages.clear();  // 相移图集合
    grayCodeImages.clear();    // 格雷码图集合

    if (primaryCameraCapturedImg->size() == SettingPara::getInstance().projector_num) {
        imageDistribute();                               // 0. 将采集到的图像放入相移和格雷码容器
        makeMaskForReconstruct(minU, maxU, minV, maxV);  // 1.1 更新背景平面的mask设置
        solveWrapPhase();                                // 2. 相移法求包裹相位
        decodeGrayCode();                                // 3. 解码格雷码
        phaseUnwrap();                                   // 4. 相位展开, 求绝对相位
        reconstructPoint();                              // 5.3 重建点云
    }

    PLOGD << "重建完成";
    return pointCloud;
}

std::vector<std::shared_ptr<WeldSeamInfo>> PointCloudReconstruction::weldAreaReconstructToSA() {
    PLOGD << "重建角钢工作台平面以及焊缝区域点云";
    // 部分变量清空
    initWorkspaceContext();

    SettingPara& set = SettingPara::getInstance();
    if (primaryCameraCapturedImg->size() == set.projector_num) {
        reInitialize();         // 变量重新初始化
        imageDistribute();      // 0. 将采集到的图像放入相移和格雷码容器
        makeMaskForSeamsDet();  // 1.2 完成重建掩膜的生成 (包括背景和焊缝区域)
        maskForReconstruct = maskForWorkbench.clone();
        solveWrapPhase();           // 2. 相移法求包裹相位
        decodeGrayCode();           // 3. 解码格雷码
        phaseUnwrap();              // 4. 相位展开, 求绝对相位
        reconstructForWorkbench();  // 5.1 背景平面的三维重建, 拟合背景平面参数

        reInitialize();     // 变量重新初始化
        imageDistribute();  // 0. 将采集到的图像放入相移和格雷码容器
        maskForReconstruct = maskForWorkpiece.clone();
        solveWrapPhase();          // 2. 相移法求包裹相位
        decodeGrayCode();          // 3. 解码格雷码
        phaseUnwrap();             // 4. 相位展开, 求绝对相位
        reconstructForSeamArea();  // 5.2 目标检测框的三维重建
    }
    PLOGD << "点云放缩平移... ...";
    for (auto& info : weldAreaInfo) {
        MyToolFunc::scalePointClouds(info->weldAreaPointCloudInCamera, set.scaleOfPointX, set.transOfPointX, set.scaleOfPointY, set.transOfPointY);
    }
    PLOGD << "焊缝区域点云重建完成";

    return weldAreaInfo;
}
// 焊缝区域点云重建
std::vector<std::shared_ptr<WeldSeamInfo>> PointCloudReconstruction::weldAreaReconstructToGF() {
    PLOGD << "重建三轴工件点云";
    // 部分变量清空
    initWorkspaceContext();
    SettingPara& set = SettingPara::getInstance();
    if (primaryCameraCapturedImg->size() == set.projector_num) {
        reInitialize();             // 变量重新初始化
        imageDistribute();          // 0. 将采集到的图像放入相移和格雷码容器
        makeMaskForSeamsDetToGF();  // 1.2 完成重建掩膜的生成 (包括背景和焊缝区域)
        if (!skipWorkbenchFilter) {
            maskForReconstruct = maskForWorkbench.clone();
            solveWrapPhase();           // 2. 相移法求包裹相位
            decodeGrayCode();           // 3. 解码格雷码
            phaseUnwrap();              // 4. 相位展开, 求绝对相位
            reconstructForWorkbench();  // 5.1 背景平面的三维重建, 拟合背景平面参数
        }

        reInitialize();     // 变量重新初始化
        imageDistribute();  // 0. 将采集到的图像放入相移和格雷码容器
        maskForReconstruct = maskForWorkpiece.clone();
        solveWrapPhase();          // 2. 相移法求包裹相位
        decodeGrayCode();          // 3. 解码格雷码
        phaseUnwrap();             // 4. 相位展开, 求绝对相位
        reconstructForSeamArea();  // 5.2 目标检测框的三维重建
    }
    PLOGD << "点云放缩平移... ...";
    for (auto& info : weldAreaInfo) {
        MyToolFunc::scalePointClouds(info->weldAreaPointCloudInCamera, set.scaleOfPointX, set.transOfPointX, set.scaleOfPointY, set.transOfPointY);
    }
    PLOGD << "焊缝区域点云重建完成";

    return weldAreaInfo;
}
void PointCloudReconstruction::initWorkspaceContext() {
    maskForWorkpiece = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);  // 用于工件焊缝区域重建的掩模
    maskForWorkbench = maskForGlobal.clone();                               // 用于工作台重建的掩模
    workbenchPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);          // 重置工作台点云
    nonPlanePointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);           // 重置工作台点云
    phaseShiftImages.clear();                                               // 相移图集合
    grayCodeImages.clear();                                                 // 格雷码图集合
    weldAreaInfo.clear();                                                   // 清空焊缝区域信息
    workbenchCoeff = Eigen::VectorXf::Zero(4);                              // 工作台平面系数
    skipWorkbenchFilter = false;                                            // 跳过筛选工作台
}

//  点云重建变量重新初始化
void PointCloudReconstruction::reInitialize() {
    pointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);  // 重置点云
#ifdef SMART_CAMERA
    detRes = std::make_shared<std::vector<DetResult>>();  // 清空目标检测结果
#endif

    sinSum = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                     // sin和
    cosSum = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                     // cos和
    wrapPhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                  // 包裹相位
    modulation = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);                 // 调制度
    K1 = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);                          // 每一像素的K1
    K2 = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);                          // 每一像素的K2
    binarizationThreshold = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);      // 每一像素的二值化阈值
    absolutePhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);              // 绝对相位
    unDistortionAbsolutePhase = cv::Mat::zeros(cameraHeight, cameraWidth, CV_64FC1);  // 去除相机畸变后的绝对相位
    maskForReconstruct = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);          // 二维目标检测mask清零
}

// 0. 将采集到的图像放入相移和格雷码容器
void PointCloudReconstruction::imageDistribute() {
    int index = 0;
    // 载入相移
    for (int i = 0; i < phaseShiftImgNum; i++) {
        cv::Mat img;
        // 后面一项参数若加0.00392(1/255), 可将图像转换为[0,1]区间, 可直接用imshow显示
        (*primaryCameraCapturedImg)[index].convertTo(img, CV_64FC1);
        phaseShiftImages.push_back(img);
        index++;
    }
    // 载入格雷码
    for (int i = 0; i < garyCodeTotalImgNum; i++) {
        cv::Mat img;
        (*primaryCameraCapturedImg)[index].convertTo(img, CV_64FC1);
        grayCodeImages.push_back(img);
        index++;
    }
}

// 1.1 更新点云重建的掩膜
void PointCloudReconstruction::makeMaskForReconstruct(int minU, int maxU, int minV, int maxV) {
    PLOGD << "更新点云重建的掩膜...";

    minU = minU < 0 ? 0 : minU;
    maxU = maxU > cameraWidth ? cameraWidth : maxU;
    minV = minV < 0 ? 0 : minV;
    maxV = maxV > cameraHeight ? cameraHeight : maxV;
    if (minU == 0 && maxU == cameraWidth && minV == 0 && maxV == cameraHeight) {
        maskForReconstruct = maskForGlobal.clone();  // 用于重建的掩模(每次重建时更新)
    } else {  // 局部重建时，更新用于重建的掩模                                                                    .
        maskForReconstruct = cv::Mat::zeros(cameraHeight, cameraWidth, CV_8UC1);
        for (int i = minV; i < maxV; i++) {      // 高 对应y
            for (int j = minU; j < maxU; j++) {  // 宽 对应x
                maskForReconstruct.at<uchar>(i, j) = 100;
            }
        }
    }
}

// 1.2 更新焊缝区域目标框的mask
#ifdef SMART_CAMERA
void PointCloudReconstruction::makeMaskForSeamsDet() {
    PLOGD << "计算背景以及焊缝区域的mask";
    cv::Mat beforDistortCorrect = (*primaryCameraCapturedImg)[19].clone();  // 获取空白图
    cv::imwrite("beforDistortCorrect.bmp", beforDistortCorrect);
    cv::Mat afterDistortCorrect;

    // 这里对图像做矫正是为后续焊缝分割做准备, 而非点云重建, 点云重建中已有矫正步骤. 同时忽略此处矫正对目标检测精度的影响.
    cv::undistort(beforDistortCorrect, afterDistortCorrect, structLightConfig.Kc, cameraDistortion);  // 畸变矫正
    cv::cvtColor(afterDistortCorrect, afterDistortCorrect, cv::COLOR_GRAY2RGB);                       // 转为彩色图

    weldsCoarsePosition->inference(afterDistortCorrect, *(detRes.get()));  // 深度学习推理

    // 如果检测到大于4个, 按照置信度排序, 并取前4个
    if (detRes->size() > 4) {
        std::sort(detRes->begin(), detRes->end(), [](const DetResult& a, const DetResult& b) { return a.score > b.score; });
        detRes->resize(4);  // 仅保留置信度最高的4个
    }
    PLOGD << "detRes->size()" << detRes->size();
    // 遍历每个结果
    for (int i = 0; i < detRes->size(); ++i) {
        PLOGD << "焊缝粗定位" << i << "结果: " << (*detRes.get())[i].classId << ", " << (*detRes.get())[i].score << ", "
              << (*detRes.get())[i].topLeftX << ", " << (*detRes.get())[i].topLeftY << ", " << (*detRes.get())[i].bottomRightX << ", "
              << (*detRes.get())[i].bottomRightY;

        // 创建焊缝区域信息对象
        std::shared_ptr<WeldSeamInfo> seamInfo = std::make_shared<WeldSeamInfo>();
        seamInfo->areaNum = i;
        seamInfo->originalImg = afterDistortCorrect;  // 保存原始图像
        seamInfo->rectPtr = std::make_shared<cv::Rect_<float>>((*detRes.get())[i].topLeftX, (*detRes.get())[i].topLeftY,
                                                               (*detRes.get())[i].bottomRightX - (*detRes.get())[i].topLeftX,
                                                               (*detRes.get())[i].bottomRightY - (*detRes.get())[i].topLeftY);  // 保存目标框
        seamInfo->weldAreaImg = afterDistortCorrect(*(seamInfo->rectPtr)).clone();         // 保存目标框内的图像
        seamInfo->weldAreaType = MyToolFunc::getWeldAreaType((*detRes.get())[i].classId);  // 保存焊缝类型
        weldAreaInfo.push_back(seamInfo);                                                  // 将焊缝区域信息添加到列表中

        // 更新mask
        if (seamInfo->rectPtr.get()->area() > 0) {
            maskForWorkpiece(*(seamInfo->rectPtr)).setTo(i + 1);
            maskForWorkbench(*(seamInfo->rectPtr)).setTo(0);
        }
    }
}
void PointCloudReconstruction::makeMaskForSeamsDetToGF() {
    PLOGD << "计算背景以及焊缝区域的mask";
    cv::Mat beforDistortCorrect = (*primaryCameraCapturedImg)[19].clone();  // 获取空白图
    cv::imwrite("beforDistortCorrect.bmp", beforDistortCorrect);
    cv::Mat afterDistortCorrect;

    // 这里对图像做矫正是为后续焊缝分割做准备, 而非点云重建, 点云重建中已有矫正步骤. 同时忽略此处矫正对目标检测精度的影响.
    cv::undistort(beforDistortCorrect, afterDistortCorrect, structLightConfig.Kc, cameraDistortion);  // 畸变矫正
    cv::cvtColor(afterDistortCorrect, afterDistortCorrect, cv::COLOR_GRAY2RGB);                       // 转为彩色图

    // weldsCoarsePosition->inference(afterDistortCorrect, *(detRes.get()));  // 深度学习推理

    if (detRes->size() == 0) {
        PLOGW << "检测结果为空，使用默认ROI";
        // detRes->emplace_back(1, 1.0f, 91, 237, 1500, 600);//管侧与三角肘板
        detRes->emplace_back(0, 1.0f, 414, 343, 498, 438);
        detRes->emplace_back(0, 1.0f, 596, 419, 1090, 469);
        detRes->emplace_back(0, 1.0f, 1170, 285, 1207, 391);
    }
    // 如果检测到大于4个, 按照置信度排序, 并取前4个
    if (detRes->size() > 4) {
        std::sort(detRes->begin(), detRes->end(), [](const DetResult& a, const DetResult& b) { return a.score > b.score; });
        detRes->resize(4);  // 仅保留置信度最高的4个
    }
    PLOGD << "detRes->size()" << detRes->size();
    // 遍历每个结果
    for (int i = 0; i < detRes->size(); ++i) {
        /* ===== ROI扩张 + 边界限制 ===== */

        auto& det = (*detRes)[i];

        int x1 = std::max(0, static_cast<int>(det.topLeftX - pclRectExtend));
        int y1 = std::max(0, static_cast<int>(det.topLeftY - pclRectExtend));
        int x2 = std::min(cameraWidth - 1, static_cast<int>(det.bottomRightX + pclRectExtend));
        int y2 = std::min(cameraHeight - 1, static_cast<int>(det.bottomRightY + pclRectExtend));

        det.topLeftX = x1;
        det.topLeftY = y1;
        det.bottomRightX = x2;
        det.bottomRightY = y2;

        PLOGD << "焊缝粗定位" << i << "结果: " << (*detRes.get())[i].classId << ", " << (*detRes.get())[i].score << ", "
              << (*detRes.get())[i].topLeftX << ", " << (*detRes.get())[i].topLeftY << ", " << (*detRes.get())[i].bottomRightX << ", "
              << (*detRes.get())[i].bottomRightY;

        // 创建焊缝区域信息对象
        std::shared_ptr<WeldSeamInfo> seamInfo = std::make_shared<WeldSeamInfo>();
        seamInfo->areaNum = i;
        seamInfo->originalImg = afterDistortCorrect;  // 保存原始图像
        seamInfo->rectPtr = std::make_shared<cv::Rect_<float>>((*detRes.get())[i].topLeftX, (*detRes.get())[i].topLeftY,
                                                               (*detRes.get())[i].bottomRightX - (*detRes.get())[i].topLeftX,
                                                               (*detRes.get())[i].bottomRightY - (*detRes.get())[i].topLeftY);  // 保存目标框
        seamInfo->weldAreaImg = afterDistortCorrect(*(seamInfo->rectPtr)).clone();               // 保存目标框内的图像
        seamInfo->weldAreaType = MyToolFunc::getWeldAreaType((*detRes.get())[i].classId + 100);  // 保存焊缝类型
        weldAreaInfo.push_back(seamInfo);                                                        // 将焊缝区域信息添加到列表中

        // 更新mask
        if (seamInfo->rectPtr.get()->area() > 0) {
            maskForWorkpiece(*(seamInfo->rectPtr)).setTo(i + 1);
            maskForWorkbench(*(seamInfo->rectPtr)).setTo(0);
        }
    }
    if (weldAreaInfo.empty()) {
        return;
    } else {
        skipWorkbenchFilter = std::all_of(weldAreaInfo.begin(), weldAreaInfo.end(),
                                          [](const std::shared_ptr<WeldSeamInfo>& info) { return info && info->weldAreaType == Plate_Plate_F; });
    }
}
#endif

// 2. 相移法求包裹相位
void PointCloudReconstruction::solveWrapPhase() {
    PLOGD << "相移法求包裹相位";
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < cameraHeight; i++) {  // 高 (在其中处理某个高度, 一行的数据), 定位到这一行的第一个像素的地址
        double* pixelSinSum = (double*)sinSum.data + i * cameraWidth;          // 像素Sin和
        double* pixelCosSum = (double*)cosSum.data + i * cameraWidth;          // 像素Cos和
        double* pixelModulation = (double*)modulation.data + i * cameraWidth;  // 像素调制度
        double* pixelWrapPhase = (double*)wrapPhase.data + i * cameraWidth;    // 像素包裹相位

        for (int j = 0; j < cameraWidth; j++) {               // 宽（在其中处理某一行的每一个像素数据）
            if (maskForReconstruct.at<uchar>(i, j) != 0) {    // 如果当前像素需要处理
                for (int k = 0; k < phaseShiftImgNum; k++) {  // 将每一张相移条纹图像的这个位置的像素值
                    (*pixelSinSum) = (*pixelSinSum) + phaseShiftImages[k].at<double>(i, j) * sinTable[k];
                    (*pixelCosSum) = (*pixelCosSum) + phaseShiftImages[k].at<double>(i, j) * cosTable[k];
                }

                // 求解每一点的调制度
                (*pixelModulation) = sqrt(pow((*pixelSinSum), 2) + pow((*pixelCosSum), 2)) * 2.0 / phaseShiftImgNum;
                if ((*pixelModulation) > modulationThreshold) {
                    // 求解每一点的包裹相位
                    (*pixelWrapPhase) = -atan2((*pixelSinSum), (*pixelCosSum));
                    if ((*pixelWrapPhase) <= 0) {
                        (*pixelWrapPhase) = (*pixelWrapPhase) + 2 * myPi;
                    }
                }
            }
            pixelSinSum++;  // 移动到当前行的下一个像素
            pixelCosSum++;
            pixelModulation++;
            pixelWrapPhase++;
        }
    }
}

// 3. 解码格雷码
void PointCloudReconstruction::decodeGrayCode() {
    PLOGD << "解码格雷码";
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < cameraHeight; i++) {                                                         // 高
        double* pixelModulation = (double*)modulation.data + i * cameraWidth;                        // 逐像素调制度
        double* pixelBinarizationThreshold = (double*)binarizationThreshold.data + i * cameraWidth;  // 逐像素黑白阈值
        double* pixelProjectBlack = (double*)grayCodeImages[6].data + i * cameraWidth;               // 逐像素投影全黑时的像素值
        double* pixelProjectWhite = (double*)grayCodeImages[7].data + i * cameraWidth;               // 逐像素投影全白时的像素值
        uchar* pixelK1 = (uchar*)K1.data + i * cameraWidth;                                          // 逐像素对应的传统格雷码编码
        uchar* pixelK2 = (uchar*)K2.data + i * cameraWidth;                                          // 逐像素对应的互补格雷码编码

        for (int j = 0; j < cameraWidth; j++) {  // 宽
            if (maskForReconstruct.at<uchar>(i, j) != 0) {
                if (*pixelModulation > modulationThreshold) {  // 调制度达到阈值
                    // 6+1索引[6]和[7]分别对应全黑和全白
                    *pixelBinarizationThreshold = (*pixelProjectBlack + *pixelProjectWhite) / 2.0;  // 得到区分黑与白的中间阈值
                    //-- 对每一序列的格雷码, 对应的每一像素, 分别进行二值化 --
                    // 求解每一序列的K1和K2
                    uchar V1 = 0;  // 每一像素序列, 组成的转换为格雷码的十进制V1
                    uchar V2 = 0;  // 每一像素序列, 组成的转换为格雷码的十进制V2
                    for (int k = 0; k < garyCodeEncodeImgNum; k++) {
                        uchar result_onebit;
                        if (grayCodeImages[k].at<double>(i, j) > *pixelBinarizationThreshold) {
                            result_onebit = 1;
                        } else {
                            result_onebit = 0;
                        }
                        V1 = V1 + result_onebit * pow(2, (garyCodeEncodeImgNum - 1 - k));
                        V2 = V2 + result_onebit * pow(2, (garyCodeEncodeImgNum - k));
                    }

                    // 最后一张互补格雷码
                    V2 = V2 + pow(2, 0) * char((grayCodeImages.back().at<double>(i, j) > *pixelBinarizationThreshold ? 1 : 0));

                    *pixelK1 = VK1[V1];  // 十进制码对应到格雷码
                    *pixelK2 = VK2[V2];  // 十进制码对应到互补格雷码
                }
            }
            pixelModulation++;
            pixelBinarizationThreshold++;
            pixelProjectBlack++;
            pixelProjectWhite++;
            pixelK1++;
            pixelK2++;
        }
    }
}

// 4. 相位展开, 求绝对相位
void PointCloudReconstruction::phaseUnwrap() {
    PLOGD << "相位展开, 求绝对相位";
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < cameraHeight; i++) {                                         // 高
        double* pixelModulation = (double*)modulation.data + i * cameraWidth;        // 逐像素调制度
        double* pixelWrapPhase = (double*)wrapPhase.data + i * cameraWidth;          // 逐像素包裹相位
        double* pixelAbsolutePhase = (double*)absolutePhase.data + i * cameraWidth;  // 逐像素绝对相位
        uchar* pixelK1 = (uchar*)K1.data + i * cameraWidth;                          // 逐像素对应的传统格雷码编码
        uchar* pixelK2 = (uchar*)K2.data + i * cameraWidth;                          // 逐像素对应的传统加互补格雷码编码

        for (int j = 0; j < cameraWidth; j++) {  // 宽
            if (maskForReconstruct.at<uchar>(i, j) != 0) {
                if (*pixelModulation > modulationThreshold) {
                    // 求解绝对相位
                    if (*pixelWrapPhase <= myPi / 2.0) {
                        *pixelAbsolutePhase = *pixelWrapPhase + 2 * myPi * (*pixelK2);
                    } else if ((*pixelWrapPhase > myPi / 2.0) && (*pixelWrapPhase < 3.0 * myPi / 2.0)) {
                        *pixelAbsolutePhase = *pixelWrapPhase + 2 * myPi * (*pixelK1);
                    } else if (*pixelWrapPhase >= 3.0 * myPi / 2.0) {
                        *pixelAbsolutePhase = *pixelWrapPhase + 2 * myPi * (*pixelK2 - 1);
                    }

                    // 绝对相位归一化 Φ∈(0,2π⋅2^N)
                    *pixelAbsolutePhase = *pixelAbsolutePhase / (2 * myPi * pow(2, garyCodeEncodeImgNum));
                } else {
                    *pixelAbsolutePhase = 0;
                }
            } else {
                *pixelAbsolutePhase = 0;
            }
            pixelModulation++;
            pixelWrapPhase++;
            pixelAbsolutePhase++;
            pixelK1++;
            pixelK2++;
        }
    }
}

// 5.0.1 相机和投影仪匹配对应点
void PointCloudReconstruction::cameraProjectMatch(std::vector<cv::Point2d>& cameraCoord, std::vector<double>& projectCoord, int minU, int maxU,
                                                  int minV, int maxV) {
#pragma omp parallel num_threads(12)
    {
        std::vector<cv::Point2d> cameraCoordinatePrivate;  // 相机匹配点(u,v)
        std::vector<double> projectCoordinatePrivate;      // 投影仪匹配点横坐标u
#pragma omp for nowait
        for (int i = 0; i < cameraHeight; i++) {
            double* pixelAbsolutePhase = (double*)absolutePhase.data + i * cameraWidth;  // 当前像素绝对相位
            float* mapXptr = cameraMapX.ptr<float>(i);
            float* mapYptr = cameraMapY.ptr<float>(i);
            for (int j = 0; j < cameraWidth; j++) {
                if (i > minV && i < maxV && j > minU && j < maxU) {
                    double up = *pixelAbsolutePhase * projectorWidth;  // 求出投影仪对应的列像素

                    float uc = mapXptr[j];
                    float vc = mapYptr[j];

                    if (uc > 0 && vc > 0 && uc < cameraWidth - 1 && vc < cameraHeight - 1) {
                        projectCoordinatePrivate.push_back(up);
                        cameraCoordinatePrivate.emplace_back(uc, vc);
                    }
                }
                pixelAbsolutePhase++;
            }
        }
#pragma omp critical
        {
            cameraCoord.insert(cameraCoord.end(), cameraCoordinatePrivate.begin(), cameraCoordinatePrivate.end());
            projectCoord.insert(projectCoord.end(), projectCoordinatePrivate.begin(), projectCoordinatePrivate.end());
        }
    }
}

// 5.0.2 计算点云
void PointCloudReconstruction::calcPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<cv::Point2d>& cameraCoord,
                                              std::vector<double>& projectCoord) {
    std::cout << projectCoord.size() << std::endl;
    const cv::Mat& Ac = structLightConfig.Ac;
    const cv::Mat& Ap = structLightConfig.Ap;

    // -------- 相机矩阵缓存 --------
    double Ac00 = Ac.at<double>(0, 0);
    double Ac01 = Ac.at<double>(0, 1);
    double Ac02 = Ac.at<double>(0, 2);
    double Ac03 = Ac.at<double>(0, 3);

    double Ac10 = Ac.at<double>(1, 0);
    double Ac11 = Ac.at<double>(1, 1);
    double Ac12 = Ac.at<double>(1, 2);
    double Ac13 = Ac.at<double>(1, 3);

    double Ac20 = Ac.at<double>(2, 0);
    double Ac21 = Ac.at<double>(2, 1);
    double Ac22 = Ac.at<double>(2, 2);
    double Ac23 = Ac.at<double>(2, 3);

    // -------- 投影仪矩阵缓存 --------
    double Ap00 = Ap.at<double>(0, 0);
    double Ap01 = Ap.at<double>(0, 1);
    double Ap02 = Ap.at<double>(0, 2);
    double Ap03 = Ap.at<double>(0, 3);

    double Ap10 = Ap.at<double>(1, 0);
    double Ap11 = Ap.at<double>(1, 1);
    double Ap12 = Ap.at<double>(1, 2);
    double Ap13 = Ap.at<double>(1, 3);

    double Ap20 = Ap.at<double>(2, 0);
    double Ap21 = Ap.at<double>(2, 1);
    double Ap22 = Ap.at<double>(2, 2);
    double Ap23 = Ap.at<double>(2, 3);

#pragma omp parallel num_threads(12)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr recons_cloud_private(new pcl::PointCloud<pcl::PointXYZ>);  // 重建点云
        cv::Mat A(3, 3, CV_64FC1);
        cv::Mat B(3, 1, CV_64FC1);
        cv::Mat XYZ(3, 1, CV_64FC1);
        cv::Mat XYZ_new(3, 1, CV_64FC1);

#pragma omp for nowait
        for (int i = 0; i < cameraCoord.size(); i++) {
            // 1. 求出带有投影仪畸变的三维点坐标
            // 相机和投影仪坐标点
            double uc = cameraCoord[i].x;
            double vc = cameraCoord[i].y;
            double up = projectCoord[i];
            //(AX=B) 最小二乘法中A矩阵(3*3)
            A.at<double>(0, 0) = Ac00 - uc * Ac20;
            A.at<double>(0, 1) = Ac01 - uc * Ac21;
            A.at<double>(0, 2) = Ac02 - uc * Ac22;

            A.at<double>(1, 0) = Ac10 - vc * Ac20;
            A.at<double>(1, 1) = Ac11 - vc * Ac21;
            A.at<double>(1, 2) = Ac12 - vc * Ac22;

            A.at<double>(2, 0) = Ap00 - up * Ap20;
            A.at<double>(2, 1) = Ap01 - up * Ap21;
            A.at<double>(2, 2) = Ap02 - up * Ap22;
            //(AX=B) 最小二乘法中B矩阵(3*1)
            B.at<double>(0, 0) = uc * Ac23 - Ac03;
            B.at<double>(1, 0) = vc * Ac23 - Ac13;
            B.at<double>(2, 0) = up * Ap23 - Ap03;
            // 最小二乘法X矩阵(3*1)
            XYZ.setTo(0);
            // 采用SVD、LU等方法最小二乘法求解XYZ
            cv::solve(A, B, XYZ, cv::DECOMP_LU);
            // 世界坐标系中坐标
            pcl::PointXYZ coordinate;
            coordinate.x = XYZ.at<double>(0, 0);
            coordinate.y = XYZ.at<double>(1, 0);
            coordinate.z = XYZ.at<double>(2, 0);
            // 2. 求出带畸变的vp
            double vp = (Ap10 * coordinate.x + Ap11 * coordinate.y + Ap12 * coordinate.z + Ap13) /
                        (Ap20 * coordinate.x + Ap21 * coordinate.y + Ap22 * coordinate.z + Ap23);
            // 3. 投影仪畸变矫正
            std::vector<cv::Point2f> currentPoint;
            currentPoint.push_back(cv::Point2f(up, vp));  // 当前已经畸变的当前点
            std::vector<cv::Point2f> undistortPoint;      // 对应的原始未畸变点
            cv::undistortPoints(currentPoint, undistortPoint, structLightConfig.Kp, projectorDistortion, cv::Mat(),
                                structLightConfig.Kp);  // 畸变矫正
            // 4. 求出无畸变的三维坐标点
            XYZ_new.setTo(0);
            if ((undistortPoint[0].x > 0) && (undistortPoint[0].y > 0) && (undistortPoint[0].x < projectorWidth - 1) &&
                (undistortPoint[0].y < projectorHeight - 1) && up > 0) {  // 对应原始未畸变点范围进行限制
                double up_new = undistortPoint[0].x;
                // 将AX=B中, 存在up项的重新赋值
                A.at<double>(2, 0) = Ap00 - up_new * Ap20;
                A.at<double>(2, 1) = Ap01 - up_new * Ap21;
                A.at<double>(2, 2) = Ap02 - up_new * Ap22;
                B.at<double>(2, 0) = up_new * Ap23 - Ap03;
                // 采用SVD、LU等方法最小二乘法求解XYZ
                cv::solve(A, B, XYZ_new, cv::DECOMP_LU);
                coordinate.x = XYZ_new.at<double>(0, 0);
                coordinate.y = XYZ_new.at<double>(1, 0);
                coordinate.z = XYZ_new.at<double>(2, 0);
                recons_cloud_private->push_back(coordinate);
                // std::cout << " [" << undistortPoint[0].x << ", " << undistortPoint[0].y << "] ";
            }
        }
        // std::cout << std::endl;
#pragma omp critical
        { pointCloud->insert(pointCloud->end(), recons_cloud_private->begin(), recons_cloud_private->end()); }
    }
}

// 5.0.3 点云后处理
void PointCloudReconstruction::pointCloudPostProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
    if (pointCloud->size() > 0) {
        // 将点云坐标变换为相机光心Trans_c或投影仪光心Trans_p为原点
        pcl::transformPointCloud(*pointCloud, *pointCloud, structLightConfig.Trans_c);
        // 对点云模型进行滤波
        MyToolFunc::passthroughFilter(pointCloud, pointCloud, SettingPara::getInstance().passthrough_Min,
                                      SettingPara::getInstance().passthrough_Max);  // 直通滤波
        MyToolFunc::statisticalFilter(pointCloud, pointCloud, SettingPara::getInstance().statistical_Pts,
                                      SettingPara::getInstance().statistical_Std);  // 统计滤波
        // 解决由于滤波后无序点云尺寸发生变化，height和width未被自动赋值导致的程序崩溃问题；
        pointCloud->height = 1;
        pointCloud->width = static_cast<uint32_t>(pointCloud->size());
    }
}

// 5.1 背景平面的三维重建, 拟合背景平面参数
#ifdef SMART_CAMERA
void PointCloudReconstruction::reconstructForWorkbench() {
    PLOGD << "背景平面的三维重建, 拟合背景平面参数...";
    auto t0 = std::chrono::steady_clock::now();
    // 0. 定义变量
    std::vector<cv::Point2d> cameraCoordinate;  // 相机匹配点(u,v)
    std::vector<double> projectCoordinate;      // 投影仪匹配点横坐标u
    pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto t1 = std::chrono::steady_clock::now();
    PLOGD << "step0 init time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms";

    // 1. 相机和投影仪匹配对应点, 然后对相机点进行畸变矫正
    cameraProjectMatch(cameraCoordinate, projectCoordinate);
    auto t2 = std::chrono::steady_clock::now();
    PLOGD << "step1 cameraProjectMatch time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms";

    // 2. 根据相机的(u,v)和投影仪对应的u, 进行每个匹配点的三维重建
    calcPointCloud(reconstructPointCloud, cameraCoordinate, projectCoordinate);
    auto t3 = std::chrono::steady_clock::now();
    PLOGD << "step2 calcPointCloud time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms";

    // 3. 点云后处理
    pointCloudPostProcess(reconstructPointCloud);
    auto t4 = std::chrono::steady_clock::now();
    PLOGD << "step3 pointCloudPostProcess time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms";

    // 4. 拟合工作台平面
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(reconstructPointCloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelPlane);  // 定义RANSAC算法模型
    ransac.setDistanceThreshold(ransacPlaneThreshold);             // 设定距离阈值
    ransac.setMaxIterations(500);                                  // 设置最大迭代次数
    ransac.setProbability(0.99);                                   // 设置从离群值中选择至少一个样本的期望概率
    ransac.computeModel();                                         // 拟合平面
    ransac.getModelCoefficients(workbenchCoeff);                   // 获取拟合平面参数, coeff分别按顺序保存a,b,c,d
    std::vector<int> inliers;                                      // 用于存放内点索引的vector
    ransac.getInliers(inliers);                                    // 获取内点索引
    auto t5 = std::chrono::steady_clock::now();
    PLOGD << "step4 RANSAC plane time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << " ms";

    // 5. 保存点云到本地(若需)
    PLOGD << "bool_save_model: " << SettingPara::getInstance().bool_save_model;
    reconstructPointCloud->width = reconstructPointCloud->points.size();
    reconstructPointCloud->height = 1;
    if (SettingPara::getInstance().bool_save_model) {
        pcl::io::savePCDFile("./data/common/pointCloud.pcd", *reconstructPointCloud);
    }
    // 背景平面赋值
    pcl::copyPointCloud(*reconstructPointCloud, inliers, *workbenchPointCloud);
    // 获取工件点云
    double a = workbenchCoeff[0];
    double b = workbenchCoeff[1];
    double c = workbenchCoeff[2];
    double d = workbenchCoeff[3];

    double norm = sqrt(a * a + b * b + c * c);

    nonPlanePointCloud->clear();

    for (const auto& p : reconstructPointCloud->points) {
        double dist = fabs(a * p.x + b * p.y + c * p.z + d) / norm;

        if (dist > ransacPlaneThreshold)  // 非平面
        {
            nonPlanePointCloud->points.push_back(p);
        }
    }

    nonPlanePointCloud->width = nonPlanePointCloud->points.size();
    nonPlanePointCloud->height = 1;
    if (SettingPara::getInstance().bool_save_model) {
        pcl::io::savePCDFile("./data/common/nonPlanePointCloud.pcd", *nonPlanePointCloud);
    }
    PLOGD << "重建完成";
}
#endif

// 5.2 目标检测框的三维重建, 填充到对应焊缝信息结构体
#ifdef SMART_CAMERA
void PointCloudReconstruction::reconstructForSeamArea() {
    PLOGD << "焊缝区域三维重建, 剔除背景点云...";
    // 遍历每个焊缝区域
    for (auto& areaInfo : weldAreaInfo) {
        // 0. 定义变量
        std::vector<cv::Point2d> cameraCoordinate;  // 相机匹配点(u,v)
        std::vector<double> projectCoordinate;      // 投影仪匹配点横坐标u
        pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 1. 相机和投影仪匹配对应点,然后对相机点进行畸变矫正
        cameraProjectMatch(cameraCoordinate, projectCoordinate, areaInfo->rectPtr.get()->x,
                           areaInfo->rectPtr.get()->x + areaInfo->rectPtr.get()->width, areaInfo->rectPtr.get()->y,
                           areaInfo->rectPtr.get()->y + areaInfo->rectPtr.get()->height);

        // 2. 根据相机的(u,v)和投影仪对应的u，进行每个匹配点的三维重建
        calcPointCloud(reconstructPointCloud, cameraCoordinate, projectCoordinate);

        // 3. 点云后处理
        pointCloudPostProcess(reconstructPointCloud);

        // 4. 根据工作台平面参数，从原始点云中剔除背景点
        if (workbenchCoeff.head<3>().norm() > 1e-6) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemovePlane(new pcl::PointCloud<pcl::PointXYZ>);
            cloudRemovePlane->reserve(reconstructPointCloud->size());
            for (int i = 0; i < reconstructPointCloud->size(); i++) {
                if (pcl::pointToPlaneDistance(reconstructPointCloud->points[i], workbenchCoeff) > workPlaneRemoveThreshold) {
                    cloudRemovePlane->push_back(reconstructPointCloud->points[i]);
                }
            }
            reconstructPointCloud->swap(*cloudRemovePlane);  // 用新点云替换原点云，swap 只是交换内部指针
        }

        areaInfo->weldAreaPointCloudInCamera = reconstructPointCloud;  // 保存焊缝区域点云到焊缝信息结构体

        // 5. 保存点云到本地(若需)
        // PLOGD << "bool_save_model: " << SettingPara::getInstance().bool_save_model;
        PLOGD << "reconstructPointCloud->size(): " << reconstructPointCloud->size();
        if (SettingPara::getInstance().bool_save_model) {
            pcl::io::savePCDFile("./data/common/pointCloud" + std::to_string(areaInfo->areaNum) + ".pcd", *reconstructPointCloud);
        }
    }
}
#endif

// 5.3 点云三维重建
void PointCloudReconstruction::reconstructPoint() {
    auto t0 = std::chrono::high_resolution_clock::now();
    PLOGD << "点云三维重建...";

    // 0. 定义变量
    std::vector<cv::Point2d> cameraCoordinate;  // 相机匹配点(u,v)
    std::vector<double> projectCoordinate;      // 投影仪匹配点横坐标u
    pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 相机和投影仪匹配对应点, 然后对相机点进行畸变矫正
    auto t1 = std::chrono::high_resolution_clock::now();
    cameraProjectMatch(cameraCoordinate, projectCoordinate);
    auto t2 = std::chrono::high_resolution_clock::now();

    // 2. 根据相机的(u,v)和投影仪对应的u, 进行每个匹配点的三维重建
    PLOGD << "根据相机的(u,v)和投影仪对应的u, 进行每个匹配点的三维重建";
    calcPointCloud(reconstructPointCloud, cameraCoordinate, projectCoordinate);
    auto t3 = std::chrono::high_resolution_clock::now();

    // 3. 点云后处理
    if (reconstructPointCloud->size() > 0) {
        pointCloudPostProcess(reconstructPointCloud);
        pcl::io::savePCDFile("./data/common/CommonPC.pcd", *reconstructPointCloud);
    }

    pointCloud = reconstructPointCloud;
    auto t4 = std::chrono::high_resolution_clock::now();
    // 输出耗时
    std::cout << "cameraProjectMatch time: " << std::chrono::duration<double, std::milli>(t2 - t1).count() << " ms" << std::endl;

    std::cout << "calcPointCloud time: " << std::chrono::duration<double, std::milli>(t3 - t2).count() << " ms" << std::endl;

    std::cout << "postProcess time: " << std::chrono::duration<double, std::milli>(t4 - t3).count() << " ms" << std::endl;

    std::cout << "total reconstructPoint time: " << std::chrono::duration<double, std::milli>(t4 - t0).count() << " ms" << std::endl;
}
