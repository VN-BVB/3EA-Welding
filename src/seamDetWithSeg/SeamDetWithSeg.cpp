#include "SeamDetWithSeg.h"

#include "deepLearning/segment/yolo11/Yolo11SegInference.h"
#include "settingPara/SettingPara.h"
#include "structLightCamera/config/StructLightConfig.h"
#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

SeamDetWithSeg::SeamDetWithSeg(QObject* parent) : QObject{parent}, structLightConfig(StructLightConfig::getInstance()) {
    // ******************************* 初始化实例分割和参数 *******************************
    this->initSegment();
    this->initPara();

    // ******************************* 创建结果保存文件夹 *******************************
    // 获取当前日期
    time_t now = time(0);
    tm* ltm = localtime(&now);
    // 格式化日期
    char date_str[9];
    snprintf(date_str, sizeof(date_str), "%04d%02d%02d", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday);
    // 指定父文件夹路径
    std::string parent_folder = "./data/seamDetWithSeg/";
    std::string target_folder = parent_folder + date_str;
    if (_access(target_folder.c_str(), 0) == -1) {  // 检查目标文件夹是否存在
        if (_mkdir(target_folder.c_str()) == 0) {   // 如果不存在，则创建文件夹
            PLOGD << "文件夹创建成功: " << target_folder;
            SetFileAttributesA(target_folder.c_str(), FILE_ATTRIBUTE_NORMAL);
        } else {
            PLOGE << "创建文件夹失败: " << target_folder;
        }
    } else {
        PLOGD << "文件夹已存在: " << target_folder;
    }
    // 确保文件夹权限为读写
    DWORD folderAttributes = GetFileAttributesA(target_folder.c_str());
    if (folderAttributes != INVALID_FILE_ATTRIBUTES && (folderAttributes & FILE_ATTRIBUTE_READONLY)) {
        // 如果文件夹是只读的，移除只读属性
        SetFileAttributesA(target_folder.c_str(), folderAttributes & ~FILE_ATTRIBUTE_READONLY);
        PLOGD << "文件夹权限已更改为读写: " << target_folder;
    }
}

// 初始化分割类
void SeamDetWithSeg::initSegment() {
    weldsSegmentation = std::make_shared<Yolo11SegInference>();
    std::vector<std::string> className = {"back_corner", "front_corner", "back_beam", "front_beam"};
    std::vector<std::vector<unsigned int>> colors = {
        {0,   128, 255},
        {128, 255, 0  },
        {255, 0,   128},
        {0,   255, 128}
    };

    weldsSegmentation->setEngine_path(segEnginePath);
    weldsSegmentation->setClassNames(className);
    weldsSegmentation->setColors(colors);
    weldsSegmentation->setScore_thres(0.25);
    weldsSegmentation->setIou_thres(0.1f);
    weldsSegmentation->setSeg_channels(32);
    weldsSegmentation->setSize(cv::Size(1024, 1024));

    weldsSegmentation->initialization();  // 完成类的初始化 (读取模型文件, 移入显卡等)
}

// 初始化参数
void SeamDetWithSeg::initPara() {
    this->fx = structLightConfig.Kc.at<double>(0, 0);
    this->fy = structLightConfig.Kc.at<double>(1, 1);
    this->u0 = structLightConfig.Kc.at<double>(0, 2);
    this->v0 = structLightConfig.Kc.at<double>(1, 2);
}

// 将分割结果绘制到和原图一样大的背景
void SeamDetWithSeg::drawMaskToBackground(const SegResult& res, cv::Mat& output) {
    const SegResult& result = res;                           // 获取第一个结果
    output = cv::Mat::zeros(result.segRes.size(), CV_8UC1);  // 创建与segRes同尺寸的黑色背景

    int x1 = static_cast<int>(result.topLeftX);  // 转换坐标为整数 (像素坐标必须是整数)
    int y1 = static_cast<int>(result.topLeftY);
    int x2 = static_cast<int>(result.bottomRightX);
    int y2 = static_cast<int>(result.bottomRightY);

    if (x1 >= x2 || y1 >= y2) {  // 检查坐标有效性
        std::cerr << "Error: Invalid bounding box coordinates!" << std::endl;
        return;
    }

    int width = x2 - x1;  // 计算ROI宽高
    int height = y2 - y1;

    if (result.maskOnly.cols != width || result.maskOnly.rows != height) {  // 检查mask尺寸是否匹配
        std::cerr << "Error: Mask size doesn't match bounding box!" << std::endl;
        return;
    }

    // 创建ROI (带边界检查)
    cv::Rect roi_rect(std::max(0, x1), std::max(0, y1), std::min(width, output.cols - x1), std::min(height, output.rows - y1));

    cv::Mat thresholded_mask;  // 阈值处理 (250以上设为255，其他保持0)
    cv::threshold(result.maskOnly, thresholded_mask, 250, 255, cv::THRESH_BINARY);

    cv::Mat destination_roi = output(roi_rect);  // 将处理后的mask复制到目标位置
    thresholded_mask(cv::Rect(0, 0, roi_rect.width, roi_rect.height)).copyTo(destination_roi);
}

// 求解焊缝
void SeamDetWithSeg::whenDetSeamWithSeg(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    PLOGD << "分割方法找焊缝类 收到焊缝数量: " << weldSeamInfo.size();

    // 遍历所有焊缝信息, 找到对接焊缝, 执行分割方法逻辑
    for (auto& info : weldSeamInfo) {
        if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT || info->weldType == WELD_TYPE::BACK_CORNER_BUTT ||
            info->weldType == WELD_TYPE::FRONT_BEAM_BUTT || info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
            std::vector<SegResult> res;
            weldsSegmentation->inference(info->weldAreaImg, res);  // 实例分割推理

            if (res.size() > 1) {  // 如果有多个结果, 取置信度最高的一个结果
                std::sort(res.begin(), res.end(), [](const SegResult& a, const SegResult& b) { return a.score > b.score; });
                res.resize(1);
            } else if (res.size() == 0) {  // 如果没有结果, 跳过当前焊缝
                continue;
            }

            // 保存分割结果
            info->segResultImg = res[0].segRes;              // 保存分割结果
            drawMaskToBackground(res[0], info->segMaskImg);  // 保存分割掩膜

            auto now = std::chrono::system_clock::now();                         // 获取当前时间点
            std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);  // 转换为time_t类型
            struct tm now_tm;                                                    // 转换为tm结构体
            localtime_s(&now_tm, &now_time_t);                                   // 使用localtime_s（Windows）
            std::stringstream ss;                                                // 进行格式化
            ss << std::put_time(&now_tm, "%Y%m%d-%H%M%S");
            std::string time_str = ss.str();  // 获取格式化的时间字符串
            std::stringstream Ymd;            // 进行格式化
            Ymd << std::put_time(&now_tm, "%Y%m%d");
            std::string Ymd_str = Ymd.str();  // 获取格式化的时间字符串
            if (!info->segResultImg.empty()) {
                cv::imwrite("./data/seamDetWithSeg/" + Ymd_str + "/" + time_str + " segResult" + std::to_string(imgNum) + ".bmp", info->segResultImg);
            }
            if (!info->segMaskImg.empty()) {
                cv::imwrite("./data/seamDetWithSeg/" + Ymd_str + "/" + time_str + " segMask" + std::to_string(imgNum++) + ".bmp", info->segMaskImg);
            }

            // 将当前焊缝的分割计算放入线程池
            threadPool->addTask([this, info]() { return this->detectSignalSeamWithSeg(info); });
        } else if (info->weldType == TubeSide_Plate_F_H || info->weldType == Plate_Plate_Fillet_V || info->weldType == Plate_Plate_Fillet_H) {
            PLOGD << "龙门支架目前未使用分割算法";
        }
    }

    threadPool->waitAll();  // 等待所有线程执行结束
    PLOGD << "分割方法找焊缝计算结束";

    // 打印焊缝信息
    for (auto& info : weldSeamInfo) {
        if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT || info->weldType == WELD_TYPE::BACK_CORNER_BUTT ||
            info->weldType == WELD_TYPE::FRONT_BEAM_BUTT || info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
            PLOGD << "区域编号: " << info->areaNum << ", 焊缝类型: " << MyToolFunc::getWeldTypeString(info->weldType);
            if (info->weldEndPointsFromSeg && info->weldEndPointsFromSeg->size() == 2) {
                PLOGD << "   分割得到的焊缝端点坐标: (" << info->weldEndPointsFromSeg->at(0).x << " " << info->weldEndPointsFromSeg->at(0).y << " "
                      << info->weldEndPointsFromSeg->at(0).z << ") (" << info->weldEndPointsFromSeg->at(1).x << " "
                      << info->weldEndPointsFromSeg->at(1).y << " " << info->weldEndPointsFromSeg->at(1).z << ")";
                PLOGD << "   焊缝宽度: " << info->width;
            }
        }
    }

    fusionPointCloudAndSegRes(weldSeamInfo);  // 融合点云和分割结果

    // 发出求解出的焊缝结果
    emit sendDetSeamWithSeg(weldSeamInfo);
}

// 融合点云和分割的计算结果
void SeamDetWithSeg::fusionPointCloudAndSegRes(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT || info->weldType == WELD_TYPE::BACK_CORNER_BUTT ||
            info->weldType == WELD_TYPE::FRONT_BEAM_BUTT || info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
            PLOGD << "执行点云以及分割方法检测到的焊缝融合逻辑... ...";
            if (info->weldEndPointsFromSeg != nullptr && info->weldEndPointsFromSeg->size() == 2 &&
                info->weldEndPointsFromSeg->at(0).z != 0) {  // 分割方法检测成功
                info->width = std::min(info->width, 5.0);    // 焊缝宽度规范化
                info->width = std::max(info->width, 0.1);
                if (info->detectSuccFlag == true) {  // 点云方法也检测成功了
                    PLOGD << "点云检测得到的焊缝: " << info->weldEndPointsInCamera->at(0) << " " << info->weldEndPointsInCamera->at(1);
                    PLOGD << "分割得到的焊缝: " << info->weldEndPointsFromSeg->at(0) << " " << info->weldEndPointsFromSeg->at(1);

                    if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                        // 先验证点云方法是否正确
                        double d1 = 0, d2 = 0;
                        d1 = MyToolFunc::getPoint2LineDis(info->weldEndPointsInCamera->at(0), info->seamsLineToVal);
                        d2 = MyToolFunc::getPoint2LineDis(info->weldEndPointsInCamera->at(1), info->seamsLineToVal);
                        if (d1 < 3 && d2 < 3) {  // 点云结果通过验证, 用点云结果验证分割结果
                            PLOGD << "点云焊缝点通过验证, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                            double startDistance = 0, endDistance = 0;  // 起终点距离
                            double num0AndNum0Dis = pcl::euclideanDistance(info->weldEndPointsFromSeg->at(0), info->weldEndPointsInCamera->at(0));
                            double num0AndNum1Dis = pcl::euclideanDistance(info->weldEndPointsFromSeg->at(0), info->weldEndPointsInCamera->at(1));
                            double num1AndNum0Dis = pcl::euclideanDistance(info->weldEndPointsFromSeg->at(1), info->weldEndPointsInCamera->at(0));
                            double num1AndNum1Dis = pcl::euclideanDistance(info->weldEndPointsFromSeg->at(1), info->weldEndPointsInCamera->at(1));
                            if (num0AndNum0Dis < num0AndNum1Dis) {
                                startDistance = num0AndNum0Dis;
                                endDistance = num1AndNum1Dis;
                            } else {
                                startDistance = num0AndNum1Dis;
                                endDistance = num1AndNum0Dis;
                            }
                            if (startDistance < 4 && endDistance < 4) {  // 分割焊缝在点云焊缝附近，认为分割结果也可取
                                pcl::ModelCoefficients::Ptr lineCoeff(new pcl::ModelCoefficients());  // 构造分割结果直线
                                Eigen::Vector3d lineVector(info->weldEndPointsFromSeg->at(0).x - info->weldEndPointsFromSeg->at(1).x,
                                                           info->weldEndPointsFromSeg->at(0).y - info->weldEndPointsFromSeg->at(1).y,
                                                           info->weldEndPointsFromSeg->at(0).z - info->weldEndPointsFromSeg->at(1).z);
                                lineVector /= lineVector.norm();
                                lineCoeff->values.resize(6);
                                lineCoeff->values[0] = info->weldEndPointsFromSeg->at(0).x;
                                lineCoeff->values[1] = info->weldEndPointsFromSeg->at(0).y;
                                lineCoeff->values[2] = info->weldEndPointsFromSeg->at(0).z;
                                lineCoeff->values[3] = lineVector[0];
                                lineCoeff->values[4] = lineVector[1];
                                lineCoeff->values[5] = lineVector[2];

                                // 计算点云焊缝点到分割焊缝点所在直线的投影
                                pcl::PointXYZ projPt1 = MyToolFunc::projPoint2Line(info->weldEndPointsInCamera->at(0), lineCoeff);
                                pcl::PointXYZ projPt2 = MyToolFunc::projPoint2Line(info->weldEndPointsInCamera->at(1), lineCoeff);
                                info->weldEndPointsInCamera->at(0) = projPt1;  // 采用新得到的点替代点云检测结果
                                info->weldEndPointsInCamera->at(1) = projPt2;

                                PLOGD << "采纳分割得到的焊缝, 验证距离: " << startDistance << " " << endDistance;
                            } else {
                                PLOGD << "不采纳分割得到的焊缝, 继续使用点云结果, 验证距离: " << startDistance << " " << endDistance;
                            }
                        } else {  // 点云结果未通过验证, 继续验证分割结果
                            PLOGW << "点云焊缝点未通过验证, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                            d1 = MyToolFunc::getPoint2LineDis(info->weldEndPointsFromSeg->at(0), info->seamsLineToVal);
                            d2 = MyToolFunc::getPoint2LineDis(info->weldEndPointsFromSeg->at(1), info->seamsLineToVal);
                            if (d1 < 3 && d2 < 3) {  // 分割方法焊缝通过验证, 取代点云方法焊缝点
                                info->weldEndPointsInCamera->at(0) = info->weldEndPointsFromSeg->at(0);
                                info->weldEndPointsInCamera->at(1) = info->weldEndPointsFromSeg->at(1);
                                PLOGD << "分割焊缝点通过验证, 取代点云得到的焊缝, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                            } else {
                                info->detectSuccFlag = false;
                                PLOGW << "分割焊缝点也未通过验证, 标志位置为false, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                            }
                        }
                    }
                } else {  // 点云方法检测失败了
                    if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                        double d1 = 0, d2 = 0;
                        d1 = MyToolFunc::getPoint2LineDis(info->weldEndPointsFromSeg->at(0), info->seamsLineToVal);
                        d2 = MyToolFunc::getPoint2LineDis(info->weldEndPointsFromSeg->at(1), info->seamsLineToVal);
                        if (d1 < 3 && d2 < 3) {  // 分割方法焊缝通过验证, 取代点云方法焊缝点
                            info->detectSuccFlag = true;
                            info->weldEndPointsInCamera->at(0) = info->weldEndPointsFromSeg->at(0);
                            info->weldEndPointsInCamera->at(1) = info->weldEndPointsFromSeg->at(1);
                            PLOGD << "分割焊缝点通过验证, 采纳分割得到的焊缝, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                        } else {
                            info->detectSuccFlag = false;
                            PLOGW << "分割焊缝点未通过验证, 不采纳分割得到的焊缝, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                        }
                    } else {
                        // 暂时认为只有分割的结果不可信, 无需操作
                    }
                }
            } else {  // 分割方法检测失败, 只验证点云方法的正确性
                if (info->detectSuccFlag == true) {
                    if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {  // 验证数据符合要求
                        double d1 = 0, d2 = 0;
                        d1 = MyToolFunc::getPoint2LineDis(info->weldEndPointsInCamera->at(0), info->seamsLineToVal);
                        d2 = MyToolFunc::getPoint2LineDis(info->weldEndPointsInCamera->at(1), info->seamsLineToVal);
                        if (d1 < 3 && d2 < 3) {  // 通过验证, 无需进行操作
                            PLOGD << "点云焊缝点通过验证, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                        } else {  // 未通过验证, 说明检测错了, 焊缝检测标志位置为false
                            info->detectSuccFlag = false;
                            PLOGW << "点云焊缝点未通过验证, 焊缝点到验证直线之间的距离d1: " << d1 << " d2: " << d2;
                        }
                    } else {  // 验证数据不符合要求, 认为检测结果不可信
                        info->detectSuccFlag = false;
                    }
                }
            }
        } else if (info->weldType == TubeSide_Plate_F_H || info->weldType == Plate_Plate_Fillet_V || info->weldType == Plate_Plate_Fillet_H) {
            PLOGD << "龙门支架目前不需要融合";
        }
    }
}

// 使用分割方法计算单一焊缝
void SeamDetWithSeg::detectSignalSeamWithSeg(std::shared_ptr<WeldSeamInfo> seamInfo) {
    PLOGD << "分割方法计算 " << MyToolFunc::getWeldTypeString(seamInfo->weldType) << " 类型焊缝";

    if (seamInfo->weldPlane == nullptr || seamInfo->weldPlane->values.size() != 4) {
        PLOGE << "焊缝所在平面异常, 无法使用分割方法计算焊缝";
        pcl::PointXYZ p;
        std::vector<pcl::PointXYZ> endOfOneSeam;
        p.x = p.y = p.z = 0;
        endOfOneSeam.push_back(p);
        endOfOneSeam.push_back(p);
        seamInfo->weldEndPointsFromSeg.reset(new std::vector<pcl::PointXYZ>(std::move(endOfOneSeam)));
        return;
    }

    // ****************** 提取并保存所有属于焊缝的像素点坐标 ******************
    std::vector<cv::Point> whitePixels;
    cv::Mat objectDetectImgMask = seamInfo->segMaskImg;
    for (int y = 0; y < objectDetectImgMask.rows; ++y) {
        for (int x = 0; x < objectDetectImgMask.cols; ++x) {
            if (!objectDetectImgMask.empty()) {
                if (objectDetectImgMask.at<uchar>(y, x) >= 250) {                                          // 判断像素值是否为白色
                    whitePixels.push_back(cv::Point(x + seamInfo->rectPtr->x, y + seamInfo->rectPtr->y));  // 保存坐标
                }
            }
        }
    }
    PLOGD << "白色像素提取成功, 像素数量: " << whitePixels.size();

    // ****************** 提取并保存所有用于焊缝宽度计算的像素点坐标 ******************
    std::vector<cv::Point> whitePixelsForWidth;
    bool firstWhiteRowsFlag = 0;
    for (int y = 0; y < objectDetectImgMask.rows; ++y) {
        for (int x = 0; x < objectDetectImgMask.cols; ++x) {
            if (!objectDetectImgMask.empty()) {
                if (objectDetectImgMask.at<uchar>(y, x) >= 250) {  // 判断像素值是否为白色
                    if (firstWhiteRowsFlag == 0) {
                        y = y + 40;  // 发现上边缘，直接跳到偏中间一点的位置
                        if (y >= objectDetectImgMask.rows) break;
                        firstWhiteRowsFlag = 1;
                    } else {
                        whitePixelsForWidth.push_back(cv::Point(x + seamInfo->rectPtr->x, y + seamInfo->rectPtr->y));  // 保存坐标
                    }
                }
            }
        }
    }
    PLOGD << "用于宽度计算的白色像素提取成功, 像素数量: " << whitePixelsForWidth.size();

    // ****************** 二维像素映射到三维点 ******************
    Plane p(seamInfo->weldPlane->values[0] / ((-1) * seamInfo->weldPlane->values[3]),
            seamInfo->weldPlane->values[1] / ((-1) * seamInfo->weldPlane->values[3]),
            seamInfo->weldPlane->values[2] / ((-1) * seamInfo->weldPlane->values[3]));
    pcl::PointCloud<pcl::PointXYZ>::Ptr weldSeamPointClouds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr weldSeamPointCloudsForWidth(new pcl::PointCloud<pcl::PointXYZ>);
    bool res2dTo3d = point2dTo3d(whitePixels, weldSeamPointClouds, p);
    point2dTo3d(whitePixels, weldSeamPointCloudsForWidth, p);
    if (res2dTo3d == false) {
        PLOGE << "焊缝二维像素点转三维空间点失败, 无法使用分割方法计算焊缝";
        pcl::PointXYZ p;
        std::vector<pcl::PointXYZ> endOfOneSeam;
        p.x = p.y = p.z = 0;
        endOfOneSeam.push_back(p);
        endOfOneSeam.push_back(p);
        seamInfo->weldEndPointsFromSeg.reset(new std::vector<pcl::PointXYZ>(std::move(endOfOneSeam)));
        return;
    }
    PLOGD << "焊缝二维像素点转三维空间点成功, 三维点数: " << weldSeamPointClouds->size();
    weldSeamPointClouds->height = 1;
    weldSeamPointClouds->width = static_cast<uint32_t>(weldSeamPointClouds->size());

    auto now = std::chrono::system_clock::now();                         // 获取当前时间点
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);  // 转换为time_t类型
    struct tm now_tm;                                                    // 转换为tm结构体
    localtime_s(&now_tm, &now_time_t);                                   // 使用localtime_s（Windows）
    std::stringstream ss;                                                // 进行格式化
    ss << std::put_time(&now_tm, "%Y%m%d-%H%M%S");
    std::string time_str = ss.str();  // 获取格式化的时间字符串
    std::stringstream Ymd;            // 进行格式化
    Ymd << std::put_time(&now_tm, "%Y%m%d");
    std::string Ymd_str = Ymd.str();  // 获取格式化的时间字符串
    if (weldSeamPointClouds->size() > 0) {
        pcl::io::savePCDFile("./data/seamDetWithSeg/" + Ymd_str + "/" + time_str + " seamPointCloud" + std::to_string(pointCloudNum++) + ".pcd",
                             *weldSeamPointClouds);
    }

    // ****************** 拟合焊缝三维点为直线 ******************
    pcl::ModelCoefficients::Ptr lineCoeff(new pcl::ModelCoefficients());  // 拟合出来的直线参数
    if (weldSeamPointClouds->points.size() == 0) {
        lineCoeff->values.push_back(0);
        lineCoeff->values.push_back(0);
        lineCoeff->values.push_back(0);
        PLOGW << "焊缝点云数量为零, 无法拟合直线";
    } else {
        pcl::SACSegmentation<pcl::PointXYZ> seg;                  // 创建拟合对象
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  // 内点索引
        seg.setOptimizeCoefficients(true);                        // 设置对估计模型参数进行优化处理
        seg.setModelType(pcl::SACMODEL_LINE);                     // 设置拟合模型为直线模型
        seg.setMethodType(pcl::SAC_RANSAC);                       // 设置拟合方法为RANSAC
        seg.setMaxIterations(300);                                // 设置最大迭代次数
        seg.setDistanceThreshold(6);                              // 判断是否为模型内点的距离阀值/设置误差容忍范围
        seg.setInputCloud(weldSeamPointClouds);                   // 输入点云
        seg.segment(*inliers, *lineCoeff);                        // 内点的索引，模型系数

        PLOGD << "分割方法检测焊缝拟合的焊缝直线: x:" << lineCoeff->values[0] << " y:" << lineCoeff->values[1] << " z:" << lineCoeff->values[2]
              << " a:" << lineCoeff->values[3] << " b:" << lineCoeff->values[4] << " c:" << lineCoeff->values[5]
              << "  内点数量:" << inliers->indices.size();
    }

    // ****************** 将焊缝点投影到直线得到端点即为焊缝端点 ******************
    if (lineCoeff->values[0] != 0 && lineCoeff->values[1] != 0 && lineCoeff->values[2] != 0) {
        std::vector<pcl::PointXYZ> endOfOneSeam;
        endOfOneSeam = MyToolFunc::lineCloudEndPoints(weldSeamPointClouds, lineCoeff);
        seamInfo->weldEndPointsFromSeg.reset(new std::vector<pcl::PointXYZ>(std::move(endOfOneSeam)));
        PLOGD << "分割方法求解焊缝端点结果: " << seamInfo->weldEndPointsFromSeg->at(0) << " " << seamInfo->weldEndPointsFromSeg->at(1);
    } else {
        pcl::PointXYZ p;
        std::vector<pcl::PointXYZ> endOfOneSeam;
        p.x = p.y = p.z = 0;
        endOfOneSeam.push_back(p);
        endOfOneSeam.push_back(p);
        seamInfo->weldEndPointsFromSeg.reset(new std::vector<pcl::PointXYZ>(std::move(endOfOneSeam)));
        PLOGW << "分割方法求解焊缝端点失败, 存入: " << seamInfo->weldEndPointsFromSeg->at(0) << " " << seamInfo->weldEndPointsFromSeg->at(1);
    }

    // ****************** 计算焊缝宽度 ******************
    double singleSeamWidth = 1.5;
    if (lineCoeff->values[0] != 0 && lineCoeff->values[1] != 0 && lineCoeff->values[2] != 0) {
        singleSeamWidth = 0.0;
        for (const auto& point : *weldSeamPointCloudsForWidth) {
            Eigen::Vector4f p(point.x, point.y, point.z, 0);
            double pDis = MyToolFunc::getPoint2LineDis(p, lineCoeff);
            singleSeamWidth += pDis;
        }
        singleSeamWidth /= weldSeamPointCloudsForWidth->size();  // 点到直线距离均值
        singleSeamWidth *= 4;                                    // 获得焊缝宽度
        if (seamInfo->weldType == WELD_TYPE::BACK_CORNER_BUTT || seamInfo->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
            singleSeamWidth *= 0.8;
        } else if (seamInfo->weldType == WELD_TYPE::BACK_BEAM_BUTT || seamInfo->weldType == WELD_TYPE::FRONT_BEAM_BUTT) {
            singleSeamWidth *= 1.2;
        }
        seamInfo->width = singleSeamWidth;
        PLOGD << "焊缝宽度: " << singleSeamWidth;
    } else {  // 前面的步骤直线拟合失败，直接取默认焊缝宽度为1.5
        seamInfo->width = 1.5;
        PLOGW << "分割方法计算焊缝宽度失败, 存入: " << 1.5;
    }
}

// 二维像素映射到三维点
bool SeamDetWithSeg::point2dTo3d(std::vector<cv::Point>& pixelSet, pcl::PointCloud<pcl::PointXYZ>::Ptr& seamPointsSet, Plane& p) {
    if (pixelSet.size() > 0) {
        const double a = p.A;
        const double b = p.B;
        const double c = p.C;

        for (const auto& p : pixelSet) {
            double x = 0, y = 0, z = 0;                     // 三维点坐标
            double u = p.x, v = p.y;                        // 像素坐标
            double xp = (u - u0) / fx, yp = (v - v0) / fy;  // 图像坐标

            x = xp / (a * xp + b * yp + c);
            y = yp / (a * xp + b * yp + c);
            z = 1 / (a * xp + b * yp + c);

            pcl::PointXYZ point3d(x, y, z);
            seamPointsSet->points.push_back(point3d);
        }
    } else {
        return false;
    }

    SettingPara& set = SettingPara::getInstance();
    MyToolFunc::scalePointClouds(seamPointsSet, set.scaleOfPointX, set.transOfPointX, set.scaleOfPointY, set.transOfPointY);

    return true;
}
