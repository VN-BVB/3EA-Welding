#ifndef COMMON_HPP
#define COMMON_HPP

#include "src/stable.h"
#include "structLightCamera/StructLightCamera.h"

std::vector<std::vector<double>> Visual_RGBList = {
    {0.8,  1,    1   },
    {1,    1,    0.8 },
    {0.77, 0.5,  0.93},
    {0.47, 0.92, 0.77},
    {1,    0.8,  1   },
    {0.77, 0.93, 0.47},
    {0.47, 0.77, 0.93},
    {0.93, 0.47, 0.77},
    {0.93, 0.77, 0.47}
};  // 点云可视化颜色列表

void handleWeldAreaInfo2Display(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo,
                                boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVisualizer, cv::Mat& ObjDetImg) {
    // VTK界面点云显示
    pclVisualizer->removeAllShapes();       // 清空上次的shape显示
    pclVisualizer->removeAllPointClouds();  // 清空点云

    // 焊缝区域点云可视化
    std::set<int> seamAreaPointCloudNum;                                                  // 已显示的焊缝区域点云序号
    pcl::PointCloud<pcl::PointXYZ>::Ptr visualCloud(new pcl::PointCloud<pcl::PointXYZ>);  // 用于显示的点云
    for (auto& info : weldAreaInfo) {
        auto cloud = info->weldAreaPointCloudInRobot;
        if (info->detectSuccFlag == true && cloud && !cloud->empty()) {
            if (seamAreaPointCloudNum.find(info->areaNum) == seamAreaPointCloudNum.end()) {  // 当前区域点云还未显示
                seamAreaPointCloudNum.insert(info->areaNum);
                std::string label_cloud = "label_cloud" + std::to_string(info->areaNum);
                pclVisualizer->addPointCloud(cloud, label_cloud);
                pclVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, Visual_RGBList[info->areaNum][0],
                                                                Visual_RGBList[info->areaNum][1], Visual_RGBList[info->areaNum][2], label_cloud);
                pclVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7,
                                                                label_cloud);  // 不透明度
                *visualCloud = *visualCloud + *(cloud);
            }
        }
    }

    // 焊缝可视化
    int seamsNum = 0;
    auto drawSeamLine = [&pclVisualizer](const std::shared_ptr<std::vector<pcl::PointXYZ>>& points, const std::string& labelPrefix, int seamIndex,
                                         double r, double g, double b) {
        if (!points || points->size() < 2) return;

        if (points->size() == 2) {
            std::string label = labelPrefix + std::to_string(seamIndex);
            pclVisualizer->addLine(points->at(0), points->at(1), r, g, b, label);
            pclVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, label);
            return;
        }

        for (int i = 0; i < static_cast<int>(points->size()) - 1; ++i) {
            std::string label = labelPrefix + std::to_string(seamIndex) + "_" + std::to_string(i);
            pclVisualizer->addLine(points->at(i), points->at(i + 1), r, g, b, label);
            pclVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 8, label);
        }
    };

    for (auto& info : weldAreaInfo) {
        if (!info || !info->detectSuccFlag || !info->weldEndPointsInRobot || info->weldEndPointsInRobot->empty()) continue;
        drawSeamLine(info->weldEndPointsInRobotRaw, "label_RawSeam_", seamsNum, 1, 0, 0);
        drawSeamLine(info->weldEndPointsInRobot, "label_CompensatedSeam_", seamsNum, 0, 0, 1);

        int endPtSz = info->weldEndPointsInRobot->size();
        if (endPtSz == 2) {
            // 焊缝宽度显示
            if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT || info->weldType == WELD_TYPE::BACK_CORNER_BUTT ||
                info->weldType == WELD_TYPE::FRONT_BEAM_BUTT || info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
                pcl::PointXYZ seamWidthStart((info->weldEndPointsInRobot->at(0).x + info->weldEndPointsInRobot->at(1).x) / 2,
                                             (info->weldEndPointsInRobot->at(0).y + info->weldEndPointsInRobot->at(1).y) / 2,
                                             (info->weldEndPointsInRobot->at(0).z + info->weldEndPointsInRobot->at(1).z) / 2);
                double seamWidth = info->width;
                double orientation[3] = {175, -9, 0};
                std::string width = "Width: " + std::to_string(seamWidth);
                std::string lable_SeamWidth = "lable_SeamWidth: " + std::to_string(seamWidth) + " " + std::to_string(seamsNum);
                pclVisualizer->addText3D(width, seamWidthStart, orientation, 5.0, 1.0, 0.5, 0.0, lable_SeamWidth);
            }
            seamsNum++;
        } else if (endPtSz > 2) {
            seamsNum++;
        }
    }

    // 目标检测结果可视化
    std::set<int> objDetRectNum;
    if (!weldAreaInfo.empty()) {
        ObjDetImg = weldAreaInfo[0]->originalImg.clone();
        PLOGD << "【调试】开始画框，weldAreaInfo.size()=" << weldAreaInfo.size();

        for (auto& info : weldAreaInfo) {
            if (objDetRectNum.find(info->areaNum) == objDetRectNum.end()) {  // 当前区域目标框还未显示
                objDetRectNum.insert(info->areaNum);
                PLOGD << "【调试】处理区域 areaNum=" << info->areaNum << ", rectPtr=" << (info->rectPtr ? "有效" : "无效");

                if (info->rectPtr != nullptr) {
                    bool currAreaSeams = false;
                    PLOGD << "【调试】检查该区域是否有检测成功的焊缝...";
                    for (auto& i : weldAreaInfo) {
                        if (i->areaNum == info->areaNum) {
                            PLOGD << "  检查焊缝: detectSuccFlag=" << i->detectSuccFlag;
                            if (i->detectSuccFlag == true) {  // 当前区域只要有一条焊缝检测成功, 就标记为检测成功
                                currAreaSeams = true;
                                PLOGD << "  找到检测成功的焊缝，currAreaSeams=true";
                                break;
                            }
                        }
                    }
                    cv::Scalar roi_color;
                    std::string label;

                    // ================= 1. 判断是否有效 =================
                    if (currAreaSeams) {
                        auto type = info->weldAreaType;
                        PLOGD << "【调试】画成功框，weldAreaType=" << static_cast<int>(type);

                        // ================= 2. 统一索引 =================
                        int colorIdx = static_cast<int>(type);

                        // 处理 -100 偏移类型
                        if (colorIdx >= 100) colorIdx -= 100;

                        // ================= 3. 颜色 + 文本 =================
                        roi_color =
                            cv::Scalar(Visual_RGBList[colorIdx][0] * 255, Visual_RGBList[colorIdx][1] * 255, Visual_RGBList[colorIdx][2] * 255);

                        label = MyToolFunc::getWeldAreaTypeString(type);

                    } else {
                        PLOGD << "【调试】画None框，区域 areaNum=" << info->areaNum;

                        roi_color = cv::Scalar(0, 0, 0);
                        label = "None";
                    }

                    // ================= 4. 统一绘制 =================
                    cv::rectangle(ObjDetImg, *(info->rectPtr), roi_color, currAreaSeams ? 8 : 10);

                    cv::putText(ObjDetImg, label, cv::Point(info->rectPtr->x, info->rectPtr->y - 10), cv::FONT_HERSHEY_COMPLEX,
                                currAreaSeams ? 1.2 : 1.5, roi_color, currAreaSeams ? 4 : 6);
                } else {
                    PLOGW << "【调试】rectPtr为空，无法画框，区域 areaNum=" << info->areaNum;
                }
            }
        }
        PLOGD << "【调试】画框完成";
    } else {
        PLOGW << "【调试】weldAreaInfo为空，无法画框";
    }
    // 获取点云边界框大小
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*visualCloud, minPt, maxPt);
    Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2,
                           (maxPt.z + minPt.z) / 2);  // 计算点云中心位置和对角线长度
    pclVisualizer->setCameraPosition(center(0), center(1), center(2) + 0.1, center(0), center(1), center(2), 1, 0, 0);

    // VTK界面点云显示
    // viewer_pcl->addCoordinateSystem(50);
    pclVisualizer->resetCamera();

    (void)weldAreaInfo;
}

#endif  // COMMON_HPP
