#include "DownBeamFilletSeamsDet.h"

#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

DownBeamFilletSeamsDet::DownBeamFilletSeamsDet(QObject *parent) : AbstractSeamDet{parent} {}

// 求解焊缝
std::vector<std::shared_ptr<WeldSeamInfo>> DownBeamFilletSeamsDet::solveSeamsEndPoints(
    std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    // tempWeldSeamsInfo = seamsInfo;  // 传入的焊缝信息
    tempWeldSeamsInfo.clear();

    // 焊缝求解
    for (int i = 0; i < seamsInfo.size(); i++) {
        SingleSeam_Reinitialize();  // 单条焊缝检测前，变量重新初始化

        cloud = seamsInfo[i]->weldAreaPointCloudInCamera;  // 获取焊缝区域点云

        if (cloud->size() == 0) {
            tempWeldSeamsInfo[i]->detectSuccFlag = false;
            continue;
        }

        Ransac_Multiple_planes(4);                          // 拟合三个平面
        bool bool_IdentifySeam = Solve_Fillet_Endpoints();  // 识别焊缝

        // 打印检测结果
        PLOGD << "正面倒立角接焊缝端点识别结果: " << bool_IdentifySeam;
        if (beamDownHorizonFilletSeams.size() == 2) {
            PLOGD << "水平焊缝端点坐标: (" << beamDownHorizonFilletSeams[0].x << " " << beamDownHorizonFilletSeams[0].y << " "
                  << beamDownHorizonFilletSeams[0].z << ") (" << beamDownHorizonFilletSeams[1].x << " "
                  << beamDownHorizonFilletSeams[1].y << " " << beamDownHorizonFilletSeams[1].z << ")";
        }
        if (beamDownVerticalFilletSeams.size() == 2) {
            PLOGD << "竖直焊缝端点坐标: (" << beamDownVerticalFilletSeams[0].x << " " << beamDownVerticalFilletSeams[0].y << " "
                  << beamDownVerticalFilletSeams[0].z << ") (" << beamDownVerticalFilletSeams[1].x << " "
                  << beamDownVerticalFilletSeams[1].y << " " << beamDownVerticalFilletSeams[1].z << ")";
        }

        // 保存本次检测到的信息, 水平焊缝直接放入原本的焊缝信息中
        seamsInfo[i]->detectSuccFlag = bool_IdentifySeam;  // 检测是否成功标志位
        if (bool_IdentifySeam == true) {
            // 角接焊缝暂时没有所在平面以及验证直线
            seamsInfo[i]->weldEndPointsInCamera.reset(
                new std::vector<pcl::PointXYZ>(std::move(beamDownHorizonFilletSeams)));  // 检测结果
            seamsInfo[i]->weldType = WELD_TYPE::FRONT_HORIZONTAL_FILLET;                 // 焊缝类型
        }
        tempWeldSeamsInfo.push_back(seamsInfo[i]);  // 保存焊缝信息

        // 竖直焊缝新建一个焊缝信息再放入
        auto newWeldSeamInfo = std::make_shared<WeldSeamInfo>();
        newWeldSeamInfo->areaNum = seamsInfo[i]->areaNum;                        // 区域编号
        newWeldSeamInfo->originalImg = seamsInfo[i]->originalImg;                // 原始图像
        newWeldSeamInfo->weldAreaImg = seamsInfo[i]->weldAreaImg;                // 焊缝区域图像
        newWeldSeamInfo->rectPtr = seamsInfo[i]->rectPtr;                        // 焊缝区域矩形框
        newWeldSeamInfo->weldAreaPointCloudInCamera = seamsInfo[i]->weldAreaPointCloudInCamera;  // 焊缝区域点云
        newWeldSeamInfo->weldAreaType = seamsInfo[i]->weldAreaType;              // 焊缝区域类型
        newWeldSeamInfo->detectSuccFlag = bool_IdentifySeam;                     // 检测是否成功标志位
        if (bool_IdentifySeam == true) {
            // 角接焊缝暂时没有所在平面以及验证直线
            newWeldSeamInfo->weldEndPointsInCamera.reset(
                new std::vector<pcl::PointXYZ>(std::move(beamDownVerticalFilletSeams)));  // 检测结果
            newWeldSeamInfo->weldType = WELD_TYPE::FRONT_VERTICAL_FILLET;
        }
        tempWeldSeamsInfo.push_back(newWeldSeamInfo);  // 保存焊缝信息
    }

    return tempWeldSeamsInfo;
}

// Ransac分割拟合多个平面
void DownBeamFilletSeamsDet::Ransac_Multiple_planes(int plane_nums) {
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);  // 模型系数
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                 // 索引列表
    // pcl::SACSegmentation<pcl::PointXYZ> seg;                               // 分割对象
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(Ransac_Plane_Iterations);
    // seg.setDistanceThreshold(Ransac_plane_Dth);
    // pcl::ExtractIndices<pcl::PointXYZ> extract;                                              // 提取器
    // pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>);  // 创建分割对象

    // vector<Eigen::Vector4f> centroids_Centroid_list;  // 平面质心Z方向列表

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);  // 将原始点云复制给cloud_ransac
    // pcl::copyPointCloud(*cloud, *cloud_ransac);
    // // Ransac算法提取多个平面，每次提取后进行法线方向翻转
    // for (int i = 0; i < plane_nums; i++) {
    //     seg.setInputCloud(cloud_ransac);       // 输入点云
    //     seg.segment(*inliers, *coefficients);  // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    //     extract.setInputCloud(cloud_ransac);
    //     extract.setIndices(inliers);
    //     extract.setNegative(false);
    //     extract.filter(*planar_segment);  // 提取探测出来的平面

    //     plane_clouds_list.push_back(*planar_segment);  // 提取各平面点云

    //     Eigen::Vector4f centroid;                           // 计算当前平面质心
    //     pcl::compute3DCentroid(*planar_segment, centroid);  // 质心
    //     centroids_Centroid_list.push_back(centroid);        // 提取各平面质心坐标

    //     // 使平面法线指向相机方向,相机视点v为(0,0,0), n*(v-p)
    //     double bool_overturn =
    //         coefficients->values[0] * (-centroid[0]) + coefficients->values[1] * (-centroid[1]) + coefficients->values[2] *
    //         (-centroid[2]);
    //     if (bool_overturn < 0) {
    //         coefficients->values[0] = -coefficients->values[0];
    //         coefficients->values[1] = -coefficients->values[1];
    //         coefficients->values[2] = -coefficients->values[2];
    //         coefficients->values[3] = -coefficients->values[3];
    //     }
    //     Coefficients_list.push_back(*coefficients);  // 提取各平面系数
    //     // 剔除探测出的平面，在剩余点中继续探测平面
    //     extract.setNegative(true);
    //     extract.filter(*cloud_ransac);
    // }

    // //------------------------------ 剔除与第一个平面最接近平行的平面的索引
    // float min_angle = std::numeric_limits<float>::max();
    // int index_of_parallel_plane = -1;
    // int base_index = 0;  // 基准平面
    // Eigen::Vector3f vector_base(Coefficients_list[0].values[0], Coefficients_list[0].values[1],
    // Coefficients_list[0].values[2]); for (size_t i = 1; i < Coefficients_list.size(); ++i) {
    //     Eigen::Vector3f vector_current(Coefficients_list[i].values[0], Coefficients_list[i].values[1],
    //     Coefficients_list[i].values[2]); float angle = getLineAngle(vector_base, vector_current); if (angle < min_angle) {
    //         min_angle = angle;
    //         index_of_parallel_plane = (int)i;
    //     }
    // }
    // Coefficients_list.erase(Coefficients_list.begin() + index_of_parallel_plane);

    // (void)base_index;

    // ------------------------------ 新算法 ------------------------------
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);  // 模型系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                 // 索引列表
    pcl::SACSegmentation<pcl::PointXYZ> seg;                               // 分割对象
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(Ransac_Plane_Iterations);
    seg.setDistanceThreshold(Ransac_plane_Dth);
    pcl::ExtractIndices<pcl::PointXYZ> extract;                                              // 提取器
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>);  // 创建分割对象

    std::vector<Eigen::Vector4f> centroids_Centroid_list;  // 平面质心Z方向列表

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);  // 将原始点云复制给cloud_ransac
    pcl::copyPointCloud(*cloud, *cloud_ransac);

    // 保存原始点云
    if (saveAndOutputDebugInformation == true) {
        cloud_ransac->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
        cloud_ransac->width = cloud_ransac->size();
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/Down/cloud.pcd", *cloud_ransac);
    }

    // Ransac算法提取多个平面，每次提取后进行法线方向翻转
    for (int i = 0; i < plane_nums; i++) {
        seg.setInputCloud(cloud_ransac);  // 输入点云
        seg.segment(*inliers, *coefficients);  // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
        extract.setInputCloud(cloud_ransac);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*planar_segment);  // 提取探测出来的平面

        plane_clouds_list.push_back(*planar_segment);  // 提取各平面点云

        Eigen::Vector4f centroid;                           // 计算当前平面质心
        pcl::compute3DCentroid(*planar_segment, centroid);  // 质心
        centroids_Centroid_list.push_back(centroid);        // 提取各平面质心坐标

        // 使平面法线指向相机方向,相机视点v为(0,0,0), n*(v-p)
        double bool_overturn = coefficients->values[0] * (-centroid[0]) + coefficients->values[1] * (-centroid[1]) +
                               coefficients->values[2] * (-centroid[2]);
        if (bool_overturn < 0) {
            coefficients->values[0] = -coefficients->values[0];
            coefficients->values[1] = -coefficients->values[1];
            coefficients->values[2] = -coefficients->values[2];
            coefficients->values[3] = -coefficients->values[3];
        }

        Coefficients_list.push_back(*coefficients);  // 提取各平面系数
        // 剔除探测出的平面内点，在剩余点中继续探测平面
        extract.setNegative(true);
        extract.filter(*cloud_ransac);
    }

    // 输出直线参数
    if (saveAndOutputDebugInformation == true) {
        std::cout << "Down Beam Debug Info:" << std::endl;
        for (int i = 0; i < Coefficients_list.size(); ++i) {
            std::cout << "A" << i + 1 << " = " << Coefficients_list[i].values[0] << "; B" << i + 1 << " = "
                      << Coefficients_list[i].values[1] << "; C" << i + 1 << " = " << Coefficients_list[i].values[2] << "; D"
                      << i + 1 << " = " << Coefficients_list[i].values[3] << ";" << "  内点数量：" << plane_clouds_list[i].size()
                      << std::endl;
        }
    }

    // ------------------------------找到几个相互平行的平面的索引，并去掉除了内点最多的那个平面以外的平面
    float min_angle = std::numeric_limits<float>::max();
    int index_of_parallel_plane = -1;
    std::vector<int> indexOfParallelPlanes;  // 几个相互平行的平面的索引
    indexOfParallelPlanes.push_back(0);
    Eigen::Vector3f vector_base(Coefficients_list[0].values[0], Coefficients_list[0].values[1], Coefficients_list[0].values[2]);
    for (int i = 1; i < Coefficients_list.size(); ++i) {
        Eigen::Vector3f vector_current(Coefficients_list[i].values[0], Coefficients_list[i].values[1],
                                       Coefficients_list[i].values[2]);
        float angle = MyToolFunc::getLineAngle(vector_base, vector_current);
        // 输出平面夹角信息
        // if (saveAndOutputDebugInformation == true) {
        //     std::cout << "第 " << i + 1 << " 个平面与第 1 个平面的夹角：" << angle << std::endl;
        // }
        // 保存认为是平行平面的平面索引
        if (angle < 10) {
            indexOfParallelPlanes.push_back(i);
            std::cout << "与第一面平行平面索引：" << i << std::endl;
        }
        if (angle < min_angle) {
            min_angle = angle;
            index_of_parallel_plane = i;
        }
    }

    int indexOfPlaneWithMostInliersNum = index_of_parallel_plane;  // 先用先前的算法赋值
    int mostInliersNum = 0;  // 内点最多的平面的内点数量                                   .
    for (int i = 0; i < indexOfParallelPlanes.size(); ++i) {
        if (plane_clouds_list[indexOfParallelPlanes[i]].size() > mostInliersNum) {
            mostInliersNum = plane_clouds_list[indexOfParallelPlanes[i]].size();
            indexOfPlaneWithMostInliersNum = indexOfParallelPlanes[i];
        }
    }
    // if (saveAndOutputDebugInformation == true) {
    //     std::cout << "内点最多的平面索引indexOfPlaneWithMostInliersNum：" << indexOfPlaneWithMostInliersNum << std::endl;
    // }

    // 去掉除内点最多的那个平面以外的平面
    // 将其中的元素从大到小排列，因为如果先删除了前面的，再删除后面的，后面的索引就会变化，进而导致误删了想要的
    std::sort(indexOfParallelPlanes.begin(), indexOfParallelPlanes.end(), std::greater<int>());
    for (int i = 0; i < indexOfParallelPlanes.size(); ++i) {
        if (indexOfParallelPlanes[i] != indexOfPlaneWithMostInliersNum) {
            std::cout << "删除平面 " << indexOfParallelPlanes[i] << std::endl;
            Coefficients_list.erase(Coefficients_list.begin() + indexOfParallelPlanes[i]);
            // plane_clouds_list.erase(plane_clouds_list.begin() + indexOfParallelPlanes[i]);
        }
    }

    // 去除后剩余的平面
    // if (saveAndOutputDebugInformation == true) {
    //     std::cout << "去除后剩余的平面：" << std::endl;
    //     for (int i = 0; i < Coefficients_list.size(); ++i) {
    //         std::cout << "\nA" << i + 1 << " = " << Coefficients_list[i].values[0] << ";\nB" << i + 1 << " = " <<
    //         Coefficients_list[i].values[1]
    //                   << ";\nC" << i + 1 << " = " << Coefficients_list[i].values[2] << ";\nD" << i + 1 << " = " <<
    //                   Coefficients_list[i].values[3]
    //                   << ";" << std::endl;
    //         // std::cout << "内点数量：" << plane_clouds_list[i].size() << std::endl;
    //     }
    // }

    // // 输出哪一个平面被去掉了
    // if (saveAndOutputDebugInformation == true) {
    //     std::cout << "Down Beam Debug Info:" << std::endl;
    //     std::cout << "去除的平面的参数序号index_of_parallel_plane+1: " << index_of_parallel_plane + 1 << std::endl;
    // }
    // // 去除和第一个平面最接近平行的平面
    // Coefficients_list.erase(Coefficients_list.begin() + index_of_parallel_plane);
}

// 提取焊缝
bool DownBeamFilletSeamsDet::Solve_Fillet_Endpoints() {
    // ------------------------------Step1 计算相机y轴与平面法向量之间的夹角。
    Eigen::Vector3f lineVec = {0, 1, 0};  // 或以相机y轴方向为指引
    // 初始化最小和最大角度以及对应的索引
    double minAngle = 180.0;  // 最大角度
    double maxAngle = 0.0;    // 最小角度
    int minIndex = -1;
    int maxIndex = -1;
    // 遍历平面的法向量
    for (size_t i = 1; i < Coefficients_list.size(); ++i) {
        // 获取平面的法向量
        Eigen::Vector3f normal(Coefficients_list[i].values[0], Coefficients_list[i].values[1], Coefficients_list[i].values[2]);
        double angle = MyToolFunc::getLineAngle(lineVec, normal);
        // 更新最小和最大角度以及对应的索引
        if (angle < minAngle) {
            minAngle = angle;
            minIndex = (int)i;
        }
        if (angle > maxAngle) {
            maxAngle = angle;
            maxIndex = (int)i;
        }
    }

    // ------------------------------Step2 求三面交点。
    // 三个平面的法线
    if (minIndex <= 0 || maxIndex <= 0) {
        PLOGE << "倒立横梁焊缝关键平面寻找失败";
        return false;
    }
    Eigen::Vector4f plane_base = {Coefficients_list[0].values[0], Coefficients_list[0].values[1], Coefficients_list[0].values[2],
                                  Coefficients_list[0].values[3]};
    Eigen::Vector4f plane_beam = {Coefficients_list[minIndex].values[0], Coefficients_list[minIndex].values[1],
                                  Coefficients_list[minIndex].values[2], Coefficients_list[minIndex].values[3]};
    Eigen::Vector4f plane_outer = {Coefficients_list[maxIndex].values[0], Coefficients_list[maxIndex].values[1],
                                   Coefficients_list[maxIndex].values[2], Coefficients_list[maxIndex].values[3]};
    // 三个平面法线的平均中心线
    // Eigen::Vector3f vector_ave_abc = (plane_a.head(3) + plane_b.head(3) + plane_c.head(3)) / 3.0;
    Eigen::Vector4f vector_ave_abc = (plane_base + plane_beam + plane_outer) / 3.0;
    // 求三个平面的交点
    Eigen::Vector3f Intersection_point;
    pcl::threePlanesIntersection(plane_base, plane_beam, plane_outer, Intersection_point, 1e-6);
    ThreePlanes_IntersectionPoint = {Intersection_point[0], Intersection_point[1], Intersection_point[2]};

    //-----------------------------Step3 求立焊缝的端点
    Eigen::VectorXf line_vertical_vector;  // 平面plane_beam和平面plane_outer的交线
    pcl::planeWithPlaneIntersection(plane_beam, plane_outer, line_vertical_vector, 0.1);  // 求平面b和平面c的相交直线
    if (line_vertical_vector.size() < 6) {
        PLOGE << "倒立横梁焊缝平面交线求解失败";
        return false;
    }
    Eigen::Vector4f pt_on_line_1 = {line_vertical_vector[0], line_vertical_vector[1], line_vertical_vector[2], 0};  // 直线上一点
    Eigen::Vector4f dir_line_1 = {line_vertical_vector[3], line_vertical_vector[4], line_vertical_vector[5], 0};  // 直线方向
    if (dir_line_1[0] * vector_ave_abc[0] + dir_line_1[1] * vector_ave_abc[1] + dir_line_1[2] * vector_ave_abc[2] <
        0) {  // 保持ab线方向与三平面法线方向一致
        dir_line_1 = -dir_line_1;
        line_vertical_vector.tail(3) = -line_vertical_vector.tail(3);
    }
    Eigen::Vector4f Intersection_point_4f = {Intersection_point[0], Intersection_point[1], Intersection_point[2],
                                             0};  // 转Vector4f
    // 提取近线点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NearVerticalLine(new pcl::PointCloud<pcl::PointXYZ>);  // 平面交线附近的点云
    for (int i = 0; i < cloud->size(); i++) {
        Eigen::Vector4f pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 0);
        double distance = sqrt(pcl::sqrPointToLineDistance(pt, pt_on_line_1, dir_line_1));
        if (distance < NearLine_DistanceThreshold) {
            cloud_NearVerticalLine->push_back(cloud->points[i]);
        }
    }
    // 按次序输出竖直交线的两个投影端点
    std::vector<Eigen::Vector4f> two_endpoints_VerticalSeam;
    MyToolFunc::lineCloudEndPoints(cloud_NearVerticalLine, line_vertical_vector, Intersection_point_4f,
                                   two_endpoints_VerticalSeam);
    // 输出端点1
    pcl::PointXYZ Vertical_Endpoint1(Intersection_point_4f[0] + Distance_Vertical_Offset * dir_line_1[0],
                                     Intersection_point_4f[1] + Distance_Vertical_Offset * dir_line_1[1],
                                     Intersection_point_4f[2] + Distance_Vertical_Offset * dir_line_1[2]);
    // 输出端点2
    if (two_endpoints_VerticalSeam.size() < 2) {
        PLOGE << "倒立横梁焊缝直线点云端点求解失败";
        return false;
    }
    pcl::PointXYZ Vertical_Endpoint2 = {two_endpoints_VerticalSeam[1][0], two_endpoints_VerticalSeam[1][1],
                                        two_endpoints_VerticalSeam[1][2]};
    VerticalSeam_EndPoints.push_back(Vertical_Endpoint1);
    VerticalSeam_EndPoints.push_back(Vertical_Endpoint2);

    ////-----------------------------Step4 求横梁处焊缝的端点
    Eigen::VectorXf line_beam_vector;  // 平面plane_beam和平面plane_base的交线
    pcl::planeWithPlaneIntersection(plane_beam, plane_base, line_beam_vector, 0.1);  // 求平面a和b相交直线
    if (line_beam_vector.size() < 6) {
        PLOGE << "倒立横梁焊缝平面交线求解失败";
        return false;
    }
    Eigen::Vector4f pt_on_line_2 = {line_beam_vector[0], line_beam_vector[1], line_beam_vector[2], 0};  // 直线上一点
    Eigen::Vector4f dir_line_2 = {line_beam_vector[3], line_beam_vector[4], line_beam_vector[5], 0};    // 直线方向
    if (dir_line_2[0] * vector_ave_abc[0] + dir_line_2[1] * vector_ave_abc[1] + dir_line_2[2] * vector_ave_abc[2] <
        0) {  // 保持ab线方向与三平面法线方向一致
        dir_line_2 = -dir_line_2;
        line_beam_vector.tail(3) = -line_beam_vector.tail(3);
    }
    // 提取近线点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NearHorizonLine(new pcl::PointCloud<pcl::PointXYZ>);  // 平面交线附近的点云
    for (int i = 0; i < cloud->size(); i++) {
        Eigen::Vector4f pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 0);
        double distance = sqrt(pcl::sqrPointToLineDistance(pt, pt_on_line_2, dir_line_2));

        if (distance < NearLine_DistanceThreshold) {
            cloud_NearHorizonLine->push_back(cloud->points[i]);
        }
    }
    // 高曲率点检测
    pcl::PointCloud<pcl::PointXYZ>::Ptr high_CurvaturePoints(new pcl::PointCloud<pcl::PointXYZ>);  // 高曲率点云
    MyToolFunc::highCurvaturePointsDetect(cloud_NearHorizonLine, cloud, 5, 0.25, high_CurvaturePoints);
    if (high_CurvaturePoints->size() == 0) {
        PLOGE << "倒立横梁焊缝高曲率点检测失败";
        return false;
    }

    // 按次序输出竖直交线的两个投影端点
    std::vector<Eigen::Vector4f> two_endpoints_HorizonSeam;
    MyToolFunc::lineCloudEndPoints(high_CurvaturePoints, line_beam_vector, Intersection_point_4f, two_endpoints_HorizonSeam);
    // 输出端点1
    pcl::PointXYZ Horizon_Endpoint1(Intersection_point_4f[0] + Distance_Horizon_Offset * dir_line_2[0],
                                    Intersection_point_4f[1] + Distance_Horizon_Offset * dir_line_2[1],
                                    Intersection_point_4f[2] + Distance_Horizon_Offset * dir_line_2[2]);
    // 输出端点2
    float Distance_HC_Offset = 2.5;  // 通过高曲率特征计算的端点，可设置一个偏移距离进行校正
    pcl::PointXYZ Horizon_Endpoint2 = {two_endpoints_HorizonSeam[1][0] - Distance_HC_Offset * dir_line_2[0],
                                       two_endpoints_HorizonSeam[1][1] - Distance_HC_Offset * dir_line_2[1],
                                       two_endpoints_HorizonSeam[1][2] - Distance_HC_Offset * dir_line_2[2]};
    HorizonSeam_EndPoints.push_back(Horizon_Endpoint1);
    HorizonSeam_EndPoints.push_back(Horizon_Endpoint2);

    // 焊缝输出
    beamDownVerticalFilletSeams = VerticalSeam_EndPoints;
    beamDownHorizonFilletSeams = HorizonSeam_EndPoints;
    return true;
}

void DownBeamFilletSeamsDet::SingleSeam_Reinitialize()  // 单条焊缝检测完成后，变量重新初始化
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // vector清空
    Coefficients_list.clear();
    plane_clouds_list.clear();
    HorizonSeam_EndPoints.clear();
    VerticalSeam_EndPoints.clear();

    beamDownVerticalFilletSeams.clear();
    beamDownHorizonFilletSeams.clear();
}
