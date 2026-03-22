#include "UpBeamFilletSeamsDet.h"

#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

UpBeamFilletSeamsDet::UpBeamFilletSeamsDet(QObject *parent) : AbstractSeamDet{parent} {}

// 求解焊缝
std::vector<std::shared_ptr<WeldSeamInfo>> UpBeamFilletSeamsDet::solveSeamsEndPoints(
    std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    tempWeldSeamsInfo.clear();

    // 焊缝求解
    for (int i = 0; i < seamsInfo.size(); i++) {
        SingleSeam_Reinitialize();  // 单条焊缝检测前，变量重新初始化

        if (seamsInfo[i]->detectSuccFlag == true) {    // 前面对接焊缝检测成功了
            cloud = seamsInfo[i]->weldAreaPointCloudInCamera;  // 获取焊缝区域点云

            if (cloud->size() == 0) {
                tempWeldSeamsInfo[i]->detectSuccFlag = false;
                continue;
            }

            ButtSeam_endpoints = *(seamsInfo[i]->weldEndPointsInCamera);  // 获取对接焊缝端点

            MyToolFunc::myFastMaxCluster(cloud, Max_Cluster_radius);  // 快速欧式聚类提取最大点集
            Ransac_Multiple_planes(5);  // 拟合识别三个平面（修改为识别5个平面，同时平面筛选算法也发生更改）
            if (CoefficientsList.size() < 3) {  // 如果经过筛选后平面数量不足，则继续进行下一步
                SingleSeam_Reinitialize();
                continue;
            }
            bool bool_IdentifySeam = Solve_Fillet_Endpoints(i);  // 识别焊缝

            // 打印检测结果
            PLOGD << "正面正立角接焊缝端点识别结果: " << bool_IdentifySeam;
            if (beamUpHorizonFilletSeams.size() == 2) {
                PLOGD << "水平焊缝端点坐标: (" << beamUpHorizonFilletSeams[0].x << " " << beamUpHorizonFilletSeams[0].y << " "
                      << beamUpHorizonFilletSeams[0].z << ") (" << beamUpHorizonFilletSeams[1].x << " "
                      << beamUpHorizonFilletSeams[1].y << " " << beamUpHorizonFilletSeams[1].z << ")";
            }
            if (beamUpVerticalFilletSeams.size() == 2) {
                PLOGD << "竖直焊缝端点坐标: (" << beamUpVerticalFilletSeams[0].x << " " << beamUpVerticalFilletSeams[0].y << " "
                      << beamUpVerticalFilletSeams[0].z << ") (" << beamUpVerticalFilletSeams[1].x << " "
                      << beamUpVerticalFilletSeams[1].y << " " << beamUpVerticalFilletSeams[1].z << ")";
            }

            // 保存本次检测到的信息, 由于输入的焊缝信息中已经保存了对接焊缝, 故需要新建两个焊缝信息分别保存水平和竖直焊缝
            auto newHorizonSeamInfo = std::make_shared<WeldSeamInfo>();
            newHorizonSeamInfo->areaNum = seamsInfo[i]->areaNum;                        // 区域编号
            newHorizonSeamInfo->originalImg = seamsInfo[i]->originalImg;                // 原始图像
            newHorizonSeamInfo->weldAreaImg = seamsInfo[i]->weldAreaImg;                // 焊缝区域图像
            newHorizonSeamInfo->rectPtr = seamsInfo[i]->rectPtr;                        // 焊缝区域矩形框
            newHorizonSeamInfo->weldAreaPointCloudInCamera = seamsInfo[i]->weldAreaPointCloudInCamera;  // 焊缝区域点云
            newHorizonSeamInfo->weldAreaType = seamsInfo[i]->weldAreaType;              // 焊缝区域类型
            newHorizonSeamInfo->detectSuccFlag = bool_IdentifySeam;                     // 检测是否成功标志位
            if (bool_IdentifySeam == true) {
                // 角接焊缝暂时没有所在平面以及验证直线
                newHorizonSeamInfo->weldEndPointsInCamera.reset(
                    new std::vector<pcl::PointXYZ>(std::move(beamUpVerticalFilletSeams)));  // 检测结果
                newHorizonSeamInfo->weldType = WELD_TYPE::FRONT_VERTICAL_FILLET;
            }
            tempWeldSeamsInfo.push_back(newHorizonSeamInfo);  // 保存焊缝信息

            auto newVerticalSeamInfo = std::make_shared<WeldSeamInfo>();
            newVerticalSeamInfo->areaNum = seamsInfo[i]->areaNum;                        // 区域编号
            newVerticalSeamInfo->originalImg = seamsInfo[i]->originalImg;                // 原始图像
            newVerticalSeamInfo->weldAreaImg = seamsInfo[i]->weldAreaImg;                // 焊缝区域图像
            newVerticalSeamInfo->rectPtr = seamsInfo[i]->rectPtr;                        // 焊缝区域矩形框
            newVerticalSeamInfo->weldAreaPointCloudInCamera = seamsInfo[i]->weldAreaPointCloudInCamera;  // 焊缝区域点云
            newVerticalSeamInfo->weldAreaType = seamsInfo[i]->weldAreaType;              // 焊缝区域类型
            newVerticalSeamInfo->detectSuccFlag = bool_IdentifySeam;                     // 检测是否成功标志位
            if (bool_IdentifySeam == true) {
                // 角接焊缝暂时没有所在平面以及验证直线
                newVerticalSeamInfo->weldEndPointsInCamera.reset(
                    new std::vector<pcl::PointXYZ>(std::move(beamUpHorizonFilletSeams)));  // 检测结果
                newVerticalSeamInfo->weldType = WELD_TYPE::FRONT_HORIZONTAL_FILLET;
            }
            tempWeldSeamsInfo.push_back(newVerticalSeamInfo);  // 保存焊缝信息
        }
    }

    return this->tempWeldSeamsInfo;
}

// Ransac分割拟合多个平面，参数为提取平面的个数
void UpBeamFilletSeamsDet::Ransac_Multiple_planes(int plane_nums) {
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
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/Up/cloud.pcd", *cloud_ransac);
    }

    // Ransac算法提取多个平面，每次提取后进行法线方向翻转
    for (int i = 0; i < plane_nums; i++) {
        seg.setInputCloud(cloud_ransac);  // 输入点云
        seg.segment(*inliers,
                    *coefficients);  // 找最优的一个平面模型，并存储分割结果到点集合inliers及存储平面模型系数coefficients
        extract.setInputCloud(cloud_ransac);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*planar_segment);  // 提取探测出来的平面

        planeCloudsList.push_back(*planar_segment);  // 提取各平面点云

        Eigen::Vector4f centroid;                           // 计算当前平面质心
        pcl::compute3DCentroid(*planar_segment, centroid);  // 质心
        centroids_Centroid_list.push_back(centroid);        // 提取各平面质心坐标

        // 使平面法线指向相机方向,相机视点v为(0,0,0), 点乘表示两个向量夹角：n*(v-p)>0 -> 法线大致朝向相机
        double bool_overturn = coefficients->values[0] * (-centroid[0]) + coefficients->values[1] * (-centroid[1]) +
                               coefficients->values[2] * (-centroid[2]);
        if (bool_overturn < 0) {
            coefficients->values[0] = -coefficients->values[0];
            coefficients->values[1] = -coefficients->values[1];
            coefficients->values[2] = -coefficients->values[2];
            coefficients->values[3] = -coefficients->values[3];
        }

        if (i == 0) {
            CoefficientsList.push_back(*coefficients);  // 提取各平面系数
        } else {
            Eigen::Vector3f vector_base(CoefficientsList[0].values[0], CoefficientsList[0].values[1], CoefficientsList[0].values[2]);
            Eigen::Vector3f vector_current(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            float angleDeg = MyToolFunc::getLineAngle(vector_base, vector_current);
            if (angleDeg <= 15.0 || angleDeg >= 75.0) {
                pcl::PointXYZ projPoint;
                MyToolFunc::projPoint2Plane(ButtSeam_endpoints[0], *coefficients, projPoint);
                double pointDistance =
                    std::sqrt(std::pow(ButtSeam_endpoints[0].x - projPoint.x, 2) + std::pow(ButtSeam_endpoints[0].y - projPoint.y, 2) +
                              std::pow(ButtSeam_endpoints[0].z - projPoint.z, 2));
                if (pointDistance > 10) {
                    CoefficientsList.push_back(*coefficients);  // 提取各平面系数
                }
            } else {
                std::cout << "去除非常规角度" << std::endl;
            }
        }

        // 剔除探测出的平面内点，在剩余点中继续探测平面
        extract.setNegative(true);
        extract.filter(*cloud_ransac);
    }

    // 输出平面参数与内点质量
    if (saveAndOutputDebugInformation == true) {
        std::cout << "Up Beam Debug Info:" << std::endl;
        for (int i = 0; i < CoefficientsList.size(); ++i) {
            std::cout << "A" << i + 1 << " = " << CoefficientsList[i].values[0] << "; B" << i + 1 << " = "
                      << CoefficientsList[i].values[1] << "; C" << i + 1 << " = " << CoefficientsList[i].values[2] << "; D"
                      << i + 1 << " = " << CoefficientsList[i].values[3] << ";" << "  内点数量：" << planeCloudsList[i].size()
                      << std::endl;
        }
    }

    // ------------------------------找到几个相互平行的平面的索引，并去掉除了内点最多的那个平面以外的平面
    float min_angle = std::numeric_limits<float>::max();  // 表示 float 类型的最大正值
    int index_of_parallel_plane = -1;
    std::vector<int> indexOfParallelPlanes;  // 几个相互平行的平面的索引
    indexOfParallelPlanes.push_back(0);
    Eigen::Vector3f vector_base(CoefficientsList[0].values[0], CoefficientsList[0].values[1], CoefficientsList[0].values[2]);
    for (int i = 1; i < CoefficientsList.size(); ++i) {
        Eigen::Vector3f vector_current(CoefficientsList[i].values[0], CoefficientsList[i].values[1],
                                       CoefficientsList[i].values[2]);
        float angle = MyToolFunc::getLineAngle(vector_base, vector_current);
        // 输出平面夹角信息
        if (saveAndOutputDebugInformation == true) {
            std::cout << "第 " << i + 1 << " 个平面与第 1 个平面的夹角：" << angle << std::endl;
        }
        // 保存认为是平行平面的平面索引
        if (angle < angleParalleThreshold) {
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
        if (planeCloudsList[indexOfParallelPlanes[i]].size() > mostInliersNum) {
            mostInliersNum = planeCloudsList[indexOfParallelPlanes[i]].size();
            indexOfPlaneWithMostInliersNum = indexOfParallelPlanes[i];
        }
    }
    if (saveAndOutputDebugInformation == true) {
        std::cout << "内点最多的平面索引indexOfPlaneWithMostInliersNum：" << indexOfPlaneWithMostInliersNum << std::endl;
    }

    // 去掉除内点最多的那个平面以外的平面
    // 将其中的元素从大到小排列，因为如果先删除了前面的，再删除后面的，后面的索引就会变化，进而导致误删了想要的
    std::sort(indexOfParallelPlanes.begin(), indexOfParallelPlanes.end(), std::greater<int>());
    for (int i = 0; i < indexOfParallelPlanes.size(); ++i) {
        if (indexOfParallelPlanes[i] != indexOfPlaneWithMostInliersNum) {
            std::cout << "删除平面 " << indexOfParallelPlanes[i] << std::endl;
            CoefficientsList.erase(CoefficientsList.begin() + indexOfParallelPlanes[i]);
            // planeCloudsList.erase(planeCloudsList.begin() + indexOfParallelPlanes[i]);
        }
    }

    // 去除后剩余的平面
    if (saveAndOutputDebugInformation == true) {
        std::cout << "去除后剩余的平面：" << std::endl;
        for (int i = 0; i < CoefficientsList.size(); ++i) {
            std::cout << "A" << i + 1 << " = " << CoefficientsList[i].values[0] << "; B" << i + 1 << " = "
                      << CoefficientsList[i].values[1] << "; C" << i + 1 << " = " << CoefficientsList[i].values[2] << "; D"
                      << i + 1 << " = " << CoefficientsList[i].values[3] << ";" << std::endl;
            // std::cout << "内点数量：" << planeCloudsList[i].size() << std::endl;
        }
    }

    // // 输出哪一个平面被去掉了
    // if (saveAndOutputDebugInformation == true) {
    //     std::cout << "Up Beam Debug Info:" << std::endl;
    //     std::cout << "去除的平面的参数序号 index_of_parallel_plane + 1: " << index_of_parallel_plane + 1 << std::endl;
    // }
    // // 去除和第一个平面最接近平行的平面
    // CoefficientsList.erase(CoefficientsList.begin() + index_of_parallel_plane);
}

// 提取焊缝
bool UpBeamFilletSeamsDet::Solve_Fillet_Endpoints(int m) {
    if (ButtSeam_endpoints.size() == 2) {
        // ------------------------------Step1 计算ButtSeam_endpoints点连线与平面法向量之间的夹角。
        Eigen::Vector3f lineVec =
            (ButtSeam_endpoints[1].getVector3fMap() - ButtSeam_endpoints[0].getVector3fMap()).normalized();  // 对接焊缝向量

        // 初始化最小和最大角度以及对应的索引
        double minAngle = 180.0;  // 最大角度
        double maxAngle = 0.0;    // 最小角度
        int minIndex = -1;
        int maxIndex = -1;

        // 遍历平面的法向量
        for (size_t i = 1; i < CoefficientsList.size(); ++i) {
            // 获取平面的法向量
            Eigen::Vector3f normal(CoefficientsList[i].values[0], CoefficientsList[i].values[1], CoefficientsList[i].values[2]);
            double angle = MyToolFunc::getLineAngle(lineVec, normal);
            std::cout << "############# " << i << " " << angle << std::endl;
            // 更新最小和最大角度以及对应的索引
            if (angle < minAngle) {
                minAngle = angle;
                minIndex = i;
            }
            if (angle > maxAngle) {
                maxAngle = angle;
                maxIndex = i;
            }
        }

        // ------------------------------Step2 求三面交点------------------------------
        // 三个平面的法线
        Eigen::Vector4f plane_base = {CoefficientsList[0].values[0], CoefficientsList[0].values[1], CoefficientsList[0].values[2],
                                      CoefficientsList[0].values[3]};
        Eigen::Vector4f plane_beam = {CoefficientsList[minIndex].values[0], CoefficientsList[minIndex].values[1],
                                      CoefficientsList[minIndex].values[2], CoefficientsList[minIndex].values[3]};
        Eigen::Vector4f plane_outer = {CoefficientsList[maxIndex].values[0], CoefficientsList[maxIndex].values[1],
                                       CoefficientsList[maxIndex].values[2], CoefficientsList[maxIndex].values[3]};
        // 三个平面法线的平均中心线
        // Eigen::Vector3f vector_ave_abc = (plane_a.head(3) + plane_b.head(3) + plane_c.head(3)) / 3.0;
        Eigen::Vector4f vector_ave_abc = (plane_base + plane_beam + plane_outer) / 3.0;
        // 求三个平面的交点
        Eigen::Vector3f Intersection_point;
        pcl::threePlanesIntersection(plane_base, plane_beam, plane_outer, Intersection_point, 1e-6);
        ThreePlanesIntersectionPoint = {Intersection_point[0], Intersection_point[1], Intersection_point[2]};

        //-----------------------------Step3 求横梁处焊缝的关键点
        pcl::PointXYZ BeamSeam_OnePoint;  // 横梁焊缝的一端点
        MyToolFunc::projPoint2Plane(ButtSeam_endpoints[0], CoefficientsList[minIndex], BeamSeam_OnePoint);

        //-----------------------------Step4 求立焊缝的端点
        Eigen::VectorXf line_vertical_vector;  // 平面plane_beam和平面plane_outer的交线
        pcl::planeWithPlaneIntersection(plane_beam, plane_outer, line_vertical_vector, 0.1);  // 求平面b和平面c的相交直线
        if (line_vertical_vector.size() < 6) {                                                //[x0, y0, z0, dx, dy, dz]
            PLOGE << "正立横梁焊缝平面交线求解失败";
            return false;
        }
        Eigen::Vector4f pt_on_line_1 = {line_vertical_vector[0], line_vertical_vector[1], line_vertical_vector[2],
                                        0};  // 直线上一点
        Eigen::Vector4f dir_line_1 = {line_vertical_vector[3], line_vertical_vector[4], line_vertical_vector[5], 0};  // 直线方向
        if (dir_line_1[0] * vector_ave_abc[0] + dir_line_1[1] * vector_ave_abc[1] + dir_line_1[2] * vector_ave_abc[2] <
            0) {  // 保持bc线方向与三平面法线方向一致
            dir_line_1 = -dir_line_1;
            line_vertical_vector.tail(3) = -line_vertical_vector.tail(3);  // 后三个取负
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
        pcl::PointXYZ Vertical_Endpoint2 = {two_endpoints_VerticalSeam[1][0], two_endpoints_VerticalSeam[1][1],
                                            two_endpoints_VerticalSeam[1][2]};
        VerticalSeam_EndPoints.push_back(Vertical_Endpoint1);
        VerticalSeam_EndPoints.push_back(Vertical_Endpoint2);

        //-----------------------------Step5 求横梁处焊缝的端点
        Eigen::VectorXf line_beam_vector;  // 平面plane_beam和平面plane_outer的交线
        pcl::planeWithPlaneIntersection(plane_beam, plane_base, line_beam_vector, 0.1);  // 求平面a和b相交直线
        if (line_beam_vector.size() < 6) {
            PLOGE << "正立横梁焊缝平面交线求解失败";
            return false;
        }
        Eigen::Vector4f pt_on_line_2 = {line_beam_vector[0], line_beam_vector[1], line_beam_vector[2], 0};  // 直线上一点
        Eigen::Vector4f dir_line_2 = {line_beam_vector[3], line_beam_vector[4], line_beam_vector[5], 0};    // 直线方向
        if (dir_line_2[0] * vector_ave_abc[0] + dir_line_2[1] * vector_ave_abc[1] + dir_line_2[2] * vector_ave_abc[2] <
            0) {  // 保持ab线方向与三平面法线方向一致
            dir_line_2 = -dir_line_2;
            line_beam_vector.tail(3) = -line_beam_vector.tail(3);
        }
        // 输出端点1
        Eigen::Vector4f Horizon_Endpoint1 = {Intersection_point_4f[0] + Distance_Horizon_Offset * dir_line_2[0],
                                             Intersection_point_4f[1] + Distance_Horizon_Offset * dir_line_2[1],
                                             Intersection_point_4f[2] + Distance_Horizon_Offset * dir_line_2[2], 0};
        // 输出端点2
        Eigen::Vector4f Buttseam_point(ButtSeam_endpoints[0].x, ButtSeam_endpoints[0].y, ButtSeam_endpoints[0].z, 0);
        Eigen::Vector4f Horizon_Endpoint2 = MyToolFunc::projPoint2Line(Buttseam_point, pt_on_line_2, dir_line_2);
        // 输出焊缝端点
        pcl::PointXYZ Point_BeamSeamStart = {Horizon_Endpoint1[0], Horizon_Endpoint1[1], Horizon_Endpoint1[2]};
        pcl::PointXYZ Point_BeamSeamEnd = {Horizon_Endpoint2[0], Horizon_Endpoint2[1], Horizon_Endpoint2[2]};
        HorizonSeam_EndPoints.push_back(Point_BeamSeamStart);
        HorizonSeam_EndPoints.push_back(Point_BeamSeamEnd);

        // 焊缝输出
        beamUpVerticalFilletSeams = VerticalSeam_EndPoints;
        beamUpHorizonFilletSeams = HorizonSeam_EndPoints;

        return true;  // 返回焊缝识别成功
    } else {
        PLOGE << "正立焊缝识别失败, 对接焊缝端点数量不足";
        return false;  // 返回焊缝识别失败
    }
}

// 单条焊缝检测完成后，变量重新初始化
void UpBeamFilletSeamsDet::SingleSeam_Reinitialize() {
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // vector清空
    CoefficientsList.clear();
    planeCloudsList.clear();
    HorizonSeam_EndPoints.clear();
    VerticalSeam_EndPoints.clear();
    ButtSeam_endpoints.clear();

    beamUpVerticalFilletSeams.clear();
    beamUpHorizonFilletSeams.clear();
}
