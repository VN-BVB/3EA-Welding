#ifndef MYMATRIXVECTOR_H
#define MYMATRIXVECTOR_H

#include <plog/Log.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cereal/archives/json.hpp>
#include <cereal/types/complex.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

// 矩阵
class MyMatrix {
public:
    MyMatrix(int row, int col);

    int _row = 4;
    int _col = 4;
    std::vector<std::vector<double>> _data;

private:
    friend class cereal::access;
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(_data));
    }
};

// 向量
class MyVector {
public:
    MyVector(int num);

    int _num = 4;
    std::vector<double> _data;

private:
    friend class cereal::access;
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(_data));
    }
};

namespace MyToolFunc {

const double DEG2RAD = 0.0174532925199432957692369;

// 将自定义的矩阵和向量转换为opencv和eigen格式, 或写入位姿数据
void write2Mat3x4(MyMatrix& data, cv::Mat& var);
void write2Mat3x3(MyMatrix& data, cv::Mat& var);
void write2Eigen4x4d(MyMatrix& data, Eigen::Matrix4d& var);
void write2Eigen4x4f(MyMatrix& data, Eigen::Matrix4f& var);
void write2Vector5(MyVector& data, std::vector<double>& var);
void write2Pose(MyVector& data, float& a, float& b, float& c);
void write2PositionPose(MyVector& data, float& x, float& y, float& z, float& a, float& b, float& c);

// 按照各种顺序创建变换矩阵
Eigen::Matrix4f createTransformationMatrixZYX(MyVector p);
Eigen::Matrix4f createTransformationMatrixZYX(double x, double y, double z, double a, double b, double c);
Eigen::Matrix4f createTransformationMatrixZYZ(MyVector p);
std::vector<double> extractEulerZYX(const Eigen::Matrix3f& R_target, const std::vector<double>& currentEulerDeg);

}  // namespace MyToolFunc

#endif  // MYMATRIXVECTOR_H
