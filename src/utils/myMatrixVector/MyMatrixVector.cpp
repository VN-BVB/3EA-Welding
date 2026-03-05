#include "MyMatrixVector.h"

MyMatrix::MyMatrix(int row, int col) : _row(row), _col(col) {
    _data.resize(row);
    for (int i = 0; i < row; i++) {
        _data[i].resize(col);
    }
    // 赋初值
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            _data[i][j] = 0;
        }
    }
}

MyVector::MyVector(int num) : _num(num) {
    _data.resize(num);
    // 赋初值
    for (int i = 0; i < num; i++) {
        _data[i] = 0;
    }
}

// clang-format off
void MyToolFunc::write2Mat3x4(MyMatrix &data, cv::Mat &var) {
    if (data._row != 3 || data._col != 4) {
        PLOGE << "write2Mat3x4, data error";
        return;
    }
    var = (cv::Mat_<double>(3, 4) << data._data[0][0], data._data[0][1], data._data[0][2], data._data[0][3],
                                     data._data[1][0], data._data[1][1], data._data[1][2], data._data[1][3],
                                     data._data[2][0], data._data[2][1], data._data[2][2], data._data[2][3]);
}

void MyToolFunc::write2Mat3x3(MyMatrix &data, cv::Mat &var) {
    if (data._row != 3 || data._col != 3) {
        PLOGE << "write2Mat3x3, data error";
        return;
    }
    var = (cv::Mat_<double>(3, 3) << data._data[0][0], data._data[0][1], data._data[0][2],
                                     data._data[1][0], data._data[1][1], data._data[1][2],
                                     data._data[2][0], data._data[2][1], data._data[2][2]);
}

void MyToolFunc::write2Eigen4x4d(MyMatrix &data, Eigen::Matrix4d &var) {
    if (data._row != 4 || data._col != 4) {
        PLOGE << "write2Eigen4x4d, data error";
        return;
    }
    var << data._data[0][0], data._data[0][1], data._data[0][2], data._data[0][3],
           data._data[1][0], data._data[1][1], data._data[1][2], data._data[1][3],
           data._data[2][0], data._data[2][1], data._data[2][2], data._data[2][3],
           data._data[3][0], data._data[3][1], data._data[3][2], data._data[3][3];
}

void MyToolFunc::write2Eigen4x4f(MyMatrix &data, Eigen::Matrix4f &var) {
    if (data._row != 4 || data._col != 4) {
        PLOGE << "write2Eigen4x4f, data error";
        return;
    }
    var << (float)data._data[0][0], (float)data._data[0][1], (float)data._data[0][2], (float)data._data[0][3],
           (float)data._data[1][0], (float)data._data[1][1], (float)data._data[1][2], (float)data._data[1][3],
           (float)data._data[2][0], (float)data._data[2][1], (float)data._data[2][2], (float)data._data[2][3],
           (float)data._data[3][0], (float)data._data[3][1], (float)data._data[3][2], (float)data._data[3][3];
}

void MyToolFunc::write2Vector5(MyVector &data, std::vector<double> &var) {
    if (data._num != 5) {
        PLOGE << "write2Vector5, data error";
        return;
    }
    var = { data._data[0], data._data[1], data._data[2], data._data[3], data._data[4] };
}

void MyToolFunc::write2Pose(MyVector &data, float &a, float &b, float &c) {
    if (data._num != 3) {
        PLOGE << "write2Vector5, data error";
        return;
    }
    a = static_cast<float>(data._data[0]);
    b = static_cast<float>(data._data[1]);
    c = static_cast<float>(data._data[2]);
}

void MyToolFunc::write2PositionPose(MyVector &data, float &x, float &y, float &z, float &a, float &b, float &c) {
    if (data._num != 6) {
        PLOGE << "write2Vector5, data error";
        return;
    }
    x = static_cast<float>(data._data[0]);
    y = static_cast<float>(data._data[1]);
    z = static_cast<float>(data._data[2]);
    a = static_cast<float>(data._data[3]);
    b = static_cast<float>(data._data[4]);
    c = static_cast<float>(data._data[5]);
}

// clang-format on

// 按照ZYX顺序创建变换矩阵
Eigen::Matrix4f MyToolFunc::createTransformationMatrixZYX(MyVector p) {
    // 创建旋转矩阵
    Eigen::Matrix3f rotX;
    rotX = Eigen::AngleAxisf(p._data[3] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::Matrix3f rotY;
    rotY = Eigen::AngleAxisf(p._data[4] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rotZ;
    rotZ = Eigen::AngleAxisf(p._data[5] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitZ());

    // 旋转矩阵，按照ZYX顺序旋转（先绕Z轴旋转，然后是Y轴，最后是X轴的旋转）
    Eigen::Matrix3f rotation = rotZ * rotY * rotX;

    // 创建 4x4 变换矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation(0, 3) = p._data[0];
    transformation(1, 3) = p._data[1];
    transformation(2, 3) = p._data[2];

    return transformation;
}

// 按照ZYX顺序创建变换矩阵
Eigen::Matrix4f MyToolFunc::createTransformationMatrixZYX(double x, double y, double z, double a, double b, double c) {
    // 创建旋转矩阵
    Eigen::Matrix3f rotX;
    rotX = Eigen::AngleAxisf(a * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::Matrix3f rotY;
    rotY = Eigen::AngleAxisf(b * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rotZ;
    rotZ = Eigen::AngleAxisf(c * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitZ());

    // 旋转矩阵，按照ZYX顺序旋转（先绕Z轴旋转，然后是Y轴，最后是X轴的旋转）
    Eigen::Matrix3f rotation = rotZ * rotY * rotX;

    // 创建 4x4 变换矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation(0, 3) = x;
    transformation(1, 3) = y;
    transformation(2, 3) = z;

    return transformation;
}

// 按照ZYZ顺序创建变换矩阵
Eigen::Matrix4f MyToolFunc::createTransformationMatrixZYZ(MyVector p) {
    // 创建旋转矩阵
    Eigen::Matrix3f rotZ1;
    rotZ1 = Eigen::AngleAxisf(p._data[3] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotY;
    rotY = Eigen::AngleAxisf(p._data[4] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rotZ2;
    rotZ2 = Eigen::AngleAxisf(p._data[5] * MyToolFunc::DEG2RAD, Eigen::Vector3f::UnitZ());

    // 旋转矩阵，按照ZYZ顺序旋转 (先绕Z轴旋转，然后是Y轴，最后还是Z轴的旋转)
    Eigen::Matrix3f rotation = rotZ1 * rotY * rotZ2;

    // 创建 4x4 变换矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation(0, 3) = p._data[0];
    transformation(1, 3) = p._data[1];
    transformation(2, 3) = p._data[2];

    return transformation;
}
