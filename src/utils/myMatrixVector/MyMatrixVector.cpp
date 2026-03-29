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
std::vector<double> MyToolFunc::extractEulerZYX(const Eigen::Matrix3f &R, const std::vector<double> &currentEulerDeg) {
    auto norm = [](double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    };

    auto angleDiff = [](double a, double b) {
        double d = a - b;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    };

    // =========================
    // 1. 手动解（基础解）
    // =========================
    double rz = std::atan2(R(1, 0), R(0, 0));
    double ry = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    double rx = std::atan2(R(2, 1), R(2, 2));

    rx = norm(rx * 180.0 / M_PI);
    ry = norm(ry * 180.0 / M_PI);
    rz = norm(rz * 180.0 / M_PI);

    // =========================
    // 2. 两组解
    // =========================
    std::vector<std::vector<double>> sols;

    sols.push_back({rx, ry, rz});

    sols.push_back({norm(rx + 180.0), norm(180.0 - ry), norm(rz + 180.0)});

    // =========================
    // 打印所有解
    // =========================
    PLOGD << "旋转矩阵求解欧拉角所有解";
    for (size_t i = 0; i < sols.size(); i++) {
        std::cout << "  sol[" << i << "]: " << sols[i][0] << ", " << sols[i][1] << ", " << sols[i][2] << std::endl;
    }

    // =========================
    //  当前姿态
    // =========================
    PLOGD << " 当前姿态打印，筛选欧拉角解";
    PLOGD << "Current Euler (deg): [" << currentEulerDeg[0] << ", " << currentEulerDeg[1] << ", " << currentEulerDeg[2] << "]";

    // =========================
    // 选最优解
    // =========================
    auto cost = [&](const std::vector<double> &euler) {
        double c = 0;
        for (int i = 0; i < 3; i++) {
            double d = angleDiff(euler[i], currentEulerDeg[i]);
            c += d * d;
        }
        return c;
    };

    int bestIdx = 0;
    double bestCost = cost(sols[0]);

    for (size_t i = 0; i < sols.size(); i++) {
        double c = cost(sols[i]);
        std::cout << "  cost[" << i << "] = " << c << std::endl;

        if (c < bestCost) {
            bestCost = c;
            bestIdx = i;
        }
    }

    std::cout << "Selected index: " << bestIdx << std::endl;
    std::cout << "Selected Euler: " << sols[bestIdx][0] << ", " << sols[bestIdx][1] << ", " << sols[bestIdx][2] << std::endl;

    return sols[bestIdx];
}
// 欧拉角（度）→ 四元数
Eigen::Quaterniond MyToolFunc::eulerToQuat(double a, double b, double c) {
    double A = a * M_PI / 180.0;
    double B = b * M_PI / 180.0;
    double C = c * M_PI / 180.0;

    Eigen::AngleAxisd rx(A, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(B, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(C, Eigen::Vector3d::UnitZ());

    return rz * ry * rx;  // ZYX 顺序（和你 extractEulerZYX 一致）
}

// 四元数 → 欧拉角（度）
Eigen::Vector3d MyToolFunc::quatToEuler(const Eigen::Quaterniond &q) {
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);  // ZYX

    return Eigen::Vector3d(euler[2] * 180.0 / M_PI,  // a (X)
                           euler[1] * 180.0 / M_PI,  // b (Y)
                           euler[0] * 180.0 / M_PI   // c (Z)
    );
}

// 求中间姿态
std::vector<double> MyToolFunc::interpolateEulerZYX(double a1, double b1, double c1, double a2, double b2, double c2, double alpha) {
    // 1. 转四元数
    Eigen::Quaterniond q1 = eulerToQuat(a1, b1, c1);
    Eigen::Quaterniond q2 = eulerToQuat(a2, b2, c2);

    // 2. 最短路径修正（关键！）
    if (q1.dot(q2) < 0.0) {
        q2.coeffs() *= -1.0;
    }
    // 3. slerp 插值
    Eigen::Quaterniond q_mid = q1.slerp(alpha, q2);

    // 4. 转回欧拉角
    Eigen::Vector3d euler_mid = quatToEuler(q_mid);

    return {euler_mid[0], euler_mid[1], euler_mid[2]};
}
