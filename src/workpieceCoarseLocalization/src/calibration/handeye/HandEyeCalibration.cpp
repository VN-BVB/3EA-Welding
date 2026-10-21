#include "HandEyeCalibration.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>

HandEyeCalibration::HandEyeCalibration(const std::string& imageDirectory, const std::string& outputFilename, cv::Size boardSize, double squareSize,
                                       Pattern type)
    : Calibration(imageDirectory, outputFilename, boardSize, squareSize, type) {}

HandEyeCalibration::~HandEyeCalibration() {}

void HandEyeCalibration::computeHandEyeMatrix(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij, HandEyeCaliMethod method,
                                              const std::string& file) {
    switch (method) {
        case HAND_EYE_NAVY:
            solveNavyHandEye(Hcg, Hgij, Hcij);
            break;
        case HAND_EYE_TSAI:
            solveTsaiHandEye(Hcg, Hgij, Hcij);
            break;
        default:
            solveNavyHandEye(Hcg, Hgij, Hcij);
            break;
    }

    cv::FileStorage fs(file, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cout << "无法将手眼矩阵写入 \"" << file << "\" failed." << std::endl;
        return;
    }
    fs << "handEyeMatrix" << Hcg;
    fs.release();
}

void HandEyeCalibration::solveTsaiHandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij) {
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = static_cast<int>(Hgij.size());

    auto skew = [](cv::Mat mp) {
        // 构造三维向量对应的反对称矩阵。
        cv::Mat tmp = cv::Mat::zeros(cv::Size{3, 3}, CV_64FC1);
        tmp.at<double>(0, 1) = -mp.at<double>(2, 0);
        tmp.at<double>(1, 0) = mp.at<double>(2, 0);
        tmp.at<double>(0, 2) = mp.at<double>(1, 0);
        tmp.at<double>(2, 0) = -mp.at<double>(1, 0);
        tmp.at<double>(1, 2) = -mp.at<double>(0, 0);
        tmp.at<double>(2, 1) = mp.at<double>(0, 0);
        return tmp;
    };

    cv::Mat Pgij, Pcij;
    cv::Mat Rgij, Rcij;
    cv::Mat rgij, rcij;
    cv::Mat rngij, rncij;
    double thetaGij, thetaCij;
    cv::Mat A, b;

    for (int i = 0; i < nStatus; i++) {
        Hgij[i]({0, 0, 3, 3}).copyTo(Rgij);
        Hcij[i]({0, 0, 3, 3}).copyTo(Rcij);

        cv::Rodrigues(Rgij, rgij);
        cv::Rodrigues(Rcij, rcij);

        thetaGij = cv::norm(rgij);
        thetaCij = cv::norm(rcij);

        rngij = rgij / thetaGij;
        rncij = rcij / thetaCij;

        Pgij = 2 * std::sin(thetaGij / 2) * rngij;
        Pcij = 2 * std::sin(thetaCij / 2) * rncij;

        A.push_back(skew(Pgij + Pcij));
        b.push_back(Pcij - Pgij);
    }

    // 计算旋转部分
    cv::Mat eyeM = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat pinA = A.inv(cv::DECOMP_SVD);
    cv::Mat pcgPrime = pinA * b;
    cv::Mat Pcg = 2 * pcgPrime / std::sqrt(1 + cv::norm(pcgPrime) * cv::norm(pcgPrime));
    cv::Mat PcgTrs = Pcg.t();
    cv::Mat Rcg = (1 - cv::norm(Pcg) * cv::norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + std::sqrt(4 - cv::norm(Pcg) * cv::norm(Pcg)) * skew(Pcg));

    // 计算平移部分
    cv::Mat AA, bb;
    cv::Mat Tgij, Tcij;
    for (int i = 0; i < nStatus; i++) {
        Hgij[i]({0, 0, 3, 3}).copyTo(Rgij);
        Hcij[i]({0, 0, 3, 3}).copyTo(Rcij);
        Hgij[i]({3, 0, 1, 3}).copyTo(Tgij);
        Hcij[i]({3, 0, 1, 3}).copyTo(Tcij);

        AA.push_back(Rgij - eyeM);
        bb.push_back(Rcg * Tcij - Tgij);
    }
    cv::Mat Tcg = AA.inv(cv::DECOMP_SVD) * bb;

    Hcg = cv::Mat::eye(4, 4, CV_64F);
    Rcg.copyTo(Hcg({0, 0, 3, 3}));
    Tcg.copyTo(Hcg({3, 0, 1, 3}));
}

void HandEyeCalibration::solveNavyHandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij) {
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = static_cast<int>(Hgij.size());

    cv::Mat Rx, Tx;
    cv::Mat Rgij, Rcij, Tgij, Tcij;

    // 计算旋转部分
    if (Hgij.size() == 2)  // 两组 (Ai, Bi) 数据对
    {
        cv::Mat alpha1, beta1;
        cv::Mat alpha2, beta2;
        cv::Mat A(3, 3, CV_64FC1), B(3, 3, CV_64FC1);

        cv::Rodrigues(Hgij[0]({0, 0, 3, 3}), alpha1);
        cv::Rodrigues(Hgij[1]({0, 0, 3, 3}), alpha2);

        cv::Rodrigues(Hcij[0]({0, 0, 3, 3}), beta1);
        cv::Rodrigues(Hcij[1]({0, 0, 3, 3}), beta2);

        alpha1.copyTo(A.col(0));
        alpha2.copyTo(A.col(1));
        (alpha1.cross(alpha2)).copyTo(A.col(2));

        beta1.copyTo(B.col(0));
        beta2.copyTo(B.col(1));
        (beta1.cross(beta2)).copyTo(B.col(2));

        Rx = A * B.inv();

    } else  // 多于两组 (Ai, Bi) 数据对
    {
        cv::Mat alpha, beta;
        cv::Mat M(3, 3, CV_64FC1, cv::Scalar(0));

        for (int i = 0; i < nStatus; i++) {
            Hgij[i]({0, 0, 3, 3}).copyTo(Rgij);
            Hcij[i]({0, 0, 3, 3}).copyTo(Rcij);

            cv::Rodrigues(Rgij, alpha);
            cv::Rodrigues(Rcij, beta);

            M = M + beta * alpha.t();
        }

        cv::Mat MtM, veMtM, vaMtM;
        MtM = M.t() * M;
        cv::eigen(MtM, vaMtM, veMtM);

        cv::Mat pvaM(3, 3, CV_64FC1, cv::Scalar(0));
        pvaM.at<double>(0, 0) = 1 / std::sqrt(vaMtM.at<double>(0, 0));
        pvaM.at<double>(1, 1) = 1 / std::sqrt(vaMtM.at<double>(1, 0));
        pvaM.at<double>(2, 2) = 1 / std::sqrt(vaMtM.at<double>(2, 0));

        Rx = veMtM * pvaM * veMtM.inv() * M.t();
    }

    // 计算平移部分
    cv::Mat C, d;
    cv::Mat eyeM = cv::Mat::eye(3, 3, CV_64FC1);
    for (int i = 0; i < nStatus; i++) {
        Hgij[i]({0, 0, 3, 3}).copyTo(Rgij);
        Hgij[i]({3, 0, 1, 3}).copyTo(Tgij);
        Hcij[i]({0, 0, 3, 3}).copyTo(Rcij);  // 当前未使用
        Hcij[i]({3, 0, 1, 3}).copyTo(Tcij);

        C.push_back(eyeM - Rgij);
        d.push_back(Tgij - Rx * Tcij);
    }
    Tx = (C.t() * C).inv() * (C.t() * d);

    Hcg = cv::Mat::eye(4, 4, CV_64F);
    Rx.copyTo(Hcg({0, 0, 3, 3}));
    Tx.copyTo(Hcg({3, 0, 1, 3}));
}

void HandEyeCalibration::convertRelativePoses(std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc, std::vector<cv::Mat>& vecHgij,
                                              std::vector<cv::Mat>& vecHcij) {
    CV_Assert(vecHg.size() == vecHc.size());
    vecHgij.reserve(vecHg.size());
    vecHcij.reserve(vecHc.size());
    for (int i = 0; i < vecHc.size() - 1; ++i) {
        // 相机：A = A2 * A1.inv()；机器人：B = B2.inv() * B1
        vecHgij.emplace_back(vecHg[i + 1].inv() * vecHg[i]);
        vecHcij.emplace_back(vecHc[i + 1] * vecHc[i].inv());
    }
    // 调试：输出两组相机相对位姿。
    // 调试：输出两组机器人相对位姿。
}

// 输入：（x、y、z、rx、ry、rz）
// 输出：4×4 位姿矩阵
bool HandEyeCalibration::readDataFromFile(const std::string& file, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc) {
    std::ifstream infile(file, std::ios::in);
    if (!infile.is_open()) {
        std::cout << "文件路径错误。" << std::endl;
        return false;
    }

    std::vector<cv::Mat> poseRows;
    while (!infile.eof()) {
        cv::Mat temp(1, 6, CV_64FC1);
        for (int i = 0; i < 6; ++i) infile >> temp.at<double>(0, i);
        // 调试：输出临时数据。
        poseRows.emplace_back(std::move(temp));
    }
    infile.close();

    std::size_t sz = poseRows.size();
    if (sz % 2 == 1) {
        std::cout << "数据项数量不匹配。" << std::endl;
        return false;
    }

    vecHg.assign(poseRows.cbegin(), poseRows.cbegin() + sz / 2);
    vecHc.assign(poseRows.cbegin() + sz / 2, poseRows.cend());

    // 保留旧接口的位姿向量转换方式。
    for (auto& m : vecHg) m = attitudeVectorToMatrix(m, "zyx");  // 保留旧版机器人位姿转换调用。

    for (auto& m : vecHc) m = attitudeVectorToMatrix(m);  // 保留旧版相机位姿转换调用。

    return true;
}

/**
 * @brief 读取相机外参和机器人位姿，并按标定板检测标记选择对应数据。
 * @param file 相机参数文件，外参每行依次为旋转向量和平移向量。
 * @param folder 机器人 robotpos##.xml 文件所在目录。
 * @param vecHg 输出机器人位姿的 4×4 变换矩阵。
 * @param vecHc 输出标定板到相机坐标系的 4×4 变换矩阵。
 * @param useQuaternion 是否使用四元数分支；非四元数分支按 xyz 欧拉角（度）转换。
 * @note 相机外参旋转向量为弧度制，所有平移保持输入数据的长度单位。
 */
bool HandEyeCalibration::readDataFromFile(const std::string& file, const std::string& folder, std::vector<cv::Mat>& vecHg,
                                          std::vector<cv::Mat>& vecHc, bool useQuaternion) {
    // 读取相机标定文件中的外参和检测标记。
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "open \"" << file << "\" failed." << std::endl;
        return false;
    }
    cv::Mat extBigMat = fs["extrinsic_parameters"].mat();
    cv::Mat foundChessboardMat = fs["found_cheese_board"].mat();  // 整型行向量，文件字段保持原名。
    fs.release();

    // 将 {rx,ry,rz,x,y,z} 调整为 {x,y,z,rx,ry,rz}；旋转向量为弧度制。
    cv::Mat tExtM = extBigMat(cv::Range::all(), cv::Range(3, 6)).clone();
    cv::Mat rExtM = extBigMat(cv::Range::all(), cv::Range(0, 3)).clone();
    cv::hconcat(tExtM, rExtM, extBigMat);

    if (extBigMat.rows != (int)foundChessboardMat.total()) {
        std::cout << "检测结果数量与外参数量不匹配。" << std::endl;
        return false;
    }

    // 位姿大矩阵
    cv::Mat attitudeBigMat;
    std::vector<cv::String> xmlFiles;
    cv::glob(folder + "/*.xml", xmlFiles);  // 文件名约定为 robotpos##.xml。
    // cv::glob 按字典序返回文件，因此按固定前缀后的数字序号重新排序。
    auto getSubstr = [](const cv::String& str) {        // "robotpos##.xml" 中的序号 "##"
        std::size_t strPos = str.find("robotpos") + 8;  // 固定前缀长度为 8。
        std::size_t dotPos = str.find_last_of(".");
        return str.substr(strPos, dotPos - strPos);
    };
    std::sort(xmlFiles.begin(), xmlFiles.end(),
              [&getSubstr](cv::String& lhs, cv::String& rhs) { return std::stoi(getSubstr(lhs)) < std::stoi(getSubstr(rhs)); });

    // 检查数据数量
    if (xmlFiles.size() == 0) {
        std::cout << folder << "中没有 xml 文件。" << std::endl;
        return false;
    } else if (xmlFiles.size() < (int)foundChessboardMat.total()) {
        std::cout << folder << "中的 xml 文件数量与成功检测数量不匹配。" << std::endl;
        return false;
    }

    // 根据成功检测标记读取对应外参
    for (int i = 0; i < foundChessboardMat.total(); ++i) {
        if (foundChessboardMat.at<int>(0, i) == 1) {
            cv::FileStorage fs(xmlFiles[i], cv::FileStorage::READ);
            if (!fs.isOpened()) {
                std::cout << "无法打开 \"" << file << "\"." << std::endl;
                return false;
            }
            std::cout << "已打开 " << xmlFiles[i] << std::endl;
            std::vector<double> poseValues;  // Position0～Position5 共 6 个数据项。
            for (int i = 0; i < 6; ++i) {
                double val = fs[cv::format("Position%d", i)].real();
                poseValues.push_back(val);
            }
            fs.release();
            attitudeBigMat.push_back(cv::Mat(poseValues).t());
        }
    }

    // 向量转换为矩阵
    for (int i = 0; i < extBigMat.rows; ++i) {
        cv::Mat tmp = attitudeVectorToMatrix(extBigMat.row(i), false);  // OpenCV Rodrigues：旋转向量转旋转矩阵
        vecHc.push_back(std::move(tmp));                                // 外参矩阵
    }
    for (int i = 0; i < attitudeBigMat.rows; ++i) {
        // 共三组标定数据：第一组旋转部分为旋转向量，第二、三组为欧拉角。
        // cv::Mat tmp = attitudeVectorToMatrix(attitudeBigMat.row(i), useQuaternion, "");  // 旋转向量  对应1
        cv::Mat tmp = attitudeVectorToMatrix(attitudeBigMat.row(i), useQuaternion, "xyz");  // 欧拉角 对应2和3    常用

        vecHg.push_back(std::move(tmp));  // 位姿矩阵
    }

    return true;
}

/**
 * @brief 按文件序号读取机器人位姿，保留眼在手外计算使用的取逆约定。
 * @param folder 机器人 robotpos##.xml 文件所在目录。
 * @param vecHg 输出读取后取逆的 4×4 位姿矩阵。
 * @param useQuaternion 是否使用四元数分支；非四元数分支按 xyz 欧拉角（度）转换。
 * @note Position0～Position2 的位置数值直接使用，不进行长度单位换算。
 */
bool HandEyeCalibration::readRobotDatasFromFile(const std::string& folder, std::vector<cv::Mat>& vecHg, bool useQuaternion) {
    // 位姿大矩阵
    cv::Mat attitudeBigMat;
    std::vector<cv::String> xmlFiles;
    cv::glob(folder + "/*.xml", xmlFiles);  // 文件名约定为 robotpos##.xml。
    // cv::glob 按字典序返回文件，因此按固定前缀后的数字序号重新排序。
    auto getSubstr = [](const cv::String& str) {        // "robotpos##.xml" 中的序号 "##"
        std::size_t strPos = str.find("robotpos") + 8;  // 固定前缀长度为 8。
        std::size_t dotPos = str.find_last_of(".");
        return str.substr(strPos, dotPos - strPos);
    };
    std::sort(xmlFiles.begin(), xmlFiles.end(),
              [&getSubstr](cv::String& lhs, cv::String& rhs) { return std::stoi(getSubstr(lhs)) < std::stoi(getSubstr(rhs)); });

    // 检查数据数量
    if (xmlFiles.size() == 0) {
        std::cout << folder << "中没有 xml 文件。" << std::endl;
        return false;
    }

    // 按排序后的文件顺序读取全部机器人位姿。
    for (int i = 0; i < xmlFiles.size(); ++i) {
        cv::FileStorage fs(xmlFiles[i], cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cout << "无法打开 \"" << xmlFiles[i] << "\"." << std::endl;
            return false;
        }
        std::cout << "机器人位姿文件：" << xmlFiles[i] << std::endl;
        std::vector<double> poseValues;  // Position0～Position5 共 6 个数据项。
        for (int i = 0; i < 6; ++i) {
            double val = fs[cv::format("Position%d", i)].real();
            poseValues.push_back(val);
        }
        attitudeBigMat.push_back(cv::Mat(poseValues).t());
    }

    for (int i = 0; i < attitudeBigMat.rows; ++i) {
        // 共三组标定数据：第一组旋转部分为旋转向量，第二、三组为欧拉角。
        // cv::Mat tmp = attitudeVectorToMatrix(attitudeBigMat.row(i), useQuaternion, "");  // 旋转向量  对应1
        cv::Mat tmp = attitudeVectorToMatrix(attitudeBigMat.row(i), useQuaternion, "xyz");  // 欧拉角 对应2和3   常用"xyz"

        tmp = tmp.inv();  // 根据眼在手外公式推导，此处需取逆转换矩阵关系

        vecHg.push_back(std::move(tmp));  // 位姿矩阵
    }

    return true;
}

/**
 * @brief 将平移与四元数、欧拉角或旋转向量组成的位姿转换为 4×4 变换矩阵。
 * @param m 6 项为 {x,y,z,rx,ry,rz}，10 项为 {x,y,z,qw,qx,qy,qz,rx,ry,rz}。
 * @param useQuaternion 是否读取四元数分支；该分支要求输入包含 qw、qx、qy、qz。
 * @param seq 欧拉角轴顺序；为空时使用旋转向量。
 * @note 欧拉角单位为度，旋转向量单位为弧度；平移长度单位保持输入约定。
 */

cv::Mat HandEyeCalibration::attitudeVectorToMatrix(cv::Mat m, bool useQuaternion, const std::string& seq) {
    CV_Assert(m.total() == 6 || m.total() == 10);
    if (m.cols == 1) m = m.t();

    cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

    // 从第 4 项起读取 {qw,qx,qy,qz}，输入四元数应已归一化。
    if (useQuaternion) {
        cv::Vec4d quaternionVec = m({3, 0, 4, 1});
        quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({0, 0, 3, 3}));
        // 调试：输出四元数向量的模长。
    } else {
        cv::Mat rotVec;
        if (m.total() == 6)
            rotVec = m({3, 0, 3, 1});  // 6
        else
            rotVec = m({7, 0, 3, 1});  // 10

        // 空轴顺序表示弧度制旋转向量，其余分支按角度制欧拉角处理。
        if (0 == seq.compare(""))
            cv::Rodrigues(rotVec, tmp({0, 0, 3, 3}));
        else
            eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({0, 0, 3, 3}));
    }
    tmp({3, 0, 1, 3}) = m({0, 0, 3, 1}).t();
    // 调试时可交换两个矩阵。
    return tmp;
}

/**
 * @brief 将角度制欧拉角转换为 3×3 旋转矩阵。
 * @param eulerAngle 含三个角度值的行向量，单位为度。
 * @param seq 欧拉角轴顺序，例如 xyz、zyx。
 */

cv::Mat HandEyeCalibration::eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq) {
    CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

    eulerAngle /= 180 / CV_PI;
    cv::Matx13d m(eulerAngle);
    auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);

    cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
    cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
    cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

    cv::Mat rotMat;

    if (seq == "zyx")
        rotMat = rotX * rotY * rotZ;
    else if (seq == "yzx")
        rotMat = rotX * rotZ * rotY;
    else if (seq == "zxy")
        rotMat = rotY * rotX * rotZ;
    else if (seq == "xzy")
        rotMat = rotY * rotZ * rotX;
    else if (seq == "yxz")
        rotMat = rotZ * rotX * rotY;
    else if (seq == "xyz")
        rotMat = rotZ * rotY * rotX;
    else if (seq == "zyz")
        rotMat = rotZ * rotY * rotZ;  // 新添加
    else {
        cv::error(cv::Error::StsAssert, "欧拉角顺序字符串错误。", __FUNCTION__, __FILE__, __LINE__);
    }

    if (!isRotationMatrix(rotMat)) {
        cv::error(cv::Error::StsAssert, "欧拉角无法转换为旋转矩阵。", __FUNCTION__, __FILE__, __LINE__);
    }

    return rotMat;
    // 调试：输出旋转矩阵合法性判断结果。
}

/** @brief 四元数转旋转矩阵
 *
 * @note 数据类型为 double；四元数定义 q = w + x*i + y*j + z*k
 *
 * @param q 四元数输入 {w,x,y,z} 向量
 *
 * @return 返回 3*3 旋转矩阵
 */
cv::Mat HandEyeCalibration::quaternionToRotatedMatrix(const cv::Vec4d& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double x2 = x * x, y2 = y * y, z2 = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    cv::Matx33d res{
        1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy), 2 * (xy + wz),     1 - 2 * (x2 + z2),
        2 * (yz - wx),     2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2),
    };

    return cv::Mat(res);
}

// 正交旋转块满足 RᵀR = I，使用矩阵范数判断其偏差。
bool HandEyeCalibration::isRotationMatrix(const cv::Mat& R) {
    cv::Mat tmp33 = R({0, 0, 3, 3});
    cv::Mat shouldBeIdentity;

    shouldBeIdentity = tmp33.t() * tmp33;

    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1e-6;
}

/**
 * @brief 根据已校正的图像坐标和相机深度，求解基坐标系下的三维位置。
 * @param imgPt 图像坐标 (u,v)，单位为像素。
 * @param z 相机坐标系的 Z 深度，与姿态矩阵平移使用相同长度单位。
 * @param Hcg 相机到夹具的 4×4 手眼变换矩阵。
 * @param Hg 夹具到基座的 4×4 位姿矩阵。
 * @param camMatrix 3×3 相机内参矩阵。
 * @return 基坐标系中的三维点，长度单位与输入平移相同。
 */
cv::Point3d HandEyeCalibration::getWorldPos(cv::Point2d& imgPt, double z, cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& camMatrix) {
    CV_Assert(Hcg.type() == CV_64FC1 && Hg.type() == CV_64FC1 && camMatrix.type() == CV_64FC1);

    cv::Mat imgPosHomo = cv::Mat::ones(3, 1, CV_64F);
    cv::Mat(imgPt).copyTo(imgPosHomo({0, 0, 1, 2}));

    cv::Mat camPosHomo = cv::Mat::ones(4, 1, CV_64F);
    cv::Mat camMatInv = camMatrix.inv(cv::DECOMP_SVD);
    cv::Mat camPos = camMatInv * z * imgPosHomo;
    camPos.copyTo(camPosHomo({0, 0, 1, 3}));

    cv::Mat resHomo = Hg * Hcg * camPosHomo;
    cv::Mat res = resHomo({0, 0, 1, 3});

    return cv::Point3d(res);
}

// 从配置文件中读取相机内参和畸变系数。
bool HandEyeCalibration::readCameraParameters(const std::string& filename, cv::Mat& camMatrix, cv::Mat& distCoefs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "open \"" << filename << "\" failed." << std::endl;
        return false;
    }
    camMatrix = fs["camera_matrix"].mat();
    distCoefs = fs["distortion_coefficients"].mat();
    fs.release();

    return true;
}

/**
 * @brief 求解眼在手上场景的相机到夹具变换。
 * @param Hcg 输出 4×4 相机到夹具的手眼变换矩阵。
 * @param vecHg 夹具到基座的位姿矩阵序列。
 * @param vecHc 标定板到相机的位姿矩阵序列。
 * @param method 手眼求解方法。
 * @param file 保存手眼矩阵的文件名。
 */
void HandEyeCalibration::calibrateEyeInHand(cv::Mat& Hcg, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc, HandEyeCaliMethod method,
                                            const std::string& file) {
    // 构造 AX = XB 的相对运动数据对。
    std::vector<cv::Mat> vecHgij, vecHcij;
    convertRelativePoses(vecHg, vecHc, vecHgij, vecHcij);
    // 求解并保存手眼变换矩阵 X。
    computeHandEyeMatrix(Hcg, vecHgij, vecHcij, method, file);

    return;
}
// 参考实现：https://github.com/wanggao1990/HandEyeCalibration/blob/fa3d161b554702b0e670291a6e842c102efb58b0/calibration/HandEyeCalibration.cpp
