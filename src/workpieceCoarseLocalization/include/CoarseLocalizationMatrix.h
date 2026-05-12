#ifndef CoarseLocalizationMatrix_H
#define CoarseLocalizationMatrix_H

#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

extern int cameraIndex;

class CoarseLocalizationMatrix {
public:
    std::string CameraSerialNum;  // 新增序列号字段
    std::vector<double> cameraMatrixData;
    std::vector<double> distCoeffsData;
    std::vector<double> globalPlaneData;
    std::vector<double> extrinsicMatrixData;

    // 构造函数，初始化矩阵数据
    CoarseLocalizationMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs, std::vector<double> globalPlane, cv::Mat extrinsicMatrix)
        : globalPlaneData(globalPlane) {
        cameraMatrixData = std::vector<double>(cameraMatrix.begin<double>(), cameraMatrix.end<double>());
        distCoeffsData = std::vector<double>(distCoeffs.begin<double>(), distCoeffs.end<double>());
        extrinsicMatrixData = std::vector<double>(extrinsicMatrix.begin<double>(), extrinsicMatrix.end<double>());
        // this->globalPlaneData = globalPlane;
    }
    CoarseLocalizationMatrix() {}  // 默认构造函数1
    // 序列化支持
    // clang-format off
    template <class Archive>
    void serialize(Archive& ar) {
        ar(
            cereal::make_nvp("CameraSerialNum", CameraSerialNum),
            cereal::make_nvp("CameraMatrix", cameraMatrixData),
            cereal::make_nvp("DistortionCoefficients", distCoeffsData),
            cereal::make_nvp("GlobalPlane", globalPlaneData),
            cereal::make_nvp("extrinsicMatrix", extrinsicMatrixData));
    }
    // clang-format on
    void transferData(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<double>& globalPlane, cv::Mat& extrinsicMatrix) {
        // 将反序列化后的数据恢复为 cv::Mat
        if (cameraMatrixData.size() == 9) {
            cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixData.data()).clone();
        } else {
            std::cerr << "Error: Camera Matrix size is incorrect!" << std::endl;
            return;
        }

        if (distCoeffsData.size() == 5) {
            distCoeffs = cv::Mat(1, 5, CV_64F, distCoeffsData.data()).clone();
        } else {
            std::cerr << "Error: Distortion Coefficients size is incorrect!" << std::endl;
            return;
        }
        globalPlane = globalPlaneData;
        if (extrinsicMatrixData.size() == 16) {
            extrinsicMatrix = cv::Mat(4, 4, CV_64F, extrinsicMatrixData.data()).clone();
        } else {
            std::cerr << "Error: extrinsic Matrix size is incorrect!" << std::endl;
            return;
        }
    }
    void saveCalibConfigToFile() {
        // 构造3个相机的CoarseLocalizationMatrix实例
        CoarseLocalizationMatrix cam1;
        cam1.CameraSerialNum = "21158836";
        // 这里示例给cameraMatrixData填充9个double
        cam1.cameraMatrixData = {1891.93, 0.0, 798.93, 0.0, 1886.25, 617.64, 0.0, 0.0, 1.0};
        // distCoeffsData示例填充5个double
        cam1.distCoeffsData = {0, 0, 0, 0, 0};
        cam1.globalPlaneData = {0, 0, 0};
        cam1.extrinsicMatrixData = std::vector<double>(16, 0.0);  // 全零示例

        CoarseLocalizationMatrix cam2 = cam1;
        cam2.CameraSerialNum = "22256419";
        cam2.cameraMatrixData = {1856.43, 0.0, 768.45, 0.0, 1851.80, 595.30, 0.0, 0.0, 1.0};

        CoarseLocalizationMatrix cam3 = cam1;
        cam3.CameraSerialNum = "22301065";
        cam3.cameraMatrixData = {1850.00, 0.0, 760.00, 0.0, 1840.00, 590.00, 0.0, 0.0, 1.0};

        // 用std::map组织数据，key就是Camera1, Camera2, Camera3
        std::map<std::string, CoarseLocalizationMatrix> cameraMap;
        cameraMap["Camera1"] = cam1;
        cameraMap["Camera2"] = cam2;
        cameraMap["Camera3"] = cam3;

        // 打开文件保存
        std::ofstream os("./data/config/workpiece_localization_calib.json");
        if (!os.is_open()) {
            std::cerr << "Failed to open file for writing: " << "./data/config/workpiece_localization_calib.json" << std::endl;
            return;
        }

        cereal::JSONOutputArchive archive(os);
        archive(cereal::make_nvp("Cameras", cameraMap));
    }
};

class MyMatrixTrackDirection {
public:
    std::vector<double> TrackDirectionData;

    // 构造函数，初始化矩阵数据
    MyMatrixTrackDirection(cv::Mat TrackDirectionData) {
        this->TrackDirectionData = std::vector<double>(TrackDirectionData.begin<double>(), TrackDirectionData.end<double>());
    }
    MyMatrixTrackDirection() {}  // 默认构造函数
    // 反序列化支持
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("TrackDirection", TrackDirectionData));
    }
    void appendToJsonFile(const std::string& filename) {
        std::ofstream file(filename);  // 打开文件，使用追加模式 std::ios::app
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file for appending!" << std::endl;
            return;
        }

        cereal::JSONOutputArchive archive(file);
        archive(cereal::make_nvp("TrackDirectionData", *this));  // 序列化对象
        // file.close();
    }
    cv::Mat readTrackDirectionFromJsonFile(const std::string& filename) {
        cv::Mat TrackDirection;
        std::ifstream file(filename);  // 打开文件读取
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file for reading!" << std::endl;
            return cv::Mat();
        }
        cereal::JSONInputArchive archive(file);
        // 只反序列化 TrackDirection 字段
        archive(cereal::make_nvp("TrackDirectionData", *this));
        file.close();

        if (TrackDirectionData.size() == 3) {
            TrackDirection = cv::Mat(3, 1, CV_64F, TrackDirectionData.data()).clone();
        } else {
            std::cerr << "Error: Camera Matrix size is incorrect!" << std::endl;
            return cv::Mat();
        }
        return TrackDirection;
    }
};

class CalibConfig {
public:
    std::string primaryCameraSerialNum;
    std::string secondaryCameraSerialNum;
    std::string thirdaryCameraSerialNum;
    CalibConfig() {}

    // cereal序列化支持
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("primaryCameraSerialNum", primaryCameraSerialNum), cereal::make_nvp("secondaryCameraSerialNum", secondaryCameraSerialNum),
           cereal::make_nvp("thirdaryCameraSerialNum", thirdaryCameraSerialNum));
    }
};
// 视点规划读取
struct ViewTransformConfig {
    std::string name;

    // 基座偏移
    std::vector<double> offsetXYZ{0.0, 0.0, 0.0};

    // 4x4齐次矩阵（二维数组格式）
    std::vector<std::vector<double>> T{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(name), CEREAL_NVP(offsetXYZ), CEREAL_NVP(T));
    }
    
    // 辅助方法：将二维数组T转换为cv::Mat
    cv::Mat getTransformMatrix() const {
        if (T.size() != 4) {
            std::cerr << "Error: Transform matrix T size is incorrect!" << std::endl;
            return cv::Mat::eye(4, 4, CV_64F);
        }
        cv::Mat mat(4, 4, CV_64F);
        for (int i = 0; i < 4; i++) {
            if (T[i].size() != 4) {
                std::cerr << "Error: Transform matrix T row " << i << " size is incorrect!" << std::endl;
                return cv::Mat::eye(4, 4, CV_64F);
            }
            std::memcpy(mat.ptr<double>(i), T[i].data(), 4 * sizeof(double));
        }
        return mat;
    }
    
    // 辅助方法：从cv::Mat设置T
    void setTransformMatrix(const cv::Mat& mat) {
        if (mat.rows != 4 || mat.cols != 4 || mat.type() != CV_64F) {
            std::cerr << "Error: Invalid transform matrix for setTransformMatrix!" << std::endl;
            return;
        }
        T.resize(4);
        for (int i = 0; i < 4; i++) {
            T[i].assign(mat.ptr<double>(i), mat.ptr<double>(i) + 4);
        }
    }
};

struct ViewClassConfig {
    int classId = -1;

    std::string className;

    std::vector<ViewTransformConfig> transforms;

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(classId), CEREAL_NVP(className), CEREAL_NVP(transforms));
    }
};

class ViewPlanningConfig {
public:
    static ViewPlanningConfig& getInstance();

    std::map<int, ViewClassConfig> classConfigs;

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(classConfigs));
    }

    void readConfig();
    void writeConfig();
    void printConfig();
    void initDefaultConfig();

private:
    ViewPlanningConfig() = default;

private:
    friend class cereal::access;
    std::string configPath = "./data/config/ViewPlanningConfig.json";

    static ViewPlanningConfig* instance;
    static std::mutex mutex_;
};
#endif  // CoarseLocalizationMatrix_H
