#ifndef MYMATRIX_H
#define MYMATRIX_H

#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class MyMatrix {
public:
    std::vector<double> cameraMatrixData;
    std::vector<double> distCoeffsData;
    std::vector<double> globalPlaneData;
    std::vector<double> cameraToBaseMatrixData;
    std::vector<double> xAxisCameraToBaseMatrixData;
    std::vector<double> yAxisCameraToBaseMatrixData;
    std::vector<double> zAxisCameraToBaseMatrixData;
    std::vector<double> xAxisTrackDirectionData;
    std::vector<double> yAxisTrackDirectionData;
    std::vector<double> zAxisTrackDirectionData;
    double xAxisReferenceEncoderValue = 2500.0;
    double yAxisReferenceEncoderValue = 1000.0;
    double zAxisReferenceEncoderValue = 500.0;

    MyMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs, std::vector<double> globalPlane, cv::Mat cameraToBaseMatrix)
        : globalPlaneData(globalPlane) {
        cameraMatrixData = std::vector<double>(cameraMatrix.begin<double>(), cameraMatrix.end<double>());
        distCoeffsData = std::vector<double>(distCoeffs.begin<double>(), distCoeffs.end<double>());
        cameraToBaseMatrixData = std::vector<double>(cameraToBaseMatrix.begin<double>(), cameraToBaseMatrix.end<double>());
    }

    MyMatrix() {}

    template <class Archive>
    void save(Archive& ar) const {
        ar(cereal::make_nvp("CameraMatrix", cameraMatrixData),
           cereal::make_nvp("DistortionCoefficients", distCoeffsData),
           cereal::make_nvp("GlobalPlane", globalPlaneData),
           cereal::make_nvp("cameraToBaseMatrix", cameraToBaseMatrixData),
           cereal::make_nvp("xAxisCameraToBaseMatrix", xAxisCameraToBaseMatrixData),
           cereal::make_nvp("yAxisCameraToBaseMatrix", yAxisCameraToBaseMatrixData),
           cereal::make_nvp("zAxisCameraToBaseMatrix", zAxisCameraToBaseMatrixData),
           cereal::make_nvp("xAxisTrackDirection", xAxisTrackDirectionData),
           cereal::make_nvp("yAxisTrackDirection", yAxisTrackDirectionData),
           cereal::make_nvp("zAxisTrackDirection", zAxisTrackDirectionData),
           cereal::make_nvp("xAxisReferenceEncoderValue", xAxisReferenceEncoderValue),
           cereal::make_nvp("yAxisReferenceEncoderValue", yAxisReferenceEncoderValue),
           cereal::make_nvp("zAxisReferenceEncoderValue", zAxisReferenceEncoderValue));
    }

    template <class Archive>
    void load(Archive& ar) {
        ar(cereal::make_nvp("CameraMatrix", cameraMatrixData),
           cereal::make_nvp("DistortionCoefficients", distCoeffsData),
           cereal::make_nvp("GlobalPlane", globalPlaneData),
           cereal::make_nvp("cameraToBaseMatrix", cameraToBaseMatrixData));

        auto loadOptionalVectorField = [&ar](const char* fieldName, std::vector<double>& fieldValue) {
            try {
                ar(cereal::make_nvp(fieldName, fieldValue));
            } catch (...) {
                fieldValue.clear();
            }
        };

        auto loadOptionalScalarField = [&ar](const char* fieldName, double& fieldValue, double defaultValue) {
            try {
                ar(cereal::make_nvp(fieldName, fieldValue));
            } catch (...) {
                fieldValue = defaultValue;
            }
        };

        // Keep older calibration files readable when the three-axis fields are missing.
        loadOptionalVectorField("xAxisCameraToBaseMatrix", xAxisCameraToBaseMatrixData);
        loadOptionalVectorField("yAxisCameraToBaseMatrix", yAxisCameraToBaseMatrixData);
        loadOptionalVectorField("zAxisCameraToBaseMatrix", zAxisCameraToBaseMatrixData);
        loadOptionalVectorField("xAxisTrackDirection", xAxisTrackDirectionData);
        loadOptionalVectorField("yAxisTrackDirection", yAxisTrackDirectionData);
        loadOptionalVectorField("zAxisTrackDirection", zAxisTrackDirectionData);

        loadOptionalScalarField("xAxisReferenceEncoderValue", xAxisReferenceEncoderValue, 2500.0);
        loadOptionalScalarField("yAxisReferenceEncoderValue", yAxisReferenceEncoderValue, 1000.0);
        loadOptionalScalarField("zAxisReferenceEncoderValue", zAxisReferenceEncoderValue, 500.0);
    }

    void transferData(cv::Mat& cameraMatrix,
                      cv::Mat& distCoeffs,
                      std::vector<double>& globalPlane,
                      cv::Mat& cameraToBaseMatrix) const {
        if (cameraMatrixData.size() == 9) {
            cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(cameraMatrixData.data())).clone();
        } else {
            std::cerr << "Error: Camera Matrix size is incorrect!" << std::endl;
            return;
        }

        if (distCoeffsData.size() == 4) {
            distCoeffs = cv::Mat(1, 4, CV_64F, const_cast<double*>(distCoeffsData.data())).clone();
        } else if (distCoeffsData.size() == 5) {
            distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(distCoeffsData.data())).clone();
        } else {
            std::cerr << "Error: Distortion Coefficients size is incorrect!" << std::endl;
            return;
        }

        globalPlane = globalPlaneData;

        if (cameraToBaseMatrixData.size() == 16) {
            cameraToBaseMatrix = cv::Mat(4, 4, CV_64F, const_cast<double*>(cameraToBaseMatrixData.data())).clone();
        } else {
            std::cerr << "Error: cameraToBaseMatrix size is incorrect!" << std::endl;
            return;
        }
    }
};

#endif  // MYMATRIX_H
