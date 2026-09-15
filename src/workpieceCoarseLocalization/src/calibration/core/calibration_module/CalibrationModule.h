#ifndef CALIBRATIONMODULE_H
#define CALIBRATIONMODULE_H

#include <string>
#include <vector>

#include "model/CoreTypes.h"
#include "include/CoarseLocalizationMatrix.h"

namespace CalibrationModule {

/**
 * @brief hasCurrentCalibrationResult  判断当前进程内是否已经缓存标定结果
 * @return true 表示已有缓存，false 表示暂无缓存
 */
bool hasCurrentCalibrationResult();

/**
 * @brief currentCalibrationResult     获取当前进程内缓存的标定结果引用
 * @return 当前缓存的 CoarseLocalizationMatrix 标定结果引用
 * @note 仅建议在确认没有并发写入时使用，跨线程读取优先使用 getCurrentCalibrationResult
 */
const CoarseLocalizationMatrix& currentCalibrationResult();

/**
 * @brief getCurrentCalibrationResult  复制获取当前进程内缓存的标定结果
 * @param calibrationResult            输出的标定结果
 * @return true 表示复制成功，false 表示当前没有缓存
 */
bool getCurrentCalibrationResult(CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief setCurrentCalibrationResult  设置当前进程内缓存的标定结果
 * @param calibrationResult            需要缓存的完整标定结果
 */
void setCurrentCalibrationResult(const CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief clearCurrentCalibrationResult 清空当前进程内缓存的标定结果
 */
void clearCurrentCalibrationResult();

/**
 * @brief loadCalibrationResultFile    从 json 文件读取标定结果，并同步更新内存缓存
 * @param filename                     标定结果 json 文件路径
 * @param calibrationResult            输出读取到的标定结果
 * @return true 表示读取成功，false 表示读取失败
 */
bool loadCalibrationResultFile(const std::string& filename, CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief saveCalibrationResultFile    将标定结果保存到 json 文件，并同步更新内存缓存
 * @param filename                     标定结果 json 文件路径
 * @param calibrationResult            需要保存的标定结果
 * @return true 表示保存成功，false 表示保存失败
 */
bool saveCalibrationResultFile(const std::string& filename, const CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief loadCurrentCalibrationResultFile 从 json 文件读取标定结果并直接作为当前缓存
 * @param filename                         标定结果 json 文件路径
 * @return true 表示读取并缓存成功，false 表示读取失败
 */
bool loadCurrentCalibrationResultFile(const std::string& filename);

/**
 * @brief saveCurrentCalibrationResultFile 将当前内存缓存的标定结果保存到 json 文件
 * @param filename                         标定结果 json 文件路径
 * @return true 表示保存成功，false 表示当前无缓存或保存失败
 */
bool saveCurrentCalibrationResultFile(const std::string& filename);

/**
 * @brief updateCameraCalibrationResult 将相机内参和平面标定结果写入 CoarseLocalizationMatrix
 * @param cameraCalibrationResult        相机内参标定结果
 * @param planeCalibrationResult         工作平面标定结果
 * @param calibrationResult              输入输出的完整标定结果
 */
void updateCameraCalibrationResult(const CameraCalibrationResult& cameraCalibrationResult,
                                   const PlaneCalibrationResult& planeCalibrationResult,
                                   CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief updateHandEyeCalibrationResult 将手眼标定结果写入 CoarseLocalizationMatrix
 * @param handEyeCalibrationResult       手眼标定结果
 * @param calibrationResult              输入输出的完整标定结果
 */
void updateHandEyeCalibrationResult(const HandEyeCalibrationResult& handEyeCalibrationResult,
                                    CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief updateExternalAxisCalibrationResult 将单个外部轴标定结果写入 CoarseLocalizationMatrix
 * @param externalAxisCalibrationResult       外部轴标定结果
 * @param calibrationResult                   输入输出的完整标定结果
 */
void updateExternalAxisCalibrationResult(const ExternalAxisCalibrationResult& externalAxisCalibrationResult,
                                         CoarseLocalizationMatrix& calibrationResult);

/**
 * @brief loadCalibrationImagePaths     读取标定数据根目录下的内参图片和平面图片路径
 * @param calibPath                     标定数据根目录
 * @param calibrationImagePaths         输出分类后的图片路径集合
 */
void loadCalibrationImagePaths(const std::string& calibPath, CalibrationImagePaths& calibrationImagePaths);

/**
 * @brief runCameraCalibration          相机内参标定
 * @param imagePaths                    内参标定图片路径列表
 * @param cameraCalibrationResult       输出内参矩阵、畸变系数、每张图外参和误差
 * @return true 表示标定成功，false 表示标定失败
 */
bool runCameraCalibration(std::vector<std::string>& imagePaths, CameraCalibrationResult& cameraCalibrationResult);

/**
 * @brief runPlaneCalibration           工作平面标定
 * @param planeImagePaths               工作平面标定图片路径列表
 * @param cameraCalibrationResult       已完成的相机内参标定结果
 * @param planeCalibrationResult        输出工作平面方程和误差
 * @return true 表示标定成功，false 表示标定失败
 */
bool runPlaneCalibration(std::vector<std::string>& planeImagePaths,
                         const CameraCalibrationResult& cameraCalibrationResult,
                         PlaneCalibrationResult& planeCalibrationResult);

/**
 * @brief runHandEyeCalibration         手眼标定
 * @param imagePathEyeToHand            手眼标定图片目录
 * @param posePath                      与图片对应的机器人位姿数据目录
 * @param cameraMatrix                  相机内参矩阵
 * @param distCoeffs                    畸变系数
 * @param boardConfig                   标定板参数
 * @param handEyeCalibrationResult      输出手眼矩阵和误差统计
 * @return true 表示标定成功，false 表示标定失败
 */
bool runHandEyeCalibration(std::string imagePathEyeToHand,
                           const std::string& posePath,
                           const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs,
                           const BoardConfig& boardConfig,
                           HandEyeCalibrationResult& handEyeCalibrationResult);

/**
 * @brief runExternalAxisCalibration    多个外部轴方向向量标定
 * @param calibrationRequest            外部轴标定所需的相机、手眼、图片和编码器数据
 * @param axisTypes                     需要标定的轴类型集合
 * @param calibrationResults            输出每个轴的标定结果
 * @param errorMessage                  输出失败原因
 * @return true 表示全部标定成功，false 表示至少一个轴标定失败
 */
bool runExternalAxisCalibration(const ExternalAxisCalibrationRequest& calibrationRequest,
                                const std::vector<ExternalAxisType>& axisTypes,
                                std::vector<ExternalAxisCalibrationResult>& calibrationResults,
                                std::string& errorMessage);

/**
 * @brief runExternalAxisCalibration    单个外部轴方向向量标定
 * @param calibrationRequest            外部轴标定所需的相机、手眼、图片和编码器数据
 * @param axisType                      需要标定的轴类型
 * @param calibrationResult             输出该轴的标定结果
 * @param errorMessage                  输出失败原因
 * @return true 表示标定成功，false 表示标定失败
 */
bool runExternalAxisCalibration(const ExternalAxisCalibrationRequest& calibrationRequest,
                                ExternalAxisType axisType,
                                ExternalAxisCalibrationResult& calibrationResult,
                                std::string& errorMessage);

/**
 * @brief axisTypeToName                将外部轴类型转换为字符串名称
 * @param axisType                      外部轴类型
 * @return 轴名称字符串
 */
std::string axisTypeToName(ExternalAxisType axisType);

/**
 * @brief projectPointToAxisComponent   按旧项目 debugProjectPointOnlyY 逻辑计算点在方向向量上的分量
 * @param trackDirection                机械臂基坐标系下的外部轴单位方向向量
 * @param pt                            机械臂基坐标系下的空间点
 * @param projectionComponentIndex      用于投影的坐标分量索引，0 为 X，1 为 Y，2 为 Z
 * @return 点在方向向量上的分量值
 */
double projectPointToAxisComponent(const cv::Mat& trackDirection,
                                   const cv::Point3d& pt,
                                   int projectionComponentIndex = 1);

/**
 * @brief calculateRequiredMoveValue    计算目标点和当前工具点在外部轴方向上的相对移动量
 * @param trackDirection                机械臂基坐标系下的外部轴单位方向向量
 * @param targetPointInBase             机械臂基坐标系下的目标点
 * @param currentToolPointInBase        机械臂基坐标系下的当前工具点
 * @param projectionComponentIndex      用于投影的坐标分量索引，0 为 X，1 为 Y，2 为 Z
 * @return 外部轴需要移动的相对量
 */
double calculateRequiredMoveValue(const cv::Mat& trackDirection,
                                  const cv::Point3d& targetPointInBase,
                                  const cv::Point3d& currentToolPointInBase,
                                  int projectionComponentIndex = 1);

/**
 * @brief localizeExternalAxisCoordinate 根据像素点计算单个外部轴的目标编码器值
 * @param localizationRequest            单轴定位输入数据
 * @param localizationResult             输出单轴定位结果
 * @param errorMessage                   输出失败原因
 * @return true 表示计算成功，false 表示计算失败
 */
bool localizeExternalAxisCoordinate(const ExternalAxisLocalizationRequest& localizationRequest,
                                    ExternalAxisLocalizationResult& localizationResult,
                                    std::string& errorMessage);

/**
 * @brief localizeThreeAxisCoordinate   根据同一个像素点计算三轴目标编码器值
 * @param xAxisRequest                  X 轴定位输入数据
 * @param yAxisRequest                  Y 轴定位输入数据
 * @param zAxisRequest                  Z 轴定位输入数据
 * @param localizationResult            输出三轴定位结果
 * @param errorMessage                  输出失败原因
 * @return true 表示计算成功，false 表示计算失败
 */
bool localizeThreeAxisCoordinate(const ExternalAxisLocalizationRequest& xAxisRequest,
                                 const ExternalAxisLocalizationRequest& yAxisRequest,
                                 const ExternalAxisLocalizationRequest& zAxisRequest,
                                 ThreeAxisLocalizationResult& localizationResult,
                                 std::string& errorMessage);

/**
 * @brief calculateExternalAxisMoveToPlane 计算单个外部轴末端到达工作平面目标点所需移动量
 * @param moveRequest                       单轴移动量计算输入数据
 * @param moveResult                        输出单轴移动量计算结果
 * @param errorMessage                      输出失败原因
 * @return true 表示计算成功，false 表示计算失败
 */
bool calculateExternalAxisMoveToPlane(const ExternalAxisMoveToPlaneRequest& moveRequest,
                                      ExternalAxisMoveToPlaneResult& moveResult,
                                      std::string& errorMessage);

/**
 * @brief calculateThreeAxisMoveToPlane 三轴末端到达工作平面目标点所需移动量计算
 * @param xAxisRequest                  X 轴移动量计算输入数据
 * @param yAxisRequest                  Y 轴移动量计算输入数据
 * @param zAxisRequest                  Z 轴移动量计算输入数据
 * @param moveResult                    输出三轴移动量计算结果
 * @param errorMessage                  输出失败原因
 * @return true 表示计算成功，false 表示计算失败
 */
bool calculateThreeAxisMoveToPlane(const ExternalAxisMoveToPlaneRequest& xAxisRequest,
                                   const ExternalAxisMoveToPlaneRequest& yAxisRequest,
                                   const ExternalAxisMoveToPlaneRequest& zAxisRequest,
                                   ThreeAxisMoveToPlaneResult& moveResult,
                                   std::string& errorMessage);

}  // namespace CalibrationModule

#endif  // CALIBRATIONMODULE_H
