#ifndef ERRORSAVE_H
#define ERRORSAVE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <plog/Log.h>

#include <QObject>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <vector>

class WeldSeamInfo;

class ErrorSave : public QObject {
    Q_OBJECT
public:
    explicit ErrorSave(QObject *parent = nullptr);

    void creatorTimeStr();  // 生成当前时间字符串

private:
    std::string time_str;  // 图像保存时间字符串
};

#endif  // ERRORSAVE_H
