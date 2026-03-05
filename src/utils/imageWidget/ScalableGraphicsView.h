#ifndef SCALABLEGRAPHICSVIEW_H
#define SCALABLEGRAPHICSVIEW_H

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QMouseEvent>
#include <QScrollBar>
#include <QWheelEvent>

class ScalableGraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    // 定义构造函数，接受 QGraphicsScene 和 QWidget* 作为参数
    explicit ScalableGraphicsView(QWidget *parent = nullptr);  // 父类为 QWidget*
    void wheelEvent(QWheelEvent *event) override;              // 处理鼠标滚轮事件
    void mousePressEvent(QMouseEvent *event) override;         // 处理鼠标按下事件
    void setOriginalImageInfo(int width, int height, int angle);

signals:
    void senderSignalPixelCoordinates(const int x, const int y);

private:
    bool isDragging = false;  // 是否正在拖拽
    QPoint lastMousePos;      // 上一次鼠标位置
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    int originalImageWidth = 0;
    int originalImageHeight = 0;
    int rotationAngle = 0;  // 旋转角度：0, 90, 180, 270
};
class CoordinateMapper {
public:
    // 坐标映射类型枚举
    enum class CoordMappingType {
        XY,         // x->x, y->y
        NegX_Y,     // x->-x, y->y
        X_NegY,     // x->x, y->-y
        NegX_NegY,  // x->-x, y->-y
        YX,         // x->y, y->x
        NegY_X,     // x->-y, y->x
        Y_NegX,     // x->y, y->-x
        NegY_NegX   // x->-y, y->-x
    };

    // 映射函数：将 rel 从像素坐标变换到世界坐标系
    static cv::Point2d relativeMapToCoord(const cv::Point2d &rel, const cv::Point2d &worldCenter, CoordMappingType type) {
        double dx = rel.x;
        double dy = rel.y;
        double x = worldCenter.x;
        double y = worldCenter.y;

        switch (type) {
            case CoordMappingType::XY:
                return {x + dx, y + dy};
            case CoordMappingType::NegX_Y:
                return {x - dx, y + dy};
            case CoordMappingType::X_NegY:
                return {x + dx, y - dy};
            case CoordMappingType::NegX_NegY:
                return {x - dx, y - dy};
            case CoordMappingType::YX:
                return {x + dy, y + dx};
            case CoordMappingType::NegY_X:
                return {x - dy, y + dx};
            case CoordMappingType::Y_NegX:
                return {x + dy, y - dx};
            case CoordMappingType::NegY_NegX:
                return {x - dy, y - dx};
            default:
                return {x + dx, y + dy};  // 默认情况：不做变换
        }
    }
    template <typename T>
    static cv::Point_<T> rotateToOriginal(const cv::Point_<T> &rotatedPoint, int originalImageWidth, int originalImageHeight,
                                          int rotationAngle) {
        T x_rotated = rotatedPoint.x;
        T y_rotated = rotatedPoint.y;
        T x_original = 0;
        T y_original = 0;

        switch (rotationAngle) {
            case 0:
                x_original = x_rotated;
                y_original = y_rotated;
                break;
            case 90:
                x_original = y_rotated;
                y_original = static_cast<T>(originalImageWidth - x_rotated - 1);
                break;
            case 180:
                x_original = static_cast<T>(originalImageWidth - x_rotated - 1);
                y_original = static_cast<T>(originalImageHeight - y_rotated - 1);
                break;
            case 270:
                x_original = static_cast<T>(originalImageHeight - y_rotated - 1);
                y_original = x_rotated;
                break;
            default:
                x_original = x_rotated;
                y_original = y_rotated;
                break;
        }

        return cv::Point_<T>(x_original, y_original);
    }
    template <typename T>
    static cv::Point_<T> originalToRotated(const cv::Point_<T> &originalPoint, int originalImageWidth, int originalImageHeight,
                                           int rotationAngle) {
        T x_original = originalPoint.x;
        T y_original = originalPoint.y;
        T x_rotated = 0;
        T y_rotated = 0;

        switch (rotationAngle) {
            case 0:
                x_rotated = x_original;
                y_rotated = y_original;
                break;
            case 90:
                x_rotated = static_cast<T>(originalImageWidth - y_original - 1);
                y_rotated = x_original;
                break;
            case 180:
                x_rotated = static_cast<T>(originalImageWidth - x_original - 1);
                y_rotated = static_cast<T>(originalImageHeight - y_original - 1);
                break;
            case 270:
                x_rotated = y_original;
                y_rotated = static_cast<T>(originalImageHeight - x_original - 1);
                break;
            default:
                x_rotated = x_original;
                y_rotated = y_original;
                break;
        }

        return cv::Point_<T>(x_rotated, y_rotated);
    }
};
#endif  // SCALABLEGRAPHICSVIEW_H
