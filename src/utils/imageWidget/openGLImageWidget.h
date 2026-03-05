#ifndef OPENGLIMAGEWIDGET_H
#define OPENGLIMAGEWIDGET_H

#include <QImage>
#include <QMouseEvent>
#include <QMutex>
#include <QOpenGLWidget>
#include <algorithm>
#include <opencv2/opencv.hpp>

class openGLImageWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit openGLImageWidget(QWidget *parent = nullptr);
    void setOpenCVImage(const cv::Mat &mat);

    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

    bool backgroundColor = 0;  // 背景颜色, 默认黑色

private slots:
    void resetView();  // 槽函数：重置画面

protected:
    void paintEvent(QPaintEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;

private:
    QImage matToQImage(const cv::Mat &mat);

private:
    QImage image_;
    QMutex mutex_;
    QPointF offset_ = QPointF(0, 0);

    double scaleFactor_ = 1.0;
    QPointF translation_;            // 当前平移
    QPoint lastMousePos_;            // 上一次鼠标位置
    bool isPanning_ = false;         // 是否正在平移状态
    bool firstLoad_ = true;          // 是否是第一次加载图像
    bool hasUserTransform_ = false;  // 是否用户缩放/平移过
};

#endif  // OPENGLIMAGEWIDGET_H
