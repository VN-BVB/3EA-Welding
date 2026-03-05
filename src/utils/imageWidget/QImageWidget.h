#ifndef QIMAGEWIDGET_H
#define QIMAGEWIDGET_H

#include <plog/Init.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

#include <QDebug>
#include <QImage>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPixmap>
#include <QRgb>
#include <QWidget>
#include <opencv2/opencv.hpp>
using namespace cv;

#define USE_OPENCV

class QImageWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit QImageWidget(QWidget *parent = nullptr);
    ~QImageWidget();
    void setImage(QImage &image);
    void setImage(const uchar *data, int data_length, int width, int height);
#ifdef USE_OPENCV
    void setOpenCVImage(cv::Mat img);
    void setOpenCVImage2(cv::Mat img);
    void setMode(int mode);
#endif
    void clear();
    QImage getDisplayImage();  // 获得当前显示的图片
    void setPenSize(int);      // 设置画笔大小
    void initSize();           // 初始大小
    QImage getCurrentArea();   // 获取当前窗口显示区域
    int mode() const;

protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    bool validatePosInImage(QPoint pos);  // 判断点是否在图片中
    void drawImage(QPainter *painter);    // 绘制图片

private:
    QImage displayImage, displayImage2, originalImage;
    int mMode = 0;
    QVector<QRgb> colorTable;
    QMatrix matrix;  // 缩放移动矩阵
    QMutex mutex;    // 枷锁
    QPoint mouseWheelPoint, mousePressPoint, mouseReleasePoint, mouseMovePoint;

    double olderMoveX, olderMoveY;  // 保存前一次移动距离
    int moveX, moveY;               // 当前移动距离
    double scale, preScale;         // 放大比例

    QPoint maskPt;
    int penSize;  // 画笔大小
    bool bMoveFlag;
    QPixmap penCursor, xpcCursor;
    QCursor penQCursor, xpcQCursor;

    int nw, nh, preNw, preNh, niw, nih;
    double dScaleTmp, dScaleW, dScaleH, adjustX, adjustY;
    QPainter *painter;
    bool isInit;

signals:
    void pic_moved();
};

#endif  // QIMAGEWIDGET_H
