#include "openGLImageWidget.h"

#include <QContextMenuEvent>
#include <QMenu>
#include <QPainter>

template <typename T>
T clamp(const T &v, const T &lo, const T &hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

openGLImageWidget::openGLImageWidget(QWidget *parent) : QOpenGLWidget(parent) {}

void openGLImageWidget::setOpenCVImage(const cv::Mat &mat) {
    QMutexLocker locker(&mutex_);
    image_ = matToQImage(mat);

    if (!image_.isNull()) {
        // 获取窗口和图片的尺寸
        QSize widgetSize = size();
        QSize imageSize = image_.size();

        // 计算按比例适配窗口的缩放因子
        double scaleX = static_cast<double>(widgetSize.width()) / imageSize.width();
        double scaleY = static_cast<double>(widgetSize.height()) / imageSize.height();
        scaleFactor_ = std::min(scaleX, scaleY);  // 保持原始比例铺满窗口

        // 居中显示
        double displayWidth = imageSize.width() * scaleFactor_;
        double displayHeight = imageSize.height() * scaleFactor_;
        offset_.setX((widgetSize.width() - displayWidth) / 2.0);
        offset_.setY((widgetSize.height() - displayHeight) / 2.0);
    }

    update();  // 触发重绘
}

QImage openGLImageWidget::matToQImage(const cv::Mat &mat) {
    if (mat.empty()) return QImage();

    switch (mat.type()) {
        case CV_8UC1: {
            QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
            return img.copy();
        }
        case CV_8UC3: {
            QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
            return img.rgbSwapped().copy();  // OpenCV 是 BGR
        }
        case CV_8UC4: {
            QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
            return img.copy();
        }
        default:
            return QImage();
    }
}

void openGLImageWidget::wheelEvent(QWheelEvent *event) {
    constexpr double zoomFactor = 1.15;

    QPointF mousePos = event->position();  // 鼠标在 widget 上的位置

    // 鼠标在图像坐标系下的位置（未缩放前）
    QPointF imagePos = (mousePos - offset_) / scaleFactor_;

    if (event->angleDelta().y() > 0)
        scaleFactor_ *= zoomFactor;
    else
        scaleFactor_ /= zoomFactor;

    // 限制缩放比例
    scaleFactor_ = clamp(scaleFactor_, 0.1, 10.0);

    // 根据缩放后的图像坐标，调整 offset_ 使鼠标仍指向原图像点
    offset_ = mousePos - imagePos * scaleFactor_;

    hasUserTransform_ = true;  // 标记用户手动交互
    update();
}

void openGLImageWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        lastMousePos_ = event->pos();
        isPanning_ = true;
    }
}

void openGLImageWidget::mouseMoveEvent(QMouseEvent *event) {
    if (!isPanning_) return;
    QPointF delta = event->pos() - lastMousePos_;
    offset_ += delta;
    lastMousePos_ = event->pos();
    update();
}

void openGLImageWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        isPanning_ = false;
    }
}

void openGLImageWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    QMutexLocker locker(&mutex_);
    if (image_.isNull()) return;

    // 1. 计算缩放后图像大小
    QSizeF scaledSize = image_.size() * scaleFactor_;

    // 2. 用 offset_ 作为左上角点
    QPointF topLeft = offset_;

    // 3. 构造目标矩形并绘制
    if (backgroundColor == 0) {
        painter.fillRect(rect(), Qt::black);  // 保持黑色背景
    } else if (backgroundColor == 1) {
        painter.fillRect(rect(), Qt::white);  // 保持白色背景
    }

    QRectF targetRect(topLeft, scaledSize);
    painter.drawImage(targetRect, image_);
}

void openGLImageWidget::resizeEvent(QResizeEvent *event) {
    if (firstLoad_ == false) return;  // 用户已手动操作，跳过

    if (!image_.isNull()) {
        QSize imageSize = image_.size();
        QSize widgetSize = event->size();

        double scaleX = static_cast<double>(widgetSize.width()) / imageSize.width();
        double scaleY = static_cast<double>(widgetSize.height()) / imageSize.height();
        scaleFactor_ = std::min(scaleX, scaleY);

        QPointF imageCenter(imageSize.width() * scaleFactor_ / 2.0, imageSize.height() * scaleFactor_ / 2.0);
        QPointF widgetCenter(widgetSize.width() / 2.0, widgetSize.height() / 2.0);
        offset_ = widgetCenter - imageCenter;
    }

    QOpenGLWidget::resizeEvent(event);
}

void openGLImageWidget::contextMenuEvent(QContextMenuEvent *event) {
    QMenu menu(this);
    QAction *resetAction = menu.addAction(u8"重置画面");
    connect(resetAction, &QAction::triggered, this, &openGLImageWidget::resetView);
    menu.exec(event->globalPos());
}

void openGLImageWidget::resetView() {
    QMutexLocker locker(&mutex_);
    if (image_.isNull()) return;

    QSize widgetSize = size();
    QSize imageSize = image_.size();
    double scaleX = double(widgetSize.width()) / imageSize.width();
    double scaleY = double(widgetSize.height()) / imageSize.height();
    scaleFactor_ = std::min(scaleX, scaleY);

    double w = imageSize.width() * scaleFactor_;
    double h = imageSize.height() * scaleFactor_;
    offset_.setX((widgetSize.width() - w) / 2.0);
    offset_.setY((widgetSize.height() - h) / 2.0);

    update();
}
