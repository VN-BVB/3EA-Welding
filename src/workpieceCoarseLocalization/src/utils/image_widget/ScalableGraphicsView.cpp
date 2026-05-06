#include "ScalableGraphicsView.h"

#include <QDebug>

// ScalableGraphicsView::ScalableGraphicsView(QGraphicsScene* scene, QWidget* parent)
//     // : QGraphicsView(scene, parent)
// {
//     setRenderHint(QPainter::Antialiasing);
//     setRenderHint(QPainter::TextAntialiasing);
// }

ScalableGraphicsView::ScalableGraphicsView(QWidget *parent) : QGraphicsView(parent) {
    QGraphicsScene *scene = new QGraphicsScene(this);  // 创建场景
    setScene(scene);                                   // 设置场景
    setRenderHint(QPainter::Antialiasing);
    setRenderHint(QPainter::TextAntialiasing);

    scale(0.15, 0.15);  // 缩小
}

void ScalableGraphicsView::wheelEvent(QWheelEvent *event) {
    // 获取鼠标相对于视图的位置
    QPointF mousePosBefore = mapToScene(event->pos());

    // 根据滚轮方向缩放
    if (event->angleDelta().y() > 0) {
        scale(1.1, 1.1);  // 放大
    } else {
        scale(0.9, 0.9);  // 缩小
    }

    // 获取缩放后的视图位置
    QPointF mousePosAfter = mapToScene(event->pos());

    // 计算鼠标位置的偏移量
    QPointF delta = mousePosAfter - mousePosBefore;

    // 调整视图的滚动位置，使得鼠标所在位置保持不变
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());

    event->accept();  // 接受事件，防止其他处理
}

void ScalableGraphicsView::setOriginalImageInfo(int width, int height, int angle) {  // 列、行
    originalImageWidth = width;
    originalImageHeight = height;
    rotationAngle = angle % 360;  // 保证在 0~359 范围内
}

void ScalableGraphicsView::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        // 记录鼠标按下的位置
        lastMousePos = event->pos();
        isDragging = true;
    }
    // 获取点击位置的场景坐标
    QPointF scenePos = mapToScene(event->pos());  // 局部变为全局

    // 获取对应的像素坐标
    cv::Point2i rotatedXY(static_cast<int>(scenePos.x()), static_cast<int>(scenePos.y()));

    cv::Point2i originalXY = CoordinateMapper::rotateToOriginal(rotatedXY, originalImageWidth, originalImageHeight, rotationAngle);

    emit senderSignalPixelCoordinates(originalXY.x, originalXY.y);

    // 如果需要，调用父类的事件处理
    QGraphicsView::mousePressEvent(event);
}

// 3. 处理鼠标移动事件（拖拽图像）
void ScalableGraphicsView::mouseMoveEvent(QMouseEvent *event) {
    if (isDragging) {
        // 计算鼠标移动的距离
        QPoint delta = event->pos() - lastMousePos;

        // 更新视图的位置
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
        verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());

        // 更新上一次的鼠标位置
        lastMousePos = event->pos();
    }

    QGraphicsView::mouseMoveEvent(event);  // 调用父类的事件处理
}

// 4. 处理鼠标释放事件（停止拖拽）
void ScalableGraphicsView::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        isDragging = false;  // 停止拖拽
    }

    QGraphicsView::mouseReleaseEvent(event);  // 调用父类的事件处理
}
