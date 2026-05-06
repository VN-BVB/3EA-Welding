#include "QImageWidget.h"

// todo:获取窗口中的区域并返回
// 坐标系转换关系理清
QImageWidget::QImageWidget(QWidget *parent) : QOpenGLWidget(parent) {
    nw = nh = niw = nih = 0;
    preNw = preNh = 0;
    dScaleTmp = dScaleW = dScaleH = adjustX = adjustY = 0.0;
    colorTable.resize(256);

    for (int i = 0; i < 256; i++) colorTable[i] = qRgb(i, i, i);

    olderMoveX = moveX = 0;
    olderMoveY = moveY = 0;
    mousePressPoint = QPoint(0, 0);
    mouseReleasePoint = QPoint(0, 0);
    mouseMovePoint = QPoint(0, 0);
    scale = 1;
    preScale = 1;
    painter = new QPainter(this);

    penSize = 4;
    bMoveFlag = false;

    //    m_penCursor=QPixmap("pen.png");
    //    m_xpcCursor=QPixmap("xpc.png");

    //    m_penQCursor=QCursor(m_penCursor,0,m_penCursor.height());
    //    m_xpcQCursor=QCursor(m_xpcCursor,0,m_xpcCursor.height());

    isInit = false;
}

QImageWidget::~QImageWidget() {}

void QImageWidget::clear() {
#ifdef USE_OPENCV
//    Mat black =
//    Mat::zeros(m_DisplayImage.width(),m_DisplayImage.height(),CV_8UC3);
//    setOpenCVImage(black);
#endif
}

void QImageWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter_tmp(this);
    drawImage(&painter_tmp);
}

void QImageWidget::setImage(QImage &image) {
    QMutexLocker lock(&mutex);
    if (image.isNull()) {
        PLOGD << "Image is null";
        return;
    }
    if (displayImage.isNull() || displayImage.size() != image.size() || displayImage.format() != image.format()) {
        originalImage = displayImage = image;
        //        return;
    }
    int bpp = image.depth() / 8;
    setImage(image.bits(), image.width() * image.height() * bpp, image.width(), image.height());
}

void QImageWidget::setImage(const uchar *data, int data_length, int width, int height) {
    // 得到图像的位深
    int Dst_bpp = displayImage.depth() / 8;
    int Src_bpp = data_length / (width * height);

    // 如果当前图像的大小和位深和要显示的图像一样则直接内存拷贝就行
    if (Dst_bpp == Src_bpp && displayImage.width() == width && displayImage.height() == height) {
        memcpy(displayImage.bits(), data, data_length);
    } else {
        QImage::Format fmt;
        // 否则，就要确定图像的类型
        switch (Src_bpp) {
            case 1:
                fmt = QImage::Format_Indexed8;
                break;
            case 3:
                fmt = QImage::Format_RGB888;
                break;
            case 4:
                fmt = QImage::Format_ARGB32;
                break;
            default:
                return;
        }

        // 重新为图像赋值
        displayImage = QImage(data, width, height, fmt);
        if (fmt == QImage::Format_Indexed8) {
            displayImage.setColorTable(colorTable);
        }
    }
    update();
    isInit = false;
}

#ifdef USE_OPENCV
void QImageWidget::setOpenCVImage(cv::Mat imgo) {
    cv::Mat img;
    cv::cvtColor(imgo, img, cv::COLOR_RGB2BGR);
    QMutexLocker lock(&mutex);
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (img.type() == CV_8UC1) {
        if (displayImage.width() == img.cols && displayImage.height() == img.rows && displayImage.format() == QImage::Format_Indexed8) {
            uchar *pSrc = img.data;
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            const uchar *pSrc = static_cast<const uchar *>(img.data);
            displayImage = QImage(pSrc, img.cols, img.rows, QImage::Format_Indexed8);
            displayImage.setColorTable(colorTable);
        }
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (img.type() == CV_8UC3) {
        // 如果尺寸和格式都没有改变，则直接进行内存拷贝
        if (displayImage.width() == img.cols && displayImage.height() == img.rows && displayImage.format() == QImage::Format_RGB888) {
            const uchar *pSrc = static_cast<const uchar *>(img.data);
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            const uchar *pSrc = static_cast<const uchar *>(img.data);
            displayImage = QImage(pSrc, img.cols, img.rows, static_cast<int>(img.step), QImage::Format_RGB888);
            displayImage = displayImage.rgbSwapped();
        }
    } else if (img.type() == CV_8UC4) {
        if (displayImage.width() == img.cols && displayImage.height() == img.rows && displayImage.format() == QImage::Format_ARGB32) {
            uchar *pSrc = img.data;
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            displayImage = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_ARGB32);
        }
    } else {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
    }
    update();
    isInit = false;
}

void QImageWidget::setOpenCVImage2(cv::Mat img) {
    QMutexLocker lock(&mutex);
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (img.type() == CV_8UC1) {
        if (displayImage2.width() == img.cols && displayImage2.height() == img.rows && displayImage2.format() == QImage::Format_Indexed8) {
            uchar *pSrc = img.data;
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage2.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            displayImage2 = QImage(img.data, img.cols, img.rows, QImage::Format_Indexed8);
            displayImage2.setColorTable(colorTable);
        }
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (img.type() == CV_8UC3) {
        // 如果尺寸和格式都没有改变，则直接进行内存拷贝
        if (displayImage2.width() == img.cols && displayImage2.height() == img.rows && displayImage2.format() == QImage::Format_RGB888) {
            const uchar *pSrc = static_cast<const uchar *>(img.data);
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage2.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            const uchar *pSrc = static_cast<const uchar *>(img.data);
            displayImage2 = QImage(pSrc, img.cols, img.rows, static_cast<int>(img.step), QImage::Format_RGB888);
            displayImage2 = displayImage2.rgbSwapped();
        }
    } else if (img.type() == CV_8UC4) {
        if (displayImage2.width() == img.cols && displayImage2.height() == img.rows && displayImage2.format() == QImage::Format_ARGB32) {
            uchar *pSrc = img.data;
            for (int row = 0; row < img.rows; row++) {
                uchar *pDest = displayImage2.scanLine(row);
                memcpy(pDest, pSrc, static_cast<size_t>(img.step));
                pSrc += img.step;
            }
        } else {
            displayImage2 = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_ARGB32);
        }
    } else {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
    }
    update();
    isInit = false;
}

void QImageWidget::setMode(int mode) {
    mMode = mode;
    update();
}
#endif

void QImageWidget::mousePressEvent(QMouseEvent *event) {
    mousePressPoint = event->pos();
    mouseReleasePoint = QPoint(-1, -1);
    if (event->button() == Qt::LeftButton) {
        bMoveFlag = true;
    }
    update();
    emit pic_moved();
}

void QImageWidget::mouseMoveEvent(QMouseEvent *event) {
    mouseMovePoint = event->pos();

    if (bMoveFlag) {
        moveX = mouseMovePoint.x() - mousePressPoint.x();
        moveY = mouseMovePoint.y() - mousePressPoint.y();
    }
    update();
    emit pic_moved();
}

void QImageWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        mouseReleasePoint = event->pos();
        moveX = mouseReleasePoint.x() - mousePressPoint.x();
        moveY = mouseReleasePoint.y() - mousePressPoint.y();
        olderMoveX += moveX;
        olderMoveY += moveY;
        moveX = 0;
        moveY = 0;
        bMoveFlag = false;
    }
    update();
    emit pic_moved();
}

void QImageWidget::wheelEvent(QWheelEvent *event) {
    mouseWheelPoint = event->pos();
    QPoint numDegrees = event->angleDelta() / 8;  // 得到滚轮转动的数值

    if (numDegrees.y() > 0) {
        preScale = scale;
        scale += (scale / 5.0);
        olderMoveX = mouseWheelPoint.x() - (mouseWheelPoint.x() - olderMoveX) * scale / preScale;
        olderMoveY = mouseWheelPoint.y() - (mouseWheelPoint.y() - olderMoveY) * scale / preScale;
    } else {
        preScale = scale;
        scale -= (scale / 5.0);  // * (numDegrees.y()/5.0)
        if (scale < 0.2) {
            scale = 0.2;
        }
        olderMoveX = mouseWheelPoint.x() - (mouseWheelPoint.x() - olderMoveX) * scale / preScale;
        olderMoveY = mouseWheelPoint.y() - (mouseWheelPoint.y() - olderMoveY) * scale / preScale;
    }
    update();
    emit pic_moved();
}

bool QImageWidget::validatePosInImage(QPoint pos) {
    if (displayImage.isNull()) {
        return false;
    }
    QPoint mapPos = matrix.inverted().map(pos);  // 将点击位置转换到图像上
    double iw = displayImage.width();            // 获取图片大小
    double ih = displayImage.height();
    QRect imgRect(0, 0, iw, ih);

    return imgRect.contains(mapPos);  // 判断是否在图像内
}

void QImageWidget::drawImage(QPainter *painter) {  // 绘制图片

    QPixmap buff(this->size());  // 设置画布大小

    QPainter p(&buff);  // 设置画笔
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    if (!displayImage.isNull()) {  // 记录图片的缩放比例
        nw = this->width();        // 得到控件大小
        nh = this->height();

        if (nw != preNw || nh != preNh) {  // 如果控件大小改变了，调整缩放和平移
            if (!isInit) {
                isInit = true;
            } else {
                preScale = scale;  // 缩放
                float scale_1 = nw / (float)preNw;
                // float scale_2 = m_nh/(float)m_pre_nh;
                scale *= scale_1;

                olderMoveX = nw / 2 - (preNw / 2 - olderMoveX) * scale / preScale;  // 平移
                olderMoveY = nh / 2 - (preNh / 2 - olderMoveY) * scale / preScale;
            }
        }
        preNw = nw;
        preNh = nh;
        niw = displayImage.width();  // 得到图片大小
        nih = displayImage.height();
        dScaleW = (double)nw / (double)niw;  // 计算控件和图片的比例
        dScaleH = (double)nh / (double)nih;
        dScaleW = dScaleH = qMin(dScaleW, dScaleH);  // 取小比例，保证图片完全显示在控件内

        dScaleTmp = scale * dScaleW;  // 放大或者缩小图片
        niw *= dScaleW;
        nih *= dScaleW;
        adjustX = -(niw - nw) / 2 + olderMoveX + moveX;  // 调整图片在控件出现的位置
        adjustY = -(nih - nh) / 2 + olderMoveY + moveY;
        matrix = QMatrix().translate(adjustX, adjustY).scale(dScaleTmp, dScaleTmp);  // 计算画笔的绘制的转换矩阵
        p.setWorldTransform(QTransform(matrix));
        if (mMode == 0) {
            p.drawPixmap(0, 0, QPixmap::fromImage(displayImage));
        } else {
            p.drawPixmap(0, 0, QPixmap::fromImage(displayImage2));
        }
    }

    painter->drawPixmap(0, 0, buff);  // 绘制图片
}

// 获得当前显示的图片
QImage QImageWidget::getDisplayImage() {
    QMutexLocker lock(&mutex);
    return displayImage;
}

// 设置画笔大小
void QImageWidget::setPenSize(int size) { penSize = size; }

void QImageWidget::initSize() {
    olderMoveX = moveX = 0;
    olderMoveY = moveY = 0;
    scale = 1;
    update();
}

QImage QImageWidget::getCurrentArea() {
    QMutexLocker lock(&mutex);
    int count = displayImage.width() / scale;
    if (count > 200) count = 200;
    int x = (-adjustX + nw / 2) / (niw * scale) * displayImage.width() - count;
    int y = (-adjustY + nh / 2) / (nih * scale) * displayImage.height() - count;
    QImage ret = displayImage.copy(x, y, count * 2, count * 2);
    return ret.mirrored();
}
