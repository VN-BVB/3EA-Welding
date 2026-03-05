#ifndef WELDINGMAINWINDOW_H
#define WELDINGMAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class WeldingMainWindow;
}
QT_END_NAMESPACE

class WeldingMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    WeldingMainWindow(QWidget *parent = nullptr);
    ~WeldingMainWindow();

private:
    Ui::WeldingMainWindow *ui;
};
#endif // WELDINGMAINWINDOW_H
