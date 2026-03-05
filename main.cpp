#include <QApplication>

#include "ui/WeldingMainwindow.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    WeldingMainWindow w;
    w.show();
    return a.exec();
}
