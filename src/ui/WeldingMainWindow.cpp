#include "WeldingMainwindow.h"

#include "ui_WeldingMainwindow.h"

WeldingMainWindow::WeldingMainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::WeldingMainWindow) { ui->setupUi(this); }

WeldingMainWindow::~WeldingMainWindow() { delete ui; }
