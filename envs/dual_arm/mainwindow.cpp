#include "mainwindow.hpp"

#include <QSplitter>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QDebug>

MainWindow *MainWindow::instance_ = nullptr ;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow{parent}
{
    instance_ = this ;
    setWindowTitle("dual_arm");


}

MainWindow::~MainWindow()
{

}

void MainWindow::setGui(QWidget *gui)
{
   setCentralWidget(gui);
}


