#include "mainwindow.hpp"

#include <QSplitter>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow{parent}
{
    splitter_ = new QSplitter(this) ;
    splitter_->setOrientation(Qt::Horizontal);
    controls_ = new QWidget(this) ;
    splitter_->addWidget(controls_) ;
    setCentralWidget(splitter_) ;

    control_layout_ = new QVBoxLayout() ;
    control_layout_->setContentsMargins(0,0,0,0);
    controls_->setLayout(control_layout_) ;

}

void MainWindow::setGui(QWidget *gui)
{
    gui_ = gui ;
    splitter_->addWidget(gui) ;
}

void MainWindow::addSlider(const std::string &name, float lower, float upper) {
    QVBoxLayout *l = new QVBoxLayout() ;
    l->setContentsMargins(0,0,0,0);
    l->setSpacing(0);

    QLabel *label = new QLabel(QString::fromStdString(name)) ;
    label->setMargin(0) ;
    label->setStyleSheet("QLabel{padding: 0 0 0 0px;}");

    l->addWidget(label) ;

    QSlider *slider = new QSlider(Qt::Horizontal, controls_) ;

    slider->setMinimum(0) ;
    slider->setMaximum(100) ;
    l->addWidget(slider) ;

    SliderData data ;
    data.lower_ = lower ;
    data.upper_ = upper ;
    data.name_ = name ;

    slider_to_data_.emplace(slider, data) ;

    connect(slider, &QSlider::valueChanged, this, [this, lower, upper, name] (int v) {
       float value = lower + (upper - lower) * v / 100.0 ;
       emit controlValueChanged(name, value) ;
    });
    control_layout_->addLayout(l) ;
}
