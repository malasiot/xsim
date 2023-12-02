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
    splitter_ = new QSplitter(this) ;
    splitter_->setOrientation(Qt::Horizontal);
    controls_ = new QWidget(this) ;
    image_ = new QLabel(this) ;

    QWidget *left_panel = new QWidget(this) ;
    QVBoxLayout *vbox = new QVBoxLayout() ;

    vbox->addWidget(controls_) ;
    vbox->addWidget(image_) ;
    left_panel->setLayout(vbox);
    splitter_->addWidget(left_panel) ;
    setCentralWidget(splitter_) ;

    control_layout_ = new QVBoxLayout() ;
    control_layout_->setContentsMargins(0,0,0,0);
    controls_->setLayout(control_layout_) ;

    instance_ = this ;


}

MainWindow::~MainWindow()
{
    qDebug() << "ok" ;
}

void MainWindow::setGui(QWidget *gui)
{
    gui_ = gui ;
    splitter_->addWidget(gui) ;

}

void MainWindow::addSlider(const std::string &name, float lower, float upper) {
    QVBoxLayout *l = new QVBoxLayout() ;
    l->setContentsMargins(4,4,4,4);
    l->setSpacing(4);

    QLabel *label = new QLabel(QString::fromStdString(name)) ;

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
    name_to_slider_.emplace(name, slider) ;

    connect(slider, &QSlider::valueChanged, this, [this, lower, upper, name] (int v) {
       float value = lower + (upper - lower) * v / 100.0 ;
       emit controlValueChanged(name, value) ;
    });
    control_layout_->addLayout(l) ;
}

void MainWindow::endSliders() {
    control_layout_->addStretch();
}

void MainWindow::updateControls(const xsim::JointState &state)
{
    for( const auto &sp: state ) {
        auto it = name_to_slider_.find(sp.first) ;
        float v = sp.second ;
        if ( it != name_to_slider_.end() ) {
            QSlider *slider = it->second ;
            SliderData &data = slider_to_data_[slider] ;
            int value = 100 * (v - data.lower_)/(data.upper_ - data.lower_) ;
            slider->blockSignals(true) ;
            slider->setValue(value) ;
            slider->blockSignals(false) ;
        }
    }

}

void MainWindow::updateImage(const QImage &im)
{
image_->setPixmap(QPixmap::fromImage(im.scaledToWidth(150)));
}
