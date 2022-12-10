#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <xsim/kinematic.hpp>

class QSplitter;
class QSlider ;
class GUI ;
class QVBoxLayout ;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() ;


    static MainWindow *instance() {
        return instance_ ;
    }

    void setGui(QWidget *gui) ;
    void addSlider(const std::string &name, float lower, float upper) ;
    void endSliders() ;
public slots:
    void updateControls(const xsim::JointState &) ;
signals:
    void controlValueChanged(const std::string &name, float v) ;

private:
    QSplitter *splitter_ ;
    QWidget *controls_ ;
    QWidget *gui_ ;

    struct SliderData {
        float lower_, upper_ ;
        std::string name_ ;
    };

    static MainWindow *instance_ ;

    std::map<QSlider *, SliderData> slider_to_data_ ;
    std::map<std::string, QSlider *> name_to_slider_ ;
    QVBoxLayout *control_layout_ ;

};

#endif // MAINWINDOW_HPP
