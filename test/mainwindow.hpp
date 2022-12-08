#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
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

    void setGui(QWidget *gui) ;
    void addSlider(const std::string &name, float lower, float upper) ;
    void endSliders() ;
public slots:
    void updateControls(const std::map<std::string, float> &) ;
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

    std::map<QSlider *, SliderData> slider_to_data_ ;
    std::map<std::string, QSlider *> name_to_slider_ ;
    QVBoxLayout *control_layout_ ;

};

#endif // MAINWINDOW_HPP
