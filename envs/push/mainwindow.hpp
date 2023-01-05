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


    void setGui(QWidget *gui);
private:

    static MainWindow *instance_ ;

};

#endif // MAINWINDOW_HPP
