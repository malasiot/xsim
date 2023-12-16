#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "world.hpp"
#include "player.hpp"

#include <QTimer>
#include <QThread>
#include <QTcpSocket>

#include <iostream>

#include <xviz/gui/manipulator.hpp>

#include <QThread>

#include "controller.hpp"

class ExecuteTrajectoryThread: public QThread {

    Q_OBJECT
public:
    ExecuteTrajectoryThread(Controller *c, float dt): controller_(c), dt_(dt) {

    }

    ~ExecuteTrajectoryThread() {

    }

    void update() {
        emit updateScene();
    }
    void run() override {
        while ( !controller_->step(dt_) ) {
            update() ;
        }
    }

signals:
    void updateScene();
protected:

    Controller *controller_ ;
    float dt_ ;
};

class GUI: public SimulationGui {
    Q_OBJECT
public:
    GUI(World *p);

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;
    void setGrabFramePath(const QString &path) {
        grab_frame_path_ = path ;
    }

    void onUpdate(float delta) override;


   void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void trajectory(const xsim::JointTrajectory &traj);

private slots:
    void grabScreen();
    void startRecording() ;
    void stopRecording() ;
signals:
    void recordingStarted() ;
    void recoringStopped() ;
private:


    xviz::NodePtr target1_, target2_ ;
    World *world_ ;
    std::shared_ptr<xviz::TransformManipulator> gizmo1_, gizmo2_;

    QTimer *timer_ ;
    int64_t count_ ;
    QString grab_frame_path_, title_ ;
    ExecuteTrajectoryThread *thread_ ;
};



#endif // GUI_HPP
