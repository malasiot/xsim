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

class ExecuteTrajectoryThread: public QThread {

    Q_OBJECT
public:
    ExecuteTrajectoryThread(Robot *robot, const xsim::JointTrajectory &traj, float speed): robot_(robot), traj_(traj), speed_(speed) {

    }

    ~ExecuteTrajectoryThread() {

    }

    void update() {
        emit updateScene();
    }
    void run() override {
        robot_->executeTrajectory(traj_, speed_);
#if 0
        for( int i=0 ; i<1000 ; i++ ) {
            float t = i/1000.0 ;
            auto j = traj_.getState(t, world_->controller_->iplan_) ;
            world_->controller_->setJointState(j) ;
            world_->stepSimulation(0.05) ;
        }
#endif
    }

signals:
    void updateScene();
protected:

    Robot *robot_ ;
    xsim::JointTrajectory traj_ ;
    float speed_ ;


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
};



#endif // GUI_HPP
