#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "world.hpp"
#include "robot.hpp"

#include <xsim/world.hpp>
#include <xsim/collision.hpp>
#include <xsim/ompl_planner.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/gui/manipulator.hpp>

#include <QThread>

class ExecuteTrajectoryThread: public QThread {

    Q_OBJECT
public:
    ExecuteTrajectoryThread(World *world, const xsim::JointTrajectory &traj): world_(world), traj_(traj) {
        world_->setUpdateCallback([this]() {
            update() ;

        });
    }

    ~ExecuteTrajectoryThread() {
        world_->setUpdateCallback(nullptr);
    }

    void update() {
        emit updateScene();
    }
    void run() override {
      world_->controller()->executeTrajectory(traj_, 0.5);
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

    World *world_ ;
    xsim::JointTrajectory traj_ ;


};

class GUI: public xviz::SceneViewer, xsim::CollisionFeedback {
    Q_OBJECT
public:
    GUI(World *world);

    void openGripper();

    void closeGripper();

    void onUpdate(float delta) override;

    void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void processContact(xsim::ContactResult &r) override;

    void trajectory(const xsim::JointTrajectory &traj);
private:
    std::shared_ptr<xviz::TransformManipulator> gizmo_;
    xviz::NodePtr target_ ;
    World *world_ ;
    xviz::NodePtr traj_node_, traj_points_[100] ;
    ExecuteTrajectoryThread *traj_thread_ = nullptr ;

    void showTrajectory(const xsim::JointTrajectory &traj);
public slots:
    void changeControlValue(const std::string &jname, float v);
    void updateControls(const xsim::JointState &state) ;

};


#endif // GUI_HPP
