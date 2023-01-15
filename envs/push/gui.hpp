#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "environment.hpp"

#include <xsim/world.hpp>
#include <xsim/collision.hpp>
#include <xsim/ompl_planner.hpp>
#include <xviz/scene/node.hpp>
#include <xviz/gui/manipulator.hpp>
#include <xsim/kinematic.hpp>

#include <cvx/math/rng.hpp>
#include <cvx/misc/format.hpp>

#include <QTimer>
#include <QThread>

#include <iostream>

class ExecuteEnvironmentThread: public QThread {

    Q_OBJECT
public:
    ExecuteEnvironmentThread(World *world): env_(world), world_(world) {
        world_->setUpdateCallback([this]() {
            emit updateScene();
        });

        world_->controller()->setStartTrajectoryCallback([this](const xsim::JointTrajectory &t) {
            emit showTrajectory(t);
        });
    }

    ~ExecuteEnvironmentThread() {
        world_->setUpdateCallback(nullptr);
        world_->controller()->setStartTrajectoryCallback(nullptr) ;
    }


    void run() override {
        std::vector<std::string> boxes = env_.getBoxNames() ;

        for( int  i = 0 ; i<100 ; i++ ) {
            State state = env_.getState() ;
            PushAction a ;
            a.box_id_ = rng_.choice(boxes) ; ;
            a.loc_ = rng_.uniform(0, 11) ;
            cv::Mat im = env_.renderState(a, state) ;
            cv::imwrite("/tmp/state.png", im) ;
            cv::imwrite(cvx::format("/tmp/state_{:03d}.png", i), im) ;
            auto res = env_.transition(state, a) ;
         //   emit showTrajectory(env_.lastTrajectory()) ;
            const State &new_state =  res.first ;
            if ( new_state.isTerminal() ) break ;
        }
    }

signals:
    void updateScene();
    void showTrajectory(const xsim::JointTrajectory &) ;
protected:

    World *world_ ;
    Environment env_ ;
    cvx::RNG rng_ ;


};

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

class GUI: public SimulationGui, xsim::CollisionFeedback {
    Q_OBJECT
public:
    GUI(World *world);

    void execute(const xsim::JointTrajectory &traj) ;

    void onUpdate(float delta) override;

    void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent * event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void processContact(xsim::ContactResult &r) override;

private:
    std::shared_ptr<xviz::TransformManipulator> gizmo_;
    xviz::NodePtr target_, traj_node_, traj_points_[100] ;
    xviz::NodePtr urdf_ ;

    QTimer timer_ ;

    World *world_ ;
    ExecuteTrajectoryThread *traj_thread_ = nullptr ;

    void trajectory(const xsim::JointTrajectory &traj);
 public slots:
    void showTrajectory(const xsim::JointTrajectory &traj);
protected:
    void runenv();
};


#endif // GUI_HPP
