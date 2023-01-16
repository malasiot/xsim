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

    }

    ~ExecuteEnvironmentThread() {
        world_->setUpdateCallback(nullptr);
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


class GUI: public SimulationGui {
    Q_OBJECT
public:
    GUI(World *world);

    void onUpdate(float delta) override;

private:

    World *world_ ;

protected:
    void runenv();
};


#endif // GUI_HPP
