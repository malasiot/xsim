#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "environment.hpp"

#undef slots
#include "trainer.hpp"
#define slots Q_SLOTS

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
    ExecuteEnvironmentThread(Environment *env): env_(env) {
        env_->world()->setUpdateCallback([this]() {
            emit updateScene();
        });
    }

    ~ExecuteEnvironmentThread() {
        env_->world()->setUpdateCallback(nullptr);
    }

    void run() override {
        using namespace std ;
        std::vector<std::string> boxes = env_->getBoxNames() ;

        int trial = 0 ;
        while (1) {
            State state = env_->getState() ;

#if 0

            PushAction a ;
            a.box_id_ = rng_.choice(boxes) ; ;
            a.loc_ = rng_.uniform(0, 11) ;
            cv::Mat im = env_->renderState(a, state) ;
            cv::imwrite(cvx::format("/tmp/state_{:03d}.png", trial++), im) ;
            auto res = env_->transition(state, a) ;

            cout << res.first << ' ' << res.second << endl ;

            if ( res.first.isTerminal() ) {
                env_->reset() ;
                trial = 0 ;
            }
#endif
        }
    }

signals:
    void updateScene();
    void showTrajectory(const xsim::JointTrajectory &) ;
protected:

    Environment *env_ ;
    cvx::RNG rng_ ;


};

class TrainingThread: public QThread {

    Q_OBJECT
public:
    TrainingThread(Trainer *trainer): trainer_(trainer) {
        trainer_->agent()->env()->world()->setUpdateCallback([this]() {
            emit updateScene();
        });
    }

    ~TrainingThread() {
        trainer_->agent()->env()->world()->setUpdateCallback(nullptr);
    }

    void run() override {
        try {
        trainer_->train(100000);
        } catch (c10::Error &e) {
            std::cout << e.what() << std::endl ;
        }
    }

signals:
    void updateScene();

protected:

    Trainer *trainer_ ;

};



class GUI: public SimulationGui {
    Q_OBJECT
public:
    GUI(xsim::PhysicsWorld *e);

    void onUpdate(float delta) override;

private:

};


#endif // GUI_HPP
