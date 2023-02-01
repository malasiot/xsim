#ifndef GUI_HPP
#define GUI_HPP

#include "bullet_gui.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "player.hpp"

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

class ExecuteStepThread: public QThread {

    Q_OBJECT
public:
    ExecuteStepThread(Player *env, int64_t action): env_(env), action_id_(action) {
        env_->world()->setUpdateCallback([this]() {
            emit updateScene();
        });
    }

    ~ExecuteStepThread() {
        env_->world()->setUpdateCallback(nullptr);
    }

    void run() override {
        auto state = env_->getState() ;
        auto [new_state,  done] = v_->step(state, action_id_) ;
        emit stepFinished(new_state, done) ;
    }

signals:
    void updateScene();
    void stepFinished(const State &state, bool done) ;
protected:

    Player *env_ ;
    int64_t action_id_ ;

};
/*
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

*/

class GUI: public SimulationGui {
    Q_OBJECT
public:
    GUI(xsim::PhysicsWorld *e);

    void setTarget(const std::string &box, const Eigen::Vector2f &pos, float radius) ;

    void onUpdate(float delta) override;

private:

};


#endif // GUI_HPP
