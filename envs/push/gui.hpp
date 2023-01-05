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

#include <cvx/math/rng.hpp>
#include <cvx/misc/format.hpp>

#include <QTimer>
#include <QThread>

class ExecuteEnvironmentThread: public QThread {

    Q_OBJECT
public:
    ExecuteEnvironmentThread(World *world): env_(world), world_(world) {
        world_->setUpdateCallback([this]() {
            emit updateScene();
        });
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
            if ( !env_.apply(state, a) ) continue ;
        }
    }

signals:
    void updateScene();
protected:

    World *world_ ;
    Environment env_ ;
    cvx::RNG rng_ ;


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
    xviz::NodePtr target_ ;
    std::shared_ptr<Robot> robot_ ;
    QTimer timer_ ;
    JointState start_state_ ;

protected:
    void runenv();
};


#endif // GUI_HPP
