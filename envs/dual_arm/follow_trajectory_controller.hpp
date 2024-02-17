#pragma once

#include <xsim/ompl_planner.hpp>
#include "robot.hpp"
#include "controller.hpp"

class World ;

class FollowTrajectoryController: public Controller {
public:
    FollowTrajectoryController(World *world, xsim::JointTrajectory &traj, Robot &robot, float speed) ;

    bool step(float dt) override ;

    bool doStep() ;

    bool hasCollision() ;

private:

    bool stateHasChanged(const xsim::JointState &j1, const xsim::JointState &j2, float delta) ;

    World *world_ ;
    uint cp_ = 0 ;
    Robot &robot_ ;
    xsim::JointTrajectory traj_ ;
    xsim::JointState prev_ ;
    bool has_stopped_ = false ;
    float speed_ ;
    float delta_ = 0.01f;
    bool segment_has_finished_ = true ;
};
