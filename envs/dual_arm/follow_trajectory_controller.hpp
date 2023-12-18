#pragma once

#include <xsim/ompl_planner.hpp>
#include "robot.hpp"
#include "controller.hpp"

class World ;

class FollowTrajectoryController: public Controller {
public:
    FollowTrajectoryController(World *world, xsim::JointTrajectory &traj, Robot &robot, float speed) ;

    bool step(float dt) override ;

private:
    World *world_ ;
    uint cp_ = 0 ;
    Robot &robot_ ;
    xsim::JointTrajectory traj_ ;
    float speed_ ;
    float delta_ = 0.01f;
    bool segment_has_finished_ = true ;
};
