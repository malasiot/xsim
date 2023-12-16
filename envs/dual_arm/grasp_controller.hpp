#pragma once

#include <xsim/ompl_planner.hpp>
#include "robot.hpp"
#include "controller.hpp"

class World ;

class GraspController: public Controller {
public:
    GraspController(World *world, xsim::JointTrajectory &traj1, Robot &robot1,
                    xsim::JointTrajectory &traj2, Robot &robot2,
                    float speed) ;

    bool step(float dt) override ;

private:

    bool stateHasChanged(const xsim::JointState &j1, const xsim::JointState &j2) ;
    World *world_ ;
    uint cp1_ = 0, cp2_ = 0 ;
    Robot &robot1_, &robot2_ ;
    xsim::JointTrajectory traj1_, traj2_ ;
    xsim::JointState prev1_, prev2_ ;
    float speed_ ;
    float delta_ = 1.0e-10f;
    bool segment1_has_finished_ = true, segment2_has_finished_ = true ;
};
