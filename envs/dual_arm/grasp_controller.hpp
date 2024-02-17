#pragma once

#include <xsim/ompl_planner.hpp>
#include "robot.hpp"
#include "controller.hpp"
#include "follow_trajectory_controller.hpp"

class World ;

class GraspController: public Controller {
public:
    GraspController(World *world, xsim::JointTrajectory &traj1, Robot &robot1,
                    xsim::JointTrajectory &traj2, Robot &robot2,
                    float speed) ;

    bool step(float dt) override ;

private:

    FollowTrajectoryController cntrl1_, cntrl2_ ;
    World *world_ ;

};
