#include "grasp_controller.hpp"
#include "world.hpp"

#include <iostream>

using namespace std ;
using namespace xsim ;

GraspController::GraspController(World *world,
                                 xsim::JointTrajectory &traj1, Robot &r1,
                                 xsim::JointTrajectory &traj2, Robot &r2,
                                 float speed):
    world_(world),
    cntrl1_(world, traj1, r1, speed),
    cntrl2_(world, traj2, r2, speed)
 {

}

bool GraspController::step(float dt)
{
    world_->stepSimulation(dt);

    bool robot1_stopped = cntrl1_.doStep()  ;
    bool robot2_stopped = cntrl2_.doStep() ;

    return robot1_stopped && robot2_stopped ;

}




