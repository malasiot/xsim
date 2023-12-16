#include "grasp_controller.hpp"
#include "world.hpp"

#include <iostream>

using namespace std ;
using namespace xsim ;

GraspController::GraspController(World *world,
                                 xsim::JointTrajectory &traj1, Robot &r1,
                                 xsim::JointTrajectory &traj2, Robot &r2,
                                 float speed):
    world_(world), traj1_(traj1), robot1_(r1),
    traj2_(traj2), robot2_(r2), speed_(speed) {

    prev1_ = robot1_.getJointState();
    prev2_ = robot2_.getJointState();
}

bool GraspController::step(float dt)
{
    const auto &t1_1 = traj1_.points()[cp1_] ;
    const auto &t1_2 = traj1_.points()[cp1_ + 1] ;

    const auto &t2_1 = traj2_.points()[cp2_] ;
    const auto &t2_2 = traj2_.points()[cp2_ + 1] ;

    if ( segment1_has_finished_ && cp1_ < traj1_.points().size()-1 ) {
        robot1_.move(t1_1, t1_2, speed_) ;
        segment1_has_finished_ = false ;
    }

    if ( segment2_has_finished_ && cp2_ < traj2_.points().size()-1 ) {
        robot2_.move(t2_1, t2_2, speed_) ;
        segment2_has_finished_ = false ;
    }

    world_->stepSimulation(dt);

    auto state1 = robot1_.getJointState() ;
    auto state2 = robot2_.getJointState() ;

    if ( !stateHasChanged(state1, prev1_) ) {
        segment1_has_finished_ = true ;
        cp1_ ++ ;
    }

    if ( !stateHasChanged(state2, prev2_) ) {
        segment2_has_finished_ = true ;
        cp2_ ++ ;
    }

    prev1_ = state1 ;
    prev2_ = state2 ;

    if ( cp1_ == traj1_.points().size() - 1 ) {
        robot1_.stop() ;
    }

    if ( cp2_ == traj2_.points().size() - 1 ) {
        robot2_.stop() ;
    }
    for( int i=0 ; i<100 ; i++ )
        world_->stepSimulation(0.05);
    return ( cp1_ == traj1_.points().size() - 1 && cp2_ == traj2_.points().size() - 1 ) ;

}

bool GraspController::stateHasChanged(const xsim::JointState &j1, const xsim::JointState &j2)
{

    bool changed = false ;
    for( const auto &sp: j1 ) {
        float v2 = j2.find(sp.first)->second;
        float v1 = sp.second ;

        if ( fabs(v1 - v2) > delta_ ) {
            return true ;
        }
    }
    return false ;
}


