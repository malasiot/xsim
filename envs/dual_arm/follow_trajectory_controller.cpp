#include "follow_trajectory_controller.hpp"
#include "world.hpp"

#include <iostream>

using namespace std ;
using namespace xsim ;

FollowTrajectoryController::FollowTrajectoryController(World *world, xsim::JointTrajectory &traj, Robot &r, float speed):
    world_(world), traj_(traj), robot_(r), speed_(speed) {

}

bool FollowTrajectoryController::step(float dt)
{
    const auto &t1 = traj_.points()[cp_] ;
    const auto &t2 = traj_.points()[cp_ + 1] ;

    if ( segment_has_finished_ && cp_ < traj_.points().size() ) {
        robot_.move(t1, t2, speed_) ;
        segment_has_finished_ = false ;
    }

    world_->stepSimulation(dt);

    auto state = robot_.getJointState() ;

    auto torques = robot_.getTorques() ;
    float max_t = *std::max_element(torques.begin(), torques.end());

    cout << max_t << endl ;

    bool changed = false ;
    for( const auto &sp: state ) {
        float start_v = t2.find(sp.first)->second;
        float v = state[sp.first] ;

        if ( fabs(v - start_v) > delta_ ) {
            changed = true ;
            break ;
        }
    }

    if ( !changed ) {
        segment_has_finished_ = true ;
        cp_ ++ ;
    }

    if ( cp_ == traj_.points().size() - 1 ) {
        robot_.stop() ;
        for( int i=0 ; i<100 ; i++ )
            world_->stepSimulation(0.05);
        return true ;
    }

    return false ;
}


