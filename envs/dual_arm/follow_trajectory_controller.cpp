#include "follow_trajectory_controller.hpp"
#include "world.hpp"

#include <iostream>

using namespace std ;
using namespace xsim ;

FollowTrajectoryController::FollowTrajectoryController(World *world, xsim::JointTrajectory &traj, Robot &r, float speed):
    world_(world), traj_(traj), robot_(r), speed_(speed) {

    prev_ = robot_.getJointState() ;
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

bool FollowTrajectoryController::doStep()
{
    if ( hasCollision() ) {
        robot_.stop() ;
        has_stopped_ = true ;
        return true ;
    }

    JointState t1, t2 ;

    if ( !has_stopped_ ) {
        t1 = traj_.points()[cp_] ;
        t2 = traj_.points()[cp_ + 1] ;
    }

    if ( segment_has_finished_ && cp_ < traj_.points().size()-1 ) {
        robot_.move(t1, t2, speed_) ;
        segment_has_finished_ = false ;
    }

    auto state = robot_.getJointState() ;

    if ( !stateHasChanged(state, t2, 0.001) ) {
        segment_has_finished_ = true ;
        cp_ ++ ;
    }

    if ( cp_ == traj_.points().size() - 1 ) {
        robot_.stop() ;
        has_stopped_ = true ;
    }

    prev_ = state ;

    return ( has_stopped_ ) ;

}

bool FollowTrajectoryController::hasCollision()
{
    auto contacts = world_->getAllContacts() ;

    for( const auto &c: contacts ) {
        if ( ( (c.a_->getName() == robot_.prefix() + "link_6" ) && ( c.b_->getName() == "box_0_0") )  ||
            ( (c.b_->getName() == robot_.prefix() + "link_6" ) && ( c.a_->getName() == "box_0_0") ) ) return true ;
    }

    return false ;
}

bool FollowTrajectoryController::stateHasChanged(const xsim::JointState &j1, const xsim::JointState &j2, float delta)
{

    bool changed = false ;
    for( const auto &sp: j1 ) {
        float v2 = j2.find(sp.first)->second;
        float v1 = sp.second ;

        if ( fabs(v1 - v2) > delta ) {
            return true ;
        }
    }
    return false ;
}

