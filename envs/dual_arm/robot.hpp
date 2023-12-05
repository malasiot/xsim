#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>
#include <map>
#include <xsim/multi_body.hpp>

#include <xsim/ompl_planner.hpp>
#include <xsim/world.hpp>

// class to control robot movement and state

using StartTrajectoryCallback = std::function<void(const xsim::JointTrajectory &)> ;

struct Robot {

    Robot(xsim::PhysicsWorld *world, const xsim::MultiBodyPtr &c): world_(world), controller_(c) {}

    void executeTrajectory(const xsim::JointTrajectory &t, float speed) ;

    void move(const xsim::JointState &start_state, const xsim::JointState &end_state, float speed) ;
    void moveTo(const xsim::JointState &target, float speed) ;

    void setJointState(const std::string &name, float v) ;
    void setJointState(const xsim::JointState &state) ;
    float getJointState(const std::string &name) ;
    xsim::JointState getJointState() const ;
    void stop() ;

    void setStartTrajectoryCallback(StartTrajectoryCallback cb) { stcb_ = cb ; }

    const std::vector<std::string> &armJointNames() const ;

private:

    xsim::MultiBodyPtr controller_ ;
    xsim::PhysicsWorld *world_ ;
    StartTrajectoryCallback stcb_ = nullptr ;
};

#endif // ROBOT_HPP
