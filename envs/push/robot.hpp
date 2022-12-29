#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>
#include <map>
#include <xsim/multi_body.hpp>
#include "ur5_ik_solver.hpp"
#include <xsim/ompl_planner.hpp>
#include <xsim/world.hpp>

struct Robot {

    Robot(xsim::MultiBodyPtr c, xsim::PlanningInterface *p): controller_(c), iplan_(p) {}

    void openGripper() ;
    void closeGripper() ;
    bool plan(const Eigen::Isometry3f &target, xsim::JointTrajectory &traj) ;
    void executeTrajectory(xsim::PhysicsWorld &physics, const xsim::JointTrajectory &t, float speed) ;
    void moveTo(const JointState &target, float speed) ;
    void moveTo(const Eigen::Isometry3f &target, float speed) ;
    void setJointState(const std::string &name, float v) ;
    void setJointState(const JointState &state) ;
    float getJointState(const std::string &name) ;
    void getJointState(JointState &state) ;
    void stop() ;

    const std::vector<std::string> &armJointNames() const ;
    const std::map<std::string, double> &targetState() const { return target_state_ ; }


public:
    bool planRelative(const Eigen::Vector3f &dp, xsim::JointTrajectory &traj);
private:

    xsim::MultiBodyPtr controller_ ;
    JointState target_state_ ;
    xsim::PlanningInterface *iplan_ ;
    xsim::JointTrajectory traj_ ;
    bool ik(const Eigen::Isometry3f &target, JointState &);
};

#endif // ROBOT_HPP
