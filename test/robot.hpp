#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>
#include <map>
#include <xsim/multi_body.hpp>
#include "ur5_ik_solver.hpp"
#include <xsim/ompl_planner.hpp>

struct Robot {

    Robot(xsim::MultiBodyPtr c, xsim::PlanningInterface *p): controller_(c), iplan_(p) {}

    void openGripper() ;
    void closeGripper() ;
    bool plan(const Eigen::Isometry3f &target) ;
    void executeTrajectory() ;
    void moveTo(const JointState &target) ;
    void moveTo(const Eigen::Isometry3f &target) ;
    void setJointState(const std::string &name, float v) ;
    void setJointState(const JointState &state) ;
    float getJointState(const std::string &name) ;
    void getJointState(std::map<std::string, float> &state) ;
    void stop() ;

    const std::vector<std::string> &armJointNames() const ;
    const std::map<std::string, double> &targetState() const { return target_state_ ; }

private:

    xsim::MultiBodyPtr controller_ ;
    JointState target_state_ ;
    std::unique_ptr<xsim::PlanningInterface> iplan_ ;
    xsim::JointTrajectory traj_ ;
    bool ik(const Eigen::Isometry3f &target, JointState &);
};

#endif // ROBOT_HPP
