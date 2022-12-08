#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>
#include <map>
#include <xsim/multi_body.hpp>
#include "ur5_ik_solver.hpp"
struct Robot {

    Robot(xsim::MultiBodyPtr c): controller_(c) {}

    void openGripper() ;
    void closeGripper() ;
    void moveTo(const Eigen::Isometry3f &target) ;
    void setJointState(const std::string &name, float v) ;
    float getJointState(const std::string &name) ;
    void getJointState(std::map<std::string, float> &state) ;
    void stop() ;

    const std::vector<std::string> &armJointNames() const ;
    const std::map<std::string, double> &targetState() const { return target_state_ ; }

private:

    xsim::MultiBodyPtr controller_ ;
    JointState target_state_ ;

    bool ik(const Eigen::Isometry3f &target, JointState &);
};

#endif // ROBOT_HPP
