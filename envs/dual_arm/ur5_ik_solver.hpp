#pragma once

#include <Eigen/Geometry>
#include <xsim/multi_body.hpp>
#include <xsim/kinematic.hpp>

class UR5IKSolver {
    public:
    // Joint names

    /*

    0: shoulder_pan_joint
    1:  shoulder_lift_joint
    2:  elbow_joint
    3:  wrist_1_joint
    4:  wrist_2_joint
    5:  wrist_3_joint
    */

    // analytic IK for UR5 arm
    // takes as input seed joint positions and target end effector position and returns joints or false
    // if no solution found
    bool solve(const Eigen::Isometry3f &target, const xsim::JointState &seed, xsim::JointState &solution) ;
    bool solve(const Eigen::Isometry3f &target, std::vector<xsim::JointState> &solutions) ;

    static double limits_min_[6], limits_max_[6] ;
    static const char *ur5_joint_names[6] ;

};



