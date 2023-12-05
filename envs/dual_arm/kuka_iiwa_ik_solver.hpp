#pragma once

#include <Eigen/Geometry>
#include <xsim/multi_body.hpp>
#include <xsim/kinematic.hpp>

class KukaIKSolver {
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

    // analytic IK for Kuka IWAA 7 dof arm
    // takes as input seed joint positions and target end effector position and returns joints or false
    // if no solution found

    bool solve(const Eigen::Isometry3f &target, double psi, const xsim::JointState &seed, xsim::JointState &solution) ;
    bool solve(const Eigen::Isometry3f &target, double psi, std::vector<xsim::JointState> &solutions) ;

    static double limits_min_[7], limits_max_[7] ;
    static const char *s_joint_names[7] ;

    Eigen::Isometry3d forward(const xsim::JointState &state);

    bool solve(const Eigen::Isometry3f &target, double nsparam, uint rconf, double js[7]) ;

    bool checkLimits(double js[7]) ;
};



