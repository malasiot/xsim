#pragma once

#include <Eigen/Geometry>
#include <xsim/multi_body.hpp>

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
    bool solve(const double jseed[6], const Eigen::Isometry3f &target, float roll, double j[6]) ;
private:

    static double limits_min_[6], limits_max_[6] ;

};
