#pragma once

#include <xsim/collision_space.hpp>
#include <xsim/ompl_planner.hpp>

class Planner {
public:
    Planner(xsim::PlanningInterface *iplan):
        iplan_(iplan) {}

    bool plan(const xsim::JointState &start_state, const Eigen::Isometry3f &target, xsim::JointTrajectory &traj) ;
    bool planRelative(const xsim::JointState &start_state, const Eigen::Vector3f &dp, xsim::JointTrajectory &traj);

private:
    xsim::PlanningInterface *iplan_ ;
};
