#include "planner.hpp"
#include <xsim/joint_state_planner.hpp>
#include <xsim/task_space_planner.hpp>

using namespace xsim ;
using namespace std;
using namespace Eigen ;

bool Planner::plan(const JointState &start_state, const Eigen::Isometry3f &target, JointTrajectory &traj) {
    iplan_->setStartState(start_state) ;

    JointSpacePlanner planner(iplan_) ;

    return planner.solve(target, traj) ;
}


bool Planner::planRelative(const JointState &start_state, const Eigen::Vector3f &dp, xsim::JointTrajectory &traj) {
    iplan_->setStartState(start_state) ;

    Isometry3f pose = iplan_->getToolPose(start_state) ;

    Vector3f c0 = pose.translation() ;
    auto r0 = pose.linear() ;
    auto euler = r0.eulerAngles(0, 1, 2);
    Vector3f c1 = c0 + dp ;

    Isometry3f target_pose ;
    target_pose.setIdentity() ;
    target_pose.linear() = r0 ;
    target_pose.translation() = c1 ;

    // setup the goal region

    BoxShapedRegion goal(c1, {0.01, 0.01, 0.02}, euler, {0.05, 0.05, 0.05}) ;
    MoveRelativeTaskSpace ts(pose, dp, 0.01, { 0.05, 0.05, 0.05}) ;

    TaskSpacePlanner planner(iplan_) ;
    return planner.solve(goal, ts, traj);
}
