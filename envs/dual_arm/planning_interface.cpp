#include "planning_interface.hpp"
#include "ur5_ik_solver.hpp"

using namespace xsim ;
using namespace std ;
using namespace Eigen ;

UR5Planning::UR5Planning(const xsim::URDFRobot &robot, xsim::CollisionSpace *collisions): robot_(robot), collisions_(collisions) {
    auto pose_ee = robot_.getLinkTransform("ee_link") ;
    auto pose_tool = robot_.getLinkTransform("ee_tool") ;

    tool_to_ee_ = pose_tool.inverse() * pose_ee ;
}

bool UR5Planning::isStateValid(const xsim::JointState &state)  {
    std::lock_guard<mutex> lock(mutex_) ;

    robot_.setJointState(state) ;

    map<string, Isometry3f> trs = robot_.getLinkTransforms() ;
    collisions_->updateObjectTransforms(trs) ;

    return !collisions_->hasCollision() ;
}

std::vector<std::string> UR5Planning::getJointChain() const {
    std::vector<string> joints ;
    std::copy(UR5IKSolver::ur5_joint_names, UR5IKSolver::ur5_joint_names + 6, std::back_inserter(joints)) ;
    return joints ;
}

void UR5Planning::getLimits(const std::string &name, double &lower, double &upper) const {
    KinematicJointPtr joint = robot_.getJoint(name) ;
    assert(joint) ;
    joint->getLimits(lower, upper) ;
}

bool UR5Planning::solveIK(const Eigen::Isometry3f &pose, std::vector<JointState> &solutions) const {
    UR5IKSolver solver ;
    return solver.solve(pose * tool_to_ee_, solutions) ;
}

bool UR5Planning::solveIK(const Eigen::Isometry3f &pose, const JointState &seed, JointState &solution) const {
    UR5IKSolver solver ;
    return solver.solve(pose * tool_to_ee_, seed, solution) ;
}

Isometry3f UR5Planning::getToolPose(const JointState &state) {
    robot_.setJointState(state) ;
    auto pose = robot_.getLinkTransform("ee_tool") ;
    return pose ;
}
