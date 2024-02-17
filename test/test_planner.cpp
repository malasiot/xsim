#include <xsim/joint_state_planner.hpp>
#include <xsim/task_space_planner.hpp>
#include <xsim/collision_space.hpp>
#include <xsim/goal_region.hpp>
#include <xsim/task_space.hpp>
#include <iostream>
#include "ur5_ik_solver.hpp"
#include <chrono>
#include <thread>

using namespace xsim ;
using namespace Eigen ;
using namespace std ;


class UR5Planning: public PlanningInterface {
public:
    UR5Planning(const URDFRobot &urdf): robot_(urdf) {

         collisions_.addRobot(urdf) ;

         CollisionShapePtr box(new BoxCollisionShape{{0.05, 0.05, 0.05}}) ;
       //  collisions_.addCollisionObject("box1", box, Isometry3f(Translation3f{{0.25, 0.15, 0.38}})) ;
      //   collisions_.addCollisionObject("box2", box, Isometry3f(Translation3f{{0, 0, 0.9}})) ;

         start_state_ = robot_.getJointState();
        // for ( int i=0 ; i<6  ; i++) {
        //     const URDFJoint *joint = urdf.findJoint(UR5IKSolver::ur5_joint_names[i]) ;
        //     start_state_.emplace(UR5IKSolver::ur5_joint_names[i], joint->position_) ;
       //  }
    }

    bool isStateValid(const JointState &state) override  {
        robot_.setJointState(state) ;
        collisions_.updateObjectTransforms(robot_.getLinkTransforms()) ;
        return !collisions_.hasCollision() ;

    }
    std::vector<std::string> getJointChain() const override {
        vector<string> joints ;
        std::copy(UR5IKSolver::ur5_joint_names, UR5IKSolver::ur5_joint_names + 6, std::back_inserter(joints)) ;
        return joints ;
    }
    void getLimits(const std::string &name, double &lower, double &upper) const override {
        KinematicJointPtr joint = robot_.getJoint(name) ;
        assert(joint) ;
        joint->getLimits(lower, upper) ;
    }

    bool solveIK(const Eigen::Isometry3f &pose, std::vector<JointState> &solutions) const override {
        UR5IKSolver solver ;
        return solver.solve(pose, solutions) ;
    }

    bool solveIK(const Eigen::Isometry3f &pose, const JointState &seed, JointState &solution) const override {
        UR5IKSolver solver ;
        return solver.solve(pose, seed, solution) ;
    }

    Isometry3f getToolPose(const JointState &state) override {
        robot_.setJointState(state) ;
        return robot_.getLinkTransform("ee_link") ;
    }

    JointState getJointState() {
        JointState state ;
         for ( int i=0 ; i<6  ; i++) {
           state.emplace(UR5IKSolver::ur5_joint_names[i], robot_.getJointPosition(UR5IKSolver::ur5_joint_names[i])) ;
         }
         return state ;
    }

 //   void updateCollisionEnv(const std::map<std::string, Eigen::Isometry3f> &trs) override {}


    KinematicModel robot_ ;
    CollisionSpace collisions_ ;
};

bool move_arm_tip(PlanningInterface &iplan, const Vector3f &dp, JointTrajectory &traj)
{

    Isometry3f pose = iplan.getToolPose(traj.points().front()) ;

    Vector3f c0 = pose.translation() ;
    auto r0 = pose.linear() ;
    Vector3f c1 = c0 + dp ;

    Isometry3f target_pose ;
    target_pose.linear() = r0 ;
    target_pose.translation() = c1 ;

    // setup the goal region

    SimplePoseGoal goal(target_pose) ;
    MoveRelativeTaskSpace ts(pose, dp, 0.02, { 0.01, 0.01, 0.001}) ;

    TaskSpacePlanner planner(&iplan) ;
    return planner.solve(goal, ts, traj);



}

int main(int argc, char *argv[]) {
    string path = "/home/malasiot/source/xsim/data/" ;
    auto robot = URDFRobot::loadURDF(path + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;
    robot.setJointPosition("shoulder_lift_joint", 0.5) ;

    UR5Planning iplan(robot) ;

    auto start_state = iplan.getJointState();
{
    iplan.setStartState(start_state) ;
    JointSpacePlanner planner(&iplan) ;

    Quaternionf rot{0, -1, 0, 1};
    rot.normalize() ;

    Isometry3f pose ;
    pose.setIdentity() ;
    pose.linear() = rot.matrix() ;
    pose.translation() = Vector3f{ 0.25, 0.25, 0.2} ;


    JointTrajectory traj ;
    planner.solve(pose, traj) ;

    start_state = traj.points().back() ;
}

    {
        iplan.setStartState(start_state) ;
        JointTrajectory traj ;
        move_arm_tip(iplan, { 0.4, 0.0, 0.0}, traj) ;
        cout << "ok" << endl ;
    }


}
