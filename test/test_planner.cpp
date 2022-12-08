#include "ompl_planner.hpp"
#include <xsim/collision_space.hpp>
#include <iostream>
#include "ur5_ik_solver.hpp"

using namespace xsim ;
using namespace Eigen ;
using namespace std ;
using namespace xviz ;

class UR5Planning: public PlanningInterface {
public:
    UR5Planning(const string &path) {
         robot_ = URDFRobot::load(path + "robots/ur5/ur5_robotiq85_gripper.urdf" ) ;
         collisions_.addRobot(robot_) ;

         CollisionShapePtr box(new BoxCollisionShape{{0.05, 0.05, 0.05}}) ;
         collisions_.addCollisionObject("box1", box, Isometry3f(Translation3f{{0.25, 0.15, 0.38}})) ;
         collisions_.addCollisionObject("box2", box, Isometry3f(Translation3f{{0, 0, 0.9}})) ;
    }

    JointState getJointState() override {
        JointState state ;
        for ( int i=0 ; i<6  ; i++) {
            URDFJoint *joint = robot_.findJoint(UR5IKSolver::ur5_joint_names[i]) ;
            state.emplace(UR5IKSolver::ur5_joint_names[i], joint->position_) ;
        }
        return state ;
    }

    bool isStateValid(const JointState &state) override  {
        for( const auto &sp: state ) {
           robot_.setJointPosition(sp.first, sp.second) ;
        }
        map<string, Isometry3f> trs ;
        robot_.computeLinkTransforms(trs);
        collisions_.updateObjectTransforms(trs) ;
        return collisions_.hasCollision() ;

    }
    std::vector<std::string> getJointChain() const override {
        vector<string> joints ;
        std::copy(UR5IKSolver::ur5_joint_names, UR5IKSolver::ur5_joint_names + 6, std::back_inserter(joints)) ;
        return joints ;
    }
    void getLimits(const std::string &name, double &lower, double &upper) override {
        URDFJoint *joint = robot_.findJoint(name) ;
        assert(joint) ;
        lower = joint->lower_ ;
        upper = joint->upper_ ;
    }
    bool solveIK(const Eigen::Isometry3f &pose, std::vector<JointState> &solutions) override {
        UR5IKSolver solver ;
        return solver.solve(pose, solutions) ;
    }

    URDFRobot robot_ ;
    CollisionSpace collisions_ ;
};

int main(int argc, char *argv[]) {
    string path = "/home/malasiot/source/xsim/data/" ;

    UR5Planning iplan(path) ;

    JointSpacePlanner planner(&iplan) ;

    Quaternionf rot{0, -1, 0, 1};
    rot.normalize() ;

    Isometry3f pose ;
    pose.setIdentity() ;
    pose.linear() = rot.matrix() ;
    pose.translation() = Vector3f{ 0.25, 0.25, 0.2} ;

    vector<Isometry3f> poses ;
    poses.push_back(pose) ;

    JointTrajectory traj ;
    planner.solve(poses, traj) ;

    cout << "ok" << endl ;
}
