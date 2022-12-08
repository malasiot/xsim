#include "robot.hpp"
#include "ur5_ik_solver.hpp"

using namespace xsim ;
using namespace Eigen ;
using namespace std ;

static vector<string> arm_joint_names = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

static const char *gripper_roll = "wrist_3_joint" ;
static const char *mimic_joint_names[] = {
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint"
};

static const char *gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint";


void ik(MultiBody &body, const Isometry3f &ee) {


    JointState seed, j ;
    for( uint i=0 ; i<6 ; i++ ) {
        double pos = body.getJointPosition(arm_joint_names[i]);
        seed[arm_joint_names[i]] = pos ;
    }

    UR5IKSolver solver ;
    if ( solver.solve(ee, seed, j) ) {
        for( const auto &s: j ) {
            Joint *ctrl = body.findJoint(s.first) ;
            ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(0.9).setTargetPosition(s.second));
            // body.setTargetPosition(arm_joint_names[i], j[i]) ;
        }
    }
}




void Robot::openGripper() {

    Joint *ctrl = controller_->findJoint(gripper_main_control_joint_name) ;

    MotorControl params(POSITION_CONTROL) ;
    params.setMaxVelocity(1.5);
    params.setMaxForce(0.1) ;
    params.setTargetPosition(0.001) ;
    ctrl->setMotorControl(params);

    for( Joint *mimic: ctrl->getMimicJoints()) {
        params.setTargetPosition(mimic->getMimicMultiplier() * 0.001) ;
        mimic->setMotorControl(params);
    }

}

void Robot::closeGripper() {
    Joint *ctrl = controller_->findJoint(gripper_main_control_joint_name) ;

    MotorControl params(POSITION_CONTROL) ;
    params.setMaxVelocity(1.5);
    params.setMaxForce(0.1) ;
    params.setTargetPosition(0.8) ;
    ctrl->setMotorControl(params);

    for( Joint *mimic: ctrl->getMimicJoints()) {
        params.setTargetPosition(mimic->getMimicMultiplier() * 0.8) ;
        mimic->setMotorControl(params);
    }

}

void Robot::moveTo(const Eigen::Isometry3f &target) {
    JointState j ;
    if ( ik(target, j) ) {
        target_state_ = j ;
        for( const auto &jn: arm_joint_names ) {
            Joint *ctrl = controller_->findJoint(jn) ;
            ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(0.9).setTargetPosition(j[jn]));
        }
    }
}

void Robot::setJointState(const std::string &name, float v)
{
    controller_->setJointPosition(name, v) ;
}

void Robot::getJointState(std::map<std::string, float> &state) {
    for( const auto &jn: arm_joint_names ) {
        float v = controller_->getJointPosition(jn) ;
        state.emplace(jn, v) ;
    }
}

void Robot::stop()
{
    for( const auto &j: arm_joint_names ) {
        Joint *ctrl = controller_->findJoint(j) ;
        ctrl->setMotorControl(MotorControl(VELOCITY_CONTROL).setTargetVelocity(0.0).setMaxForce(1000));
        // body.setTargetPosition(arm_joint_names[i], j[i]) ;
    }
}

const std::vector<string> &Robot::armJointNames() const
{
    return arm_joint_names ;
}

bool Robot::ik(const Isometry3f &target, JointState &j) {
    JointState seed ;
    for( uint i=0 ; i<6 ; i++ ) {
        double pos = controller_->getJointPosition(arm_joint_names[i]);
        seed[arm_joint_names[i]] = pos ;
    }

    UR5IKSolver solver ;
    return  solver.solve(target, seed, j) ;
}
