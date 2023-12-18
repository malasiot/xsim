#include "robot.hpp"
#include "ur5_ik_solver.hpp"
#include <xsim/joint_state_planner.hpp>
#include <xsim/task_space_planner.hpp>

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



void Robot::executeTrajectory(const JointTrajectory &traj, float speed) {
    if ( stcb_ )
        stcb_(traj) ;

    for( int i=0 ; i<traj.points().size()-1 ; i++ ) {
        const auto &t1 = traj.points()[i] ;
        const auto &t2 = traj.points()[i+1] ;

        cout << t1 << ' ' << t2 << endl ;
        move(t1, t2, speed) ;
        while (1) {
            world_->stepSimulation(0.05);

            auto state = getJointState() ;

            bool changed = false ;
            for( const auto &sp: state ) {
                float start_v = t2.find(sp.first)->second;
                float v = state[sp.first] ;

                if ( fabs(v - start_v) > 0.01 ) {
                    changed = true ;
                    break ;
                }
            }

            if ( !changed ) {
                stop() ;
                break ;
            }
        }
    }
}

void Robot::move(const JointState &start_state, const JointState &end_state, float speed) {

    double max_delta = 0.0 ;

    // find the largest movement

    for( const auto &jn: arm_joint_names ) {
        double v1 = start_state.find(jn)->second ;
        double v2 = end_state.find(jn)->second ;
        double delta = fabs(v1 - v2) ;
        if ( delta > max_delta ) {
            max_delta = delta ;
        }
    }

    for( const auto &jn: arm_joint_names ) {
        Joint *ctrl = controller_->findJoint(jn) ;

        double v1 = start_state.find(jn)->second ;
        double v2 = end_state.find(jn)->second ;
        double delta = fabs(v1 - v2) ;

        // scale motor speeds so that all motors arrive to their target at the same time
        double j_speed = speed * delta / max_delta ;
        ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(j_speed).setTargetPosition(v2));
    }
}

void Robot::moveTo(const JointState &target, float speed) {
    JointState start_state = getJointState() ;
    move(start_state, target, speed) ;
}


void Robot::setJointState(const std::string &name, float v) {
    controller_->setJointPosition(name, v) ;
}

void Robot::setJointState(const JointState &state) {
    for( const auto &jn: arm_joint_names ) {
        auto it = state.find(jn) ;
        if ( it != state.end() )
            controller_->setJointPosition(jn, it->second);
    }
}

JointState Robot::getJointState() const {
    JointState state ;
    for( const auto &jn: arm_joint_names ) {
        double v = controller_->getJointPosition(jn) ;
        state.emplace(jn, v) ;
    }
    return state ;
}

void Robot::stop() {
    JointState state ;
    for( const auto &j: arm_joint_names ) {
        Joint *ctrl = controller_->findJoint(j) ;
        ctrl->setMotorControl(MotorControl(VELOCITY_CONTROL).setTargetVelocity(0.0).setMaxForce(1000));
        state.emplace(j, ctrl->getPosition()) ;
    }
    for( int i=0 ; i<10 ; i++ )
        world_->stepSimulation(0.05);
}


const std::vector<string> &Robot::armJointNames() const {
    return arm_joint_names ;
}
