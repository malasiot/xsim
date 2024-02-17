#include "robot.hpp"
#include "kuka_iiwa_ik_solver.hpp"
#include "world.hpp"

using namespace xsim ;
using namespace Eigen ;
using namespace std ;

bool Robot::ik(const Eigen::Isometry3f &p, xsim::JointState &sol) {
    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(p* orig_.inverse()) ; // robot coordinates

    std::vector<JointCoeffs> solutions ;
    if ( solver.solve(ik,  solutions) ) {
        for( const auto solution: solutions ) {
            JointState state ;
            for( uint j=0 ; j<7 ; j++ ) {
                state.emplace(prefix_ + KukaIKSolver::s_joint_names[j], solution[j]) ;
            }

            if ( checker_(state) ) {
                sol = state ;
                return true ;
            }
        }
    }
    return false ;
}

bool Robot::ik(const Eigen::Isometry3f &p, const xsim::JointState &seed, xsim::JointState &sol) {
    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(p * orig_.inverse()) ;

    JointCoeffs c = {0} ;
    for( uint j=0 ; j<7 ; j++ ) {
        string sn = prefix_ + KukaIKSolver::s_joint_names[j] ;
        auto it = seed.find(sn) ;
        if ( it != seed.end() )
            c[j] = it->second ;
    }

    ik.setSeedState(c) ;

    std::vector<JointCoeffs> solutions ;
    if ( solver.solve(ik,  solutions) ) {
        for( const auto solution: solutions ) {
            JointState state ;
            for( uint j=0 ; j<7 ; j++ ) {
                state.emplace(prefix_ + KukaIKSolver::s_joint_names[j], solution[j]) ;
            }

            if ( checker_(state) ) {
                sol = state ;
                return true ;
            }
        }
    }
    return false ;
}

bool Robot::setPose(const Eigen::Isometry3f &p) {
    JointState sol ;
    if ( ik(p, sol) ) {
        setJointState(sol) ;
        return true ;
    }

    return false ;
}


void Robot::move(const JointState &start_state, const JointState &end_state, float speed) {

    double max_delta = 0.0 ;

    // find the largest movement

    for( const auto &jn: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + jn ;
        double v1 = start_state.find(sjn)->second ;
        double v2 = end_state.find(sjn)->second ;
        double delta = fabs(v1 - v2) ;
        if ( delta > max_delta ) {
            max_delta = delta ;
        }
    }

    for( const auto &jn: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + jn ;
        Joint *ctrl = controller_->findJoint(sjn) ;

        double v1 = start_state.find(sjn)->second ;
        double v2 = end_state.find(sjn)->second ;
        double delta = fabs(v1 - v2) ;

        // scale motor speeds so that all motors arrive to their target at the same time
        double j_speed = speed * delta / max_delta ;
        ctrl->setMotorControl(MotorControl(POSITION_CONTROL)
                              .setMaxVelocity(j_speed)
                              .setTargetPosition(v2)
                              .setMaxForce(0.4));
    }
}

void Robot::moveTo(const JointState &target, float speed) {
    JointState start_state = getJointState() ;
    move(start_state, target, speed) ;
}


void Robot::moveTo(const Eigen::Isometry3f &pose, float speed)
{
    JointState target ;
    if ( ik(pose, getJointState(), target) ) {
        moveTo(target, speed) ;
    }

}

void Robot::cartesian(const Eigen::Isometry3f &pose, xsim::JointTrajectory &traj)
{
    JointState start = getJointState() ;
    JointState target ;
    if ( ik(pose, start, target) ) {
        traj.addPoint(0, start) ;
        traj.addPoint(1, target) ;
    }

}

/*
void Robot::cartesian(const Eigen::Isometry3f &pose, xsim::JointTrajectory &traj)
{
    Isometry3f tr_start = controller_->getLink(prefix_ + "tool0")->getWorldTransform() ;

    Quaternionf q_start(tr_start.rotation()) ;
    Vector3f t_start = tr_start.translation() ;

    Quaternionf q_target(pose.rotation()) ;
    Vector3f t_target = pose.translation() ;

    const float step_sz = 0.03 ;

    float l = (t_target - t_start).norm() ;
    size_t n_steps = l / step_sz ;

    JointState start = getJointState(), prev = start ;
    traj.addPoint(0, start) ;

    for( float s = step_sz ; s < l ; s+=step_sz ) {
        float t = s/l ;
        Quaternionf q = q_start.slerp(t, q_target);
        Vector3f tr = (1 - t) * t_start  + t * t_target ;

        Isometry3f p = Isometry3f::Identity() ;
        p.linear() = q_start.toRotationMatrix() ;
        p.translation() = tr ;

        JointState target ;
        if ( ik(p, prev, target) ) {
           traj.addPoint(t, target) ;
        }
        prev = target ;
    }

}
*/

void Robot::setJointState(const std::string &name, float v) {
    controller_->setJointPosition(name, v) ;
}

void Robot::setJointState(const JointState &state) {
    for( const auto &jn: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + jn ;
        auto it = state.find(sjn) ;
        if ( it != state.end() )
            controller_->setJointPosition(sjn, it->second);
    }
}

JointState Robot::getJointState() const {
    JointState state ;
    for( const auto &jn: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + jn ;
        double v = controller_->getJointPosition(sjn) ;
        state.emplace(sjn, v) ;
    }
    return state ;
}

void Robot::stop() {
    JointState state ;
    for( const auto &j: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + j ;
        Joint *ctrl = controller_->findJoint(sjn) ;
        ctrl->setMotorControl(MotorControl(VELOCITY_CONTROL).setTargetVelocity(0.0).setMaxForce(1000));
        state.emplace(sjn, ctrl->getPosition()) ;
    }

}


std::vector<float> Robot::getTorques() const
{
    std::vector<float> t ;

    for( const auto &j: KukaIKSolver::s_joint_names ) {
        string sjn = prefix_ + j ;
        double torque = controller_->getJointTorque(sjn) ;
        t.emplace_back(torque);
    }
    return t ;
}
