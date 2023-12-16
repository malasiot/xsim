#include "robot.hpp"
#include "kuka_iiwa_ik_solver.hpp"
#include "world.hpp"

using namespace xsim ;
using namespace Eigen ;
using namespace std ;

bool Robot::ik(const Eigen::Isometry3f &p, xsim::JointState &sol) {
    KukaIKSolver solver ;

    KukaIKSolver::Problem ik(p * orig_.inverse()) ;

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
        ctrl->setMotorControl(MotorControl(POSITION_CONTROL).setMaxVelocity(j_speed).setTargetPosition(v2));
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
    for( int i=0 ; i<10 ; i++ )
        world_->stepSimulation(0.05);
}

