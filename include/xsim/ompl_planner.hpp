#ifndef OMPL_PLANNER_COMMON_HPP
#define OMPL_PLANNER_COMMON_HPP

#include <xsim/kinematic.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalStates.h>

namespace xsim {

class PlanningInterface {
public:
    // should return the values of joints used for planning
    const xsim::JointState &getStartState() { return start_state_ ; }
    void setStartState(const JointState &state) { start_state_ = state ; }

    // perform collision detection or other checks to test the validity of the state
    virtual bool isStateValid(const xsim::JointState &s) = 0 ;
    // get list of joint name that will be searched for
    virtual std::vector<std::string> getJointChain() const = 0 ;
    // get joint limits
    virtual void getLimits(const std::string &name, double &lower, double &upper) const = 0 ;
    // solve IK problem
    virtual bool solveIK(const Eigen::Isometry3f &pose, std::vector<xsim::JointState> &solutions) const = 0;
    virtual bool solveIK(const Eigen::Isometry3f &pose, const JointState &seed, xsim::JointState &solution) const = 0;

 //   virtual void updateCollisionEnv(const std::map<std::string, Eigen::Isometry3f> &trs) = 0 ;

    // get the world coordinates of the link used for task planning at the start state
    virtual Eigen::Isometry3f getToolPose(const JointState &state) = 0 ;

    const JointState &startState() const { return start_state_ ; }

protected:
    JointState start_state_ ;
};


class JointTrajectory {
public:

    JointTrajectory() {}

    void clear() {
        points_.clear() ;
        times_.clear() ;
    }

    void addPoint(double t, const JointState &state) {
        points_.push_back(state) ;
        times_.push_back(t) ;
    }

    void append(const JointTrajectory &other) ;

    JointState getState(float t, PlanningInterface *iplan) const ;

    // linear interpolation of joint at time t [0, 1]
    double lerp(const std::string &joint, double t) const ;

    const std::vector<JointState> &points() const { return points_ ; }

    friend std::ostream &operator<<(std::ostream &strm, const JointTrajectory &traj) {
        for( int i=0 ; i<traj.points_.size() ; i++ ) {
            strm << "wpt: " << i << std::endl ;
            strm << traj.points_[i] << std::endl ;
        }
        return strm ;
    }

protected:

    std::vector<JointState> points_ ;
    std::vector<double> times_ ;
};


class OMPLPlannerBase {
public:
    enum Algorithm {
        KPIECE, BKPIECE, LBKPIECE, SBL, pSBL, EST, RRT, RRTConnect, LazyRRT, pRRT, PRM, PRMStar
    } ;

    OMPLPlannerBase(PlanningInterface *pi): iplan_(pi), alg_(RRTConnect), time_out_(1.0) {}

    void setAlgorithm(Algorithm alg) { alg_ = alg ; }
    void setTimeOut(double time_out) { time_out_ = time_out ; }

protected:

    Algorithm alg_ ;
    double time_out_ ;
    PlanningInterface *iplan_ ;




};



}
#endif
