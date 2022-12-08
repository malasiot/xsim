#ifndef OMPLPLANNER_HPP
#define OMPLPLANNER_HPP

#include <Eigen/Geometry>
#include <map>

using JointState = std::map<std::string, double> ;

class PlanningInterface {
public:
    virtual JointState getJointState() = 0;
    virtual bool isStateValid(const JointState &s) = 0 ;
    virtual std::vector<std::string> getJointChain() const = 0 ;
    virtual void getLimits(const std::string &name, double &lower, double &upper) = 0 ;
    virtual bool solveIK(const Eigen::Isometry3f &pose, std::vector<JointState> &solutions) = 0;

};


class JointTrajectory {
public:

    JointTrajectory() {}

    void addPoint(double t, const JointState &state) {
        points_.push_back(state) ;
        times_.push_back(t) ;
    }

    // linear interpolation of joint at time t [0, 1]
    double lerp(const std::string &joint, double t) const ;

protected:

    std::vector<JointState> points_ ;
    std::vector<double> times_ ;
};

class PlannerBase {

public:
    enum Algorithm {
        KPIECE, BKPIECE, LBKPIECE, SBL, pSBL, EST, RRT, RRTConnect, LazyRRT, pRRT, PRM, PRMStar
    } ;

    PlannerBase(PlanningInterface *pi): iplan_(pi), alg_(LazyRRT), time_out_(5.0) {}

    void setAlgorithm(Algorithm alg) { alg_ = alg ; }
    void setTimeOut(double time_out) { time_out_ = time_out ; }

protected:

    Algorithm alg_ ;
    double time_out_ ;
    PlanningInterface *iplan_ ;
};


class JointSpacePlanner: public PlannerBase {

public:

    JointSpacePlanner(PlanningInterface *manip): PlannerBase(manip) {}

    // find a trajectory that brings the end-effector in one of the specified poses
    bool solve(const std::vector<Eigen::Isometry3f> &poses, JointTrajectory &traj) ;
};


#endif // OMPLPLANNER_HPP
