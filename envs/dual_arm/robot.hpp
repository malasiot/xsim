#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Geometry>
#include <map>
#include <functional>
#include <xsim/multi_body.hpp>

#include <xsim/ompl_planner.hpp>


class World ;

// class to control robot movement and state

using StartTrajectoryCallback = std::function<void(const xsim::JointTrajectory &)> ;
using StateValidityChecker = std::function<bool (const xsim::JointState &)> ;

struct Robot {

    Robot(const std::string &prefix, const xsim::MultiBodyPtr &c, const Eigen::Isometry3f &bw): prefix_(prefix), controller_(c), orig_(bw) {}

    void setStateValidityChecker(StateValidityChecker checker) {
        checker_ = checker ;
    }

    bool ik(const Eigen::Isometry3f &p, xsim::JointState &sol) ;
    bool ik(const Eigen::Isometry3f &p, const xsim::JointState &seed, xsim::JointState &sol) ;

    bool setPose(const Eigen::Isometry3f &p) ;

    void executeTrajectory(const xsim::JointTrajectory &t, float speed) ;

    void move(const xsim::JointState &start_state, const xsim::JointState &end_state, float speed) ;
    void moveTo(const xsim::JointState &target, float speed) ;
    void moveTo(const Eigen::Isometry3f &pose, float speed) ;
    void cartesian(const Eigen::Isometry3f &pose, xsim::JointTrajectory &traj) ;

    void setJointState(const std::string &name, float v) ;
    void setJointState(const xsim::JointState &state) ;
    float getJointState(const std::string &name) ;
    xsim::JointState getJointState() const ;
    void stop() ;

    std::vector<float> getTorques() const ;

    void setStartTrajectoryCallback(StartTrajectoryCallback cb) { stcb_ = cb ; }

    const std::string &prefix() const { return prefix_ ; }

    const Eigen::Isometry3f &origin() const { return orig_ ; }

private:

    xsim::MultiBodyPtr controller_ ;

    Eigen::Isometry3f orig_ ;
    std::string prefix_ ;
    StartTrajectoryCallback stcb_ = nullptr ;
    StateValidityChecker checker_ = nullptr ;
};

#endif // ROBOT_HPP
