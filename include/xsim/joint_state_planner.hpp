#ifndef JOINT_STATE_PLANNER_HPP
#define JOINT_STATE_PLANNER_HPP

#include <xsim/ompl_planner.hpp>

#include <Eigen/Geometry>
#include <map>

namespace xsim {

class JointSpacePlanner: public OMPLPlannerBase {
public:

    JointSpacePlanner(PlanningInterface *manip): OMPLPlannerBase(manip) {}

    // find a trajectory that brings the end-effector in one of the specified poses
    bool solve(const std::vector<Eigen::Isometry3f> &poses, JointTrajectory &traj) ;

    bool solve(const Eigen::Isometry3f &target, JointTrajectory &traj) {
        std::vector<Eigen::Isometry3f> poses ;
        poses.push_back(target) ;
        return solve(poses, traj) ;
    }

protected:

    friend class OMPLValidityChecker ;
    friend class OMPLIKRegion ;

    static ompl::base::StateSpacePtr createOmplStateSpace(PlanningInterface *manip) ;
    static void setOmplState(const ompl::base::StateSpacePtr &ompl_state_space, ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &ompl_state, const JointState &state);
    static void getOmplState(const ompl::base::StateSpacePtr &ompl_state_space, const ompl::base::RealVectorStateSpace::StateType *ompl_state, JointState &state);
    static void getOmplTrajectory(const ompl::geometric::PathGeometric &path, const ompl::base::StateSpacePtr &ompl_state_space, JointTrajectory &joint_trajectory);
};

}
#endif
