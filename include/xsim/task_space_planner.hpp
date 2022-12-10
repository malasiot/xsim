#ifndef TASK_SPACE_PLANNER_HPP
#define TASK_SPACE_PLANNER_HPP

#include <xsim/ompl_planner.hpp>
#include <xsim/goal_region.hpp>
#include <xsim/task_space.hpp>

#include <Eigen/Geometry>
#include <map>

namespace xsim {

class GoalRegion ;
class TaskSpace ;

class TaskSpacePlanner: public OMPLPlannerBase {
public:

    TaskSpacePlanner(PlanningInterface *manip): OMPLPlannerBase(manip) {}

    // find a trajectory that brings the end-effector in one of the specified poses
    bool solve(const GoalRegion &goal, const TaskSpace &ts, JointTrajectory &traj) ;


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
