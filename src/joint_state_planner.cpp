#include <xsim/joint_state_planner.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

using namespace std ;
using namespace Eigen ;

namespace xsim {

class OMPLValidityChecker: public ompl::base::StateValidityChecker {
public:

    OMPLValidityChecker(const ompl::base::SpaceInformationPtr &si,
                        PlanningInterface *manip):
        StateValidityChecker(si), manip_(manip),
        ompl_state_space_(si->getStateSpace()){}

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space_ ;
    PlanningInterface *manip_ ;
};

class OMPLIKRegion : public ompl::base::GoalStates {
public:

    OMPLIKRegion(const ompl::base::SpaceInformationPtr &space_information,
                           const std::vector<Eigen::Isometry3f> &poses,
                           PlanningInterface *manip) ;

    virtual bool hasStates(void) const {
        return num_states_ > 0 ;
    }

private:

    PlanningInterface* manip_;
    int num_states_ ;
};

ompl::base::StateSpacePtr JointSpacePlanner::createOmplStateSpace(PlanningInterface *manip)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    const vector<string> &chain= manip->getJointChain() ;

    int nDOFs = chain.size() ;

    RealVectorBounds real_vector_bounds(0) ;
    RealVectorStateSpace *state_space = new RealVectorStateSpace(nDOFs) ;

    int dim = 0 ;

    for(int i=0 ; i<chain.size() ; i++ ) {
        double lower, upper ;
        manip->getLimits(chain[i], lower, upper) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);

        state_space->setDimensionName(dim, chain[i])  ;

        ++dim ;
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}


void JointSpacePlanner::setOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
                  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &ompl_state,
             const JointState &state) {
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( const auto &sp: state ) {
        const string &name = sp.first ;

        int idx = state_space->getDimensionIndex(name) ;

        if ( idx == -1 ) continue ;

        ompl_state[idx] = sp.second ;

    }
}


void JointSpacePlanner::getOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             JointState &state) {
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ )
    {
        const string &name = state_space->getDimensionName(i) ;

         double val = (*ompl_state)[i] ;

         state[name] = val ;
    }
}

void JointSpacePlanner::getOmplTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       JointTrajectory &joint_trajectory) {
    using namespace ompl::base ;

    unsigned int num_points = path.getStateCount();

    double t = 0.0, tstep = 1.0/(num_points - 1) ;

    for(int i=0 ; i<num_points ; i++) {
        const RealVectorStateSpace::StateType *state =
                path.getState(i)->as<RealVectorStateSpace::StateType>() ;

        JointState js ;
        getOmplState(ompl_state_space, state, js) ;

        joint_trajectory.addPoint(t, js) ;

        t += tstep ;
    }
}

bool OMPLValidityChecker::isValid(const ompl::base::State *state) const {
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    JointState js ;
    JointSpacePlanner::getOmplState(ompl_state_space_, ompl_state, js) ;

    return manip_->isStateValid(js) ;
}

OMPLIKRegion::OMPLIKRegion(const ompl::base::SpaceInformationPtr &space_information,
                           const std::vector<Isometry3f> &poses,
                           PlanningInterface *manip):
    ompl::base::GoalStates(space_information), manip_(manip) {

    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = space_information->getStateSpace() ;

    num_states_ = 0 ;

    for( int i=0 ; i<poses.size() ; i++ )
    {
        std::vector<JointState> solutions ;

        if ( manip->solveIK(poses[i], solutions) )
        {
            for( int j=0 ; j<solutions.size() ; j++ )
            {

                ScopedState<RealVectorStateSpace> ompl_state(ompl_state_space) ;

                JointSpacePlanner::setOmplState(ompl_state_space, ompl_state, solutions[j]) ;

                addState(ompl_state) ;

                num_states_ ++ ;
            }
        }
    }
}


bool JointSpacePlanner::solve(const std::vector<Isometry3f> &poses,
                                 JointTrajectory &traj) {
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplStateSpace(iplan_) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OMPLValidityChecker(si, iplan_)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    JointState js = iplan_->getStartState() ;
    setOmplState(ompl_state_space, start_state, js) ;

    // set the goal region
    ompl::base::GoalPtr goal;

    OMPLIKRegion *region = new OMPLIKRegion(si, poses, iplan_) ;
    goal.reset(region);
    if ( !region->hasStates() ) return false ;

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goal) ;

    // set the planner

    PlannerPtr ompl_planner ;

    switch ( alg_ )
    {

    case RRT:
        ompl_planner.reset(new ompl::geometric::RRT(si)) ;
        break ;
    case pRRT:
        ompl_planner.reset(new ompl::geometric::pRRT(si)) ;
        break ;
    case LazyRRT:
        ompl_planner.reset(new ompl::geometric::LazyRRT(si)) ;
        break ;
    case RRTConnect:{
        auto p = new ompl::geometric::RRTConnect(si) ;
        p->setRange(0) ;
        ompl_planner.reset(p) ;
        break ;
    }

    case KPIECE:
        ompl_planner.reset(new ompl::geometric::KPIECE1(si)) ;
        break ;
    case BKPIECE:
        ompl_planner.reset(new ompl::geometric::BKPIECE1(si)) ;
        break ;
    case LBKPIECE:
        ompl_planner.reset(new ompl::geometric::LBKPIECE1(si)) ;
        break ;
    case SBL:
        ompl_planner.reset(new ompl::geometric::SBL(si)) ;
        break ;
    case pSBL:
        ompl_planner.reset(new ompl::geometric::pSBL(si)) ;
        break ;
    default:
        return false ;
    }


    //planner->setGoalBias(0.0) ;
    //planner->setRange(0.0) ;

    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out_);

    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;


        ompl::geometric::PathGeometric orig(path) ;
        // simplify path
        if ( pathSimplifier->simplifyMax(path) )
            getOmplTrajectory(path, ompl_state_space, traj) ;
        else
            getOmplTrajectory(orig, ompl_state_space, traj) ;

        return true ;

    }

    return false ;

}

}
/////////////////////////////////////////////////////////////////////////////////

#if 0
ompl::base::StateSpacePtr createOmplStateSpace(PlanningGroup *manip)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    const vector<string> &chain= manip->getJointChain() ;
    const string &freeJoint = manip->getFreeJoint() ;
    KinematicModel *model = manip->getModel() ;

    int nDOFs = chain.size() ;

    if ( !freeJoint.empty() ) nDOFs -- ;

    RealVectorBounds real_vector_bounds(0) ;
    RealVectorStateSpace *state_space = new RealVectorStateSpace(nDOFs) ;

    int dim = 0 ;

    for(int i=0 ; i<chain.size() ; i++ )
    {
        if ( chain[i] == freeJoint ) continue ;

        double lower, upper ;
        model->getLimits(chain[i], lower, upper) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);

        state_space->setDimensionName(dim, chain[i])  ;

        ++dim ;
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}

ompl::base::StateSpacePtr createOmplTaskStateSpace(const TaskSpace *ts)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    RealVectorBounds real_vector_bounds(0) ;

    int dim = ts->getDimension() ;

    RealVectorStateSpace *state_space = new RealVectorStateSpace(ts->getDimension()) ;

    for(int i=0 ; i<dim ; i++ )
    {

        double lower = ts->getLowerLimit(i) ;
        double upper = ts->getUpperLimit(i) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}

void setOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
                  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &ompl_state,
             const JointState &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;


    map<string, JointState::DOF>::const_iterator it = state.data.begin() ;

    for( ; it != state.data.end() ; ++it )
    {
        const string &name = (*it).first ;

        int idx = state_space->getDimensionIndex(name) ;

        if ( idx == -1 ) continue ;

        ompl_state[idx] = (*it).second.val ;

    }
}

void setOmplTaskState(const ompl::base::StateSpacePtr &ompl_state_space,
                  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &ompl_state,
             const std::vector<double> &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state.size() ; i++ )
    {
        ompl_state[i] = state[i] ;
    }


}


void getOmplState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             JointState &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ )
    {
        const string &name = state_space->getDimensionName(i) ;

        JointState::DOF dof ;

        dof.idx = 0 ;
        dof.val = (*ompl_state)[i] ;

        state.data[name] = dof ;
    }
}

void getOmplTaskState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             std::vector<double> &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ )
    {
        state.push_back((*ompl_state)[i]) ;
    }
}

void getOmplTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       JointTrajectory &joint_trajectory)
{
    using namespace ompl::base ;

    unsigned int num_points = path.getStateCount();

    double t = 0.0, tstep = 1.0/(num_points - 1) ;

    for(int i=0 ; i<num_points ; i++)
    {
        const RealVectorStateSpace::StateType *state =
                path.getState(i)->as<RealVectorStateSpace::StateType>() ;

        JointState js ;
        getOmplState(ompl_state_space, state, js) ;

        joint_trajectory.addPoint(t, js) ;

        t += tstep ;

    }



}

bool getOmplTaskTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       const TaskSpace *ts,
                       const PlanningGroup *group,
                       JointTrajectory &joint_trajectory)
{
    using namespace ompl::base ;

    unsigned int num_points = path.getStateCount();

    double t = 0.0, tstep = 1.0/(num_points - 1) ;

    JointState previous_state ;

    for(int i=0 ; i<num_points ; i++)
    {
        const RealVectorStateSpace::StateType *state =
                path.getState(i)->as<RealVectorStateSpace::StateType>() ;

        std::vector<double> tsv ;
        getOmplTaskState(ompl_state_space, state, tsv) ;

        // convert this state to pose and find an IK solution
        vector<Matrix4x4> pose ;
        ts->taskSpaceToPose(tsv, pose) ;

        if ( i==0 )
        {
            vector<JointState> solutions ;
            if ( !group->solve(pose, solutions) )
            {
                return false ;
            }
            else {
                previous_state = solutions[0] ;
                joint_trajectory.addPoint(t, previous_state) ;
            }
        }
        else
        {
            JointState solution ;

            if ( !group->solve(pose, previous_state, solution) ) {
                return false ;
            }
            else {
                previous_state = solution ;
                joint_trajectory.addPoint(t, previous_state) ;
            }
        }

        t += tstep ;

    }

    return true ;
}

///////////////////////////////////////////////////////////////////////////////

class OmplIKRegion : public ompl::base::GoalStates
{
public:

    OmplIKRegion(const ompl::base::SpaceInformationPtr &space_information,
                           const std::vector<Matrix4x4> &poses,
                           Manipulator *manip) ;

    virtual bool hasStates(void) const {
        return numStates > 0 ;
    }



private:

    IKSolver* solver_;
    bool numStates ;
};


OmplIKRegion::OmplIKRegion(const ompl::base::SpaceInformationPtr &space_information,
                           const std::vector<Matrix4x4> &poses,
                           Manipulator *manip):
    ompl::base::GoalStates(space_information)
{

    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = space_information->getStateSpace() ;

    IKSolver *solver = manip->getIKSolver() ;

    numStates = 0 ;

    for( int i=0 ; i<poses.size() ; i++ )
    {
        std::vector<JointState> solutions ;

        if ( solver->solve(poses[i], solutions) )
        {
            for( int j=0 ; j<solutions.size() ; j++ )
            {

                ScopedState<RealVectorStateSpace> ompl_state(ompl_state_space) ;

                setOmplState(ompl_state_space, ompl_state, solutions[j]) ;

                addState(ompl_state) ;

                numStates ++ ;
            }
        }
    }

}

class OmplValidityChecker: public ompl::base::StateValidityChecker
{
public:

    OmplValidityChecker(const ompl::base::SpaceInformationPtr &si,
                        PlanningGroup *manip_): StateValidityChecker(si), manip(manip_),
        ompl_state_space(si->getStateSpace())
    {

    }

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space ;
    PlanningGroup *manip ;
};

bool OmplValidityChecker::isValid(const ompl::base::State *state) const
{
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    JointState js ;
    getOmplState(ompl_state_space, ompl_state, js) ;


    KinematicModel *model = manip->getModel() ;

    return model->isStateValid(js) ;

}

////////////////////////////////////////////////////////////////////////////////


bool JointSpacePlanner::solve(const std::vector<Matrix4x4> &poses,
                                 JointTrajectory &traj)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplStateSpace(pgroup) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OmplValidityChecker(si, pgroup)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    JointState js ;
    pgroup->getJointState(js) ;
    setOmplState(ompl_state_space, start_state, js) ;

    // set the goal region
    ompl::base::GoalPtr goal;

    Manipulator *manip = pgroup->manips[0] ;

    OmplIKRegion *region = new OmplIKRegion(si, poses, manip) ;
    goal.reset(region);
    if ( !region->hasStates() ) return false ;

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goal) ;

    // set the planner

    PlannerPtr ompl_planner ;

    switch ( alg )
    {

    case RRT:
        ompl_planner.reset(new ompl::geometric::RRT(si)) ;
        break ;
    case pRRT:
        ompl_planner.reset(new ompl::geometric::pRRT(si)) ;
        break ;
    case LazyRRT:
        ompl_planner.reset(new ompl::geometric::LazyRRT(si)) ;
        break ;
    case RRTConnect:
        ompl_planner.reset(new ompl::geometric::RRTConnect(si)) ;
        break ;
    case KPIECE:
        ompl_planner.reset(new ompl::geometric::KPIECE1(si)) ;
        break ;
    case BKPIECE:
        ompl_planner.reset(new ompl::geometric::BKPIECE1(si)) ;
        break ;
    case LBKPIECE:
        ompl_planner.reset(new ompl::geometric::LBKPIECE1(si)) ;
        break ;
    case SBL:
        ompl_planner.reset(new ompl::geometric::SBL(si)) ;
        break ;
    case pSBL:
        ompl_planner.reset(new ompl::geometric::pSBL(si)) ;
        break ;
    default:
        return false ;
    }


    //planner->setGoalBias(0.0) ;
    //planner->setRange(0.0) ;

    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out);

    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;

        // simplify path
        pathSimplifier->simplifyMax(path);

        getOmplTrajectory(path, ompl_state_space, traj) ;

        return true ;

    }

    return false ;

}

/////////////////////////////////////////////////////////////////////////////////////////////
extern void rpy_to_quat(double roll, double pitch, double yaw, double &qx, double &qy, double &qz, double &qw) ;
extern double NormalizeCircularAngle(double theta, double min_, double max_) ;

void poseToXYZRPY(const Matrix4x4 &pose, double &X, double &Y, double &Z, double &roll, double &pitch, double &yaw)
{
    Transform3D t(pose) ;

    RotationMatrix r = t.GetUpper3x3() ;
    Vector3 p = t.GetVector3Col() ;

    r.GetEulerAngles(yaw, pitch, roll, RotationMatrix::ZYX) ;

    roll = NormalizeCircularAngle(roll, -M_PI, M_PI) ;
    pitch = NormalizeCircularAngle(pitch, -M_PI, M_PI) ;
    yaw = NormalizeCircularAngle(yaw, -M_PI, M_PI) ;

    X = p.x ; Y = p.y ; Z = p.z ;


}


SimplePoseGoal::SimplePoseGoal(const Matrix4x4 &pose_, double small_): pose(pose_)
{
    goal_tolerance_x = goal_tolerance_y = goal_tolerance_z = small_ ;

    roll_delta_minus = roll_delta_plus = small_ ;
    yaw_delta_minus = yaw_delta_plus = small_ ;
    pitch_delta_minus = pitch_delta_plus = small_ ;
}

void SimplePoseGoal::computeBounds()
{
    double x, y, z, roll, pitch, yaw ;

    poseToXYZRPY(pose, x, y, z, roll, pitch, yaw) ;

    double xmin = x - fabs(goal_tolerance_x) ;
    double ymin = y - fabs(goal_tolerance_y) ;
    double zmin = z - fabs(goal_tolerance_z) ;
    double xmax = x + fabs(goal_tolerance_x) ;
    double ymax = y + fabs(goal_tolerance_y) ;
    double zmax = z + fabs(goal_tolerance_z) ;

    double roll_min = std::max(roll - fabs(roll_delta_minus), -M_PI) ;
    double roll_max = std::min(roll + fabs(roll_delta_plus), M_PI) ;

    double pitch_min = std::max(pitch - fabs(pitch_delta_minus), -M_PI) ;
    double pitch_max = std::min(pitch + fabs(pitch_delta_plus), M_PI) ;

    double yaw_min = std::max(yaw - fabs(yaw_delta_minus), -M_PI) ;
    double yaw_max = std::min(yaw + fabs(yaw_delta_plus), M_PI) ;

    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(ymin) ;
    lower_bounds.push_back(zmin) ;
    lower_bounds.push_back(roll_min) ;
    lower_bounds.push_back(pitch_min) ;
    lower_bounds.push_back(yaw_min) ;

    upper_bounds.push_back(xmax) ;
    upper_bounds.push_back(ymax) ;
    upper_bounds.push_back(zmax) ;
    upper_bounds.push_back(roll_max) ;
    upper_bounds.push_back(pitch_max) ;
    upper_bounds.push_back(yaw_max) ;
}

ompl::RNG g_rng ;

void SimplePoseGoal::sample(vector<double> &xyz_rpy)
{
    if ( lower_bounds.empty() ) computeBounds() ;

    double X = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    double Y = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    double Z = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;
    double Roll  = g_rng.uniformReal(lower_bounds[3], upper_bounds[3]) ;
    double Pitch = g_rng.uniformReal(lower_bounds[4], upper_bounds[4]) ;
    double Yaw   = g_rng.uniformReal(lower_bounds[5], upper_bounds[5]) ;

    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(Roll) ;
    xyz_rpy.push_back(Pitch) ;
    xyz_rpy.push_back(Yaw) ;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

SimpleShapeRegion::SimpleShapeRegion(): GoalRegion()
{
    roll_min = yaw_min = pitch_min = -M_PI ;
    roll_max = yaw_max = pitch_max = M_PI ;
}

void SimpleShapeRegion::computeOrientationBounds()
{
    roll_min = std::max(roll_min, -M_PI) ;
    roll_max = std::min(roll_max, M_PI) ;

    pitch_min = std::max(pitch_min, -M_PI) ;
    pitch_max = std::min(pitch_max, M_PI) ;

    yaw_min = std::max(yaw_min, -M_PI) ;
    yaw_max = std::min(yaw_max, M_PI) ;


    lower_bounds.push_back(roll_min) ;
    lower_bounds.push_back(pitch_min) ;
    lower_bounds.push_back(yaw_min) ;

    upper_bounds.push_back(roll_max) ;
    upper_bounds.push_back(pitch_max) ;
    upper_bounds.push_back(yaw_max) ;
}

void SimpleShapeRegion::computeBounds()
{
    computePositionBounds();
    computeOrientationBounds();
}

void SimpleShapeRegion::setShapePose(const Vector3 &orig, const Vector3 &rpy)
{
    double qx, qy, qz, qw ;

    rpy_to_quat(rpy.x, rpy.y, rpy.z, qx, qy, qz, qw) ;

    Rotation3D rot(qx, qy, qz, qw) ;
    Translation3D trans(orig.x, orig.y, orig.z) ;

    t = trans * rot ;
}

void SimpleShapeRegion::sample(vector<double> &xyz_rpy)
{
    if ( lower_bounds.empty() ) computeBounds() ;

    double X_, Y_, Z_ ;
    samplePosition(X_, Y_, Z_) ;

    Vector4 pt =   t * Vector4(X_, Y_, Z_, 1);

    double X = pt.x ;
    double Y = pt.y ;
    double Z = pt.z ;

    double Roll  = g_rng.uniformReal(lower_bounds[3], upper_bounds[3]) ;
    double Pitch = g_rng.uniformReal(lower_bounds[4], upper_bounds[4]) ;
    double Yaw   = g_rng.uniformReal(lower_bounds[5], upper_bounds[5]) ;

    xyz_rpy.push_back(X) ;
    xyz_rpy.push_back(Y) ;
    xyz_rpy.push_back(Z) ;
    xyz_rpy.push_back(Roll) ;
    xyz_rpy.push_back(Pitch) ;
    xyz_rpy.push_back(Yaw) ;


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoxShapedRegion::BoxShapedRegion(const Vector3 &c_, const Vector3 &sz_, const Vector3 &rpy_): SimpleShapeRegion(), orig(c_), rpy(rpy_), sz(sz_)
{
    setShapePose(orig, rpy) ;
}

void BoxShapedRegion::computePositionBounds()
{
    double xmin = -fabs(sz.x) ;
    double ymin = -fabs(sz.y) ;
    double zmin = -fabs(sz.z) ;

    double xmax = fabs(sz.x) ;
    double ymax = fabs(sz.y) ;
    double zmax = fabs(sz.z) ;

    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(xmin) ;
    lower_bounds.push_back(zmin) ;

    upper_bounds.push_back(xmax) ;
    upper_bounds.push_back(ymax) ;
    upper_bounds.push_back(zmax) ;
}

void BoxShapedRegion::samplePosition(double &X, double &Y, double &Z)
{
    X = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    Y = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    Z = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CylinderShapedRegion::CylinderShapedRegion(double length, double radius, const Vector3 &c_, const Vector3 &rpy_): SimpleShapeRegion(), orig(c_), rpy(rpy_), rad(radius), len(length)
{
    setShapePose(orig, rpy) ;
}

void CylinderShapedRegion::computePositionBounds()
{
    double lmin = -len/2.0 ;
    double tmin = -M_PI ;
    double rmin = 0 ;

    double lmax = len/2.0 ;
    double tmax = M_PI ;
    double rmax = rad ;

    lower_bounds.push_back(lmin) ;
    lower_bounds.push_back(tmin) ;
    lower_bounds.push_back(rmin) ;

    upper_bounds.push_back(lmax) ;
    upper_bounds.push_back(tmax) ;
    upper_bounds.push_back(rmax) ;
}

void CylinderShapedRegion::samplePosition(double &X, double &Y, double &Z)
{
    double l = g_rng.uniformReal(lower_bounds[0], upper_bounds[0]) ;
    double t = g_rng.uniformReal(lower_bounds[1], upper_bounds[1]) ;
    double r = g_rng.uniformReal(lower_bounds[2], upper_bounds[2]) ;

    Z = l ;
    X = r * cos(t) ;
    Y = r * sin(t) ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


RPY_XYZ_TaskSpace::RPY_XYZ_TaskSpace(): TaskSpace()
{
    upper[0] = 2 ; lower[0] = -2 ;
    upper[1] = 2 ; lower[1] = -2 ;
    upper[2] = 2 ; lower[2] = -2 ;

    upper[3] = M_PI ; lower[3] = -M_PI ;
    upper[4] = M_PI ; lower[4] = -M_PI ;
    upper[5] = M_PI ; lower[5] = -M_PI ;
}

int RPY_XYZ_TaskSpace::getDimension() const  { return 6 ; }

double RPY_XYZ_TaskSpace::getUpperLimit(int dim) const { return upper[dim] ; }
double RPY_XYZ_TaskSpace::getLowerLimit(int dim) const { return lower[dim] ; }

void RPY_XYZ_TaskSpace::taskSpaceToPose(const std::vector<double> &state, vector<Matrix4x4> &pose) const
{
    double x = state[0] ;
    double y = state[1] ;
    double z = state[2] ;

    double roll = state[3] ;
    double pitch = state[4] ;
    double yaw = state[5] ;

    double qx, qy, qz, qw ;

    rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

    Rotation3D rot(qx, qy, qz, qw) ;
    Translation3D trans(x, y, z) ;

    pose.push_back(trans * rot) ;
}

// transform pose of the end-effector to state in task space
void RPY_XYZ_TaskSpace::poseToTaskSpace(const vector<Matrix4x4> &pose, std::vector<double> &state) const
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose[0], X, Y, Z, roll, pitch, yaw) ;

    state.push_back(X) ;
    state.push_back(Y) ;
    state.push_back(Z) ;

    state.push_back(roll) ;
    state.push_back(pitch) ;
    state.push_back(yaw) ;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Matrix3x3 getLookAtMatrix(const Vector3 &n)
{
    Vector3 n0, na, nb ;

    n0 = n ;

    n0.normalise();

    na.x = 1 - n0.x*n0.x ;
    na.y = n0.x * n0.y ;
    na.z = n0.x * n0.z ;

    na.normalise() ;

    nb = cross(n0, na) ;
    nb.normalise() ;

    return Matrix3x3(na, nb, n0) ;
}


MoveTo_TaskSpace::MoveTo_TaskSpace(const Matrix4x4 &pose, const Vector3 &dp, double pos_tol,
                     const Vector3 &rpy_minus, const Vector3 &rpy_plus)
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose, X, Y, Z, roll, pitch, yaw) ;

    c0 = Vector3(X, Y, Z) ;
    a0 = Vector3(roll, pitch, yaw) ;

    Vector3 c1 = c0 + dp ;

    // We define a coordinate system with the Z axis pointing towards the target point

    Matrix3x3 r = getLookAtMatrix(dp) ;

    frame = r  ;
    iframe = r.Inv() ;

    // we use a cylinder parameterization of the position

    lower[0] = 0.0      ; upper[0] = dp.length() ; // cylinder length
    lower[1] = -M_PI    ; upper[1] = M_PI ; // polar angle
    lower[2] = 0.0      ; upper[2] = pos_tol ; // radius

    const double small_ = 0.001 ;

    double roll_min = std::max(a0.x - fabs(rpy_minus.x) - small_, -M_PI) ;
    double roll_max = std::min(a0.x + fabs(rpy_plus.x) + small_, M_PI) ;

    double pitch_min = std::max(a0.y - fabs(rpy_minus.y) - small_, -M_PI) ;
    double pitch_max = std::min(a0.y + fabs(rpy_plus.y) + small_, M_PI) ;

    double yaw_min = std::max(a0.z - fabs(rpy_minus.z) - small_, -M_PI) ;
    double yaw_max = std::min(a0.z + fabs(rpy_plus.z) + small_, M_PI) ;

    upper[3] = roll_max     ; lower[3] = roll_min ;
    upper[4] = pitch_max    ; lower[4] = pitch_min ;
    upper[5] = yaw_max      ; lower[5] = yaw_min ;

}



void MoveTo_TaskSpace::taskSpaceToPose(const std::vector<double> &state, vector<Matrix4x4> &pose) const
{
    double l = state[0] ;
    double theta = state[1] ;
    double r = state[2] ;

    double x = r * cos(theta) ;
    double y = r * sin(theta) ;
    double z = l  ;

    Vector3 pt = frame * Vector3(x, y, z)  + c0 ;

    double roll = state[3] ;
    double pitch = state[4] ;
    double yaw = state[5] ;

    double qx, qy, qz, qw ;

    rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

    Rotation3D rot(qx, qy, qz, qw) ;
    Translation3D trans(pt) ;

    pose.push_back( trans * rot ) ;

}

void MoveTo_TaskSpace::poseToTaskSpace(const vector<Matrix4x4> &pose, std::vector<double> &state) const
{
    double X, Y, Z, roll, pitch, yaw ;

    poseToXYZRPY(pose[0], X, Y, Z, roll, pitch, yaw) ;

    Vector3 pt = iframe * ( Vector3(X, Y, Z) - c0 )  ;

    double l = pt.z  ;
    double t = atan2(pt.y, pt.x) ;
    double r = sqrt(pt.x * pt.x + pt.y * pt.y) ;

    state.push_back(l) ;
    state.push_back(t) ;
    state.push_back(r) ;

    state.push_back(roll) ;
    state.push_back(pitch) ;
    state.push_back(yaw) ;

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


class OmplTaskValidityChecker: public ompl::base::StateValidityChecker
{
public:

    OmplTaskValidityChecker(const ompl::base::SpaceInformationPtr &si,
                            PlanningGroup *manip_, const TaskSpace *ts_): StateValidityChecker(si), manip(manip_), ts(ts_),
        ompl_state_space(si->getStateSpace())
    {

    }

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space ;
    PlanningGroup *manip ;
    const TaskSpace *ts ;
};

bool OmplTaskValidityChecker::isValid(const ompl::base::State *state) const
{
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    std::vector<double> tsv ;

    getOmplTaskState(ompl_state_space, ompl_state, tsv) ;

    vector<Matrix4x4> pose ;
    ts->taskSpaceToPose(tsv, pose) ;

    KinematicModel *model = manip->getModel() ;

    vector<JointState> ik_solutions ;
    if ( ! manip->solve(pose, ik_solutions) ) return false ;

    // now for each solution find one that is valid with respect to constraints

    for( int i=0 ; i<ik_solutions.size() ; i++ )
    {
        if ( model->isStateValid(ik_solutions[i])) {

            return true ;
        }
    }

    // non found

    return false ;
}


class OmplTaskGoalSampler {
    public:

    OmplTaskGoalSampler(const ompl::base::SpaceInformationPtr &space_information,
                   const ompl::base::ProblemDefinitionPtr &pd,
                   GoalRegion *goal,
                   const TaskSpace *ts,
                   const PlanningGroup *manip) ;

    bool sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state) ;
    void randomSample(std::vector<double> &sample) ;

    int sampleNum ;
    const PlanningGroup *manip ;
    const TaskSpace *ts ;
    GoalRegion *goal ;

    const ompl::base::SpaceInformationPtr &si ;
    const ompl::base::ProblemDefinitionPtr &pd ;
    int max_sample_count_ ;



};

OmplTaskGoalSampler::OmplTaskGoalSampler(const ompl::base::SpaceInformationPtr &space_information_,
                           const ompl::base::ProblemDefinitionPtr &pd_,
                           GoalRegion *goal_,
                           const TaskSpace *ts_,
                           const PlanningGroup *manip_): manip(manip_), goal(goal_),
    ts(ts_), si(space_information_), sampleNum(0), max_sample_count_(100), pd(pd_) { }

void OmplTaskGoalSampler::randomSample(std::vector<double> &sample)
{
    vector<double> sv ;
    vector<Matrix4x4> pose ;

    goal->sample(sv) ;

    for( int i=0, k=0; i<manip->manips.size() ; i++ )
    {
        double qx, qy, qz, qw ;
        double x = sv[k++], y = sv[k++], z = sv[k++], roll = sv[k++], pitch = sv[k++], yaw = sv[k++] ;

        rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

        Rotation3D rot(qx, qy, qz, qw) ;
        Translation3D trans(x, y, z) ;

        pose.push_back( trans * rot ) ;
    }

    ts->poseToTaskSpace(pose, sample) ;

}

bool OmplTaskGoalSampler::sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state)
{
    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = si->getStateSpace() ;

    while (sampleNum < max_sample_count_)
    {
        std::vector<double> sample ;

        randomSample(sample) ;

        sampleNum ++ ;

        double *vals = static_cast<RealVectorStateSpace::StateType*>(state)->values ;

        for(int i=0 ; i<sample.size() ; i++ ) vals[i] = sample[i] ;

        RealVectorStateSpace *state_space =
                dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

        if ( !ompl_state_space->satisfiesBounds(state) ) continue ;
        else break ;

   }
    cout << sampleNum << endl ;
    return sampleNum < max_sample_count_ && !pd->hasSolution();
    return false ;

}

bool TaskSpacePlanner::solve(GoalRegion &goal_, const TaskSpace *ts, JointTrajectory &traj)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplTaskStateSpace(ts) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OmplTaskValidityChecker(si, pgroup, ts)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    vector<Matrix4x4> pose ;

    for( int i=0 ; i<pgroup->manips.size() ; i++ )
    {
        std::string ee = pgroup->manips[i]->getEndEffector() ;
        Matrix4x4 pose_ = pgroup->getModel()->getWorldTransform(ee) ;
        pose.push_back(pose_) ;
    }

    std::vector<double> start_state_vec ;
    ts->poseToTaskSpace(pose, start_state_vec) ;

    setOmplTaskState(ompl_state_space, start_state, start_state_vec) ;

    // set the goal region
    ompl::base::GoalPtr goal;

    OmplTaskGoalSampler goal_sampler(si, ompl_planner_setup->getProblemDefinition(), &goal_, ts, pgroup) ;

    goal.reset(new ompl::base::GoalLazySamples(si, boost::bind(&OmplTaskGoalSampler::sampleGoal, &goal_sampler,_1,_2)));

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goal) ;

    // set the planner

    PlannerPtr ompl_planner ;

    switch ( alg )
    {

    case RRT:
        ompl_planner.reset(new ompl::geometric::RRT(si)) ;
        break ;
    case pRRT:
        ompl_planner.reset(new ompl::geometric::pRRT(si)) ;
        break ;
    case LazyRRT:
        ompl_planner.reset(new ompl::geometric::LazyRRT(si)) ;
        break ;
    case RRTConnect:
        ompl_planner.reset(new ompl::geometric::RRTConnect(si)) ;
        break ;
    case KPIECE:
        ompl_planner.reset(new ompl::geometric::KPIECE1(si)) ;
        break ;
    case BKPIECE:
        ompl_planner.reset(new ompl::geometric::BKPIECE1(si)) ;
        break ;
    case LBKPIECE:
        ompl_planner.reset(new ompl::geometric::LBKPIECE1(si)) ;
        break ;
    case SBL:
        ompl_planner.reset(new ompl::geometric::SBL(si)) ;
        break ;
    case pSBL:
        ompl_planner.reset(new ompl::geometric::pSBL(si)) ;
        break ;
    default:
        return false ;
    }


    //planner->setGoalBias(0.0) ;
    //planner->setRange(0.0) ;

    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out);

    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;

        // simplify path
        pathSimplifier->simplifyMax(path);

        return getOmplTaskTrajectory(path, ompl_state_space, ts, pgroup, traj) ;
    }

    return false ;
}


//////////////////////////////////////////////////////////////////////////////////////////////////


bool move_arm_tip(Manipulator *manip, const Vector3 &dp, JointTrajectory &traj,
                  double goal_tolerance_z, double goal_tolerance_rad, double motion_tolerance,
                  const Vector3 &rpy_minus, const Vector3 &rpy_plus)
{

    Matrix4x4 cp = manip->getModel()->getWorldTransform(manip->getEndEffector()) ;

    double X, Y, Z, roll, pitch, yaw ;
    poseToXYZRPY(cp, X, Y, Z, roll, pitch, yaw);

    Vector3 c0(X, Y, Z) ;
    Vector3 c1 = c0 + dp ;

    // setup the goal region

    Matrix3x3 r = getLookAtMatrix(dp) ;

    Rotation3D rot(r) ;

    double X_, Y_, Z_, croll_, cpitch_, cyaw_ ;
    poseToXYZRPY(rot, X_, Y_, Z_, croll_, cpitch_, cyaw_) ;

    CylinderShapedRegion goal(2*goal_tolerance_z, goal_tolerance_rad, c1, Vector3(croll_, cpitch_, cyaw_)) ;

  //BoxShapedRegion goal( c1, Vector3(goal_tolerance_z, goal_tolerance_z, goal_tolerance_z), Vector3()) ;

    const double small_ = 0.001 ;

    goal.roll_min = std::max(roll - fabs(rpy_minus.x) - small_, -M_PI) ;
    goal.roll_max = std::min(roll + fabs(rpy_plus.x) + small_, M_PI) ;

    goal.pitch_min = std::max(pitch - fabs(rpy_minus.y) - small_, -M_PI) ;
    goal.pitch_max = std::min(pitch + fabs(rpy_plus.y) + small_, M_PI) ;

    goal.yaw_min = std::max(yaw - fabs(rpy_minus.z) - small_, -M_PI) ;
    goal.yaw_max = std::min(yaw + fabs(rpy_plus.z) + small_, M_PI) ;


    // setup the taskspace

    MoveTo_TaskSpace ts(cp, dp, motion_tolerance, rpy_minus, rpy_plus) ;

    PlanningGroup group ;
    group.addManipulator(manip) ;

    TaskSpacePlanner planner(&group) ;

    return planner.solve(goal, &ts, traj) ;

}


#if 0



bool move_arm_tip(Manipulator *manip, double dx, double dy, double dz, JointTrajectory &traj,
                  double goal_tolerance_x, double goal_tolerance_y, double goal_tolerance_z,
                  double roll_delta_minus, double roll_delta_plus,
                  double pitch_delta_minus, double pitch_delta_plus,
                  double yaw_delta_minus, double yaw_delta_plus
                  )
{
    std::string tip = manip->getEndEffector() ;
    Matrix4x4 pose = manip->getModel()->getWorldTransform(tip) ;

    double X, Y, Z, Roll, Pitch, Yaw ;

    const double small_ = 1.0e-5 ;

    poseToXYZRPY(pose, X, Y, Z, Roll, Pitch, Yaw) ;

    double xmin = std::min(X, X+dx) ;
    double xmax = std::max(X, X+dx) ;
    double ymin = std::min(Y, Y+dy) ;
    double ymax = std::max(Y, Y+dy) ;
    double zmin = std::min(Z, Z+dz) ;
    double zmax = std::max(Z, Z+dz) ;

    xmin = std::min(xmin, xmin-goal_tolerance_x) ;
    ymin = std::min(ymin, ymin-goal_tolerance_y) ;
    zmin = std::min(zmin, zmin-goal_tolerance_z) ;
    xmax = std::max(xmax, xmax+goal_tolerance_x) ;
    ymax = std::max(ymax, ymax+goal_tolerance_y) ;
    zmax = std::max(zmax, zmax+goal_tolerance_z) ;

    double roll_min = std::max(Roll - roll_delta_minus - small_, -M_PI) ;
    double roll_max = std::min(Roll + roll_delta_plus + small_, M_PI) ;

    double pitch_min = std::max(Pitch - pitch_delta_minus - small_, -M_PI) ;
    double pitch_max = std::min(Pitch + pitch_delta_plus + small_, M_PI) ;

    double yaw_min = std::max(Yaw - pitch_delta_minus - small_, -M_PI) ;
    double yaw_max = std::min(Yaw + pitch_delta_plus + small_, M_PI) ;

    MoveTaskSpace ts(xmin, ymin, zmin, xmax, ymax, zmax, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max) ;

    TaskSpacePlanner planner(manip) ;

    std::vector<double> goal, goal_tolerance ;

    goal.push_back(X + dx) ;
    goal.push_back(Y + dy) ;
    goal.push_back(Z + dz) ;
    goal.push_back(Roll) ;
    goal.push_back(Pitch) ;
    goal.push_back(Yaw) ;

    goal_tolerance.push_back(goal_tolerance_x) ;
    goal_tolerance.push_back(goal_tolerance_y) ;
    goal_tolerance.push_back(goal_tolerance_z) ;
    goal_tolerance.push_back(std::max(roll_delta_minus, roll_delta_plus)) ;
    goal_tolerance.push_back(std::max(roll_delta_minus, roll_delta_plus)) ;
    goal_tolerance.push_back(std::max(roll_delta_minus, roll_delta_plus)) ;

    planner.setAlgorithm(PlannerBase::RRT) ;
    return planner.solve(goal,  &ts, traj) ;
}
#endif
////////////////////////////////////////////////////

#if 0
// We plan on the 3D task space of a cylinder with height (max_height - min_height) and radius tol

class CylinderTaskSpace: public TaskSpace
{
public:
    CylinderTaskSpace(const Matrix4x4 &orig, double z_min_, double z_max_, double r_min_,
                  double r_max_, double theta_min_, double theta_max_,
                      double roll_min_, double roll_max_,
                      double pitch_min_, double pitch_max_,
                      double yaw_min_, double yaw_max_
                      ): frame(orig), TaskSpace() {

        upper[0] = z_max_ ;     lower[0] = z_min_ ;
        upper[1] = theta_max_ ; lower[1] = theta_min_ ;
        upper[2] = r_max_ ;     lower[2] = r_min_ ;

        upper[3] = roll_max_ ; lower[3] = roll_min_ ;
        upper[4] = pitch_max_ ; lower[4] = pitch_min_ ;
        upper[5] = yaw_max_ ;  lower[5] = yaw_min_ ;
    }

    virtual int getDimension() const  { return 6 ; }

    virtual double getUpperLimit(int dim) const { return upper[dim] ; }
    virtual double getLowerLimit(int dim) const { return lower[dim] ; }

    virtual void taskSpaceToPose(const std::vector<double> &state, Matrix4x4 &pose) const
    {
        double z = state[0] ;
        double theta = state[1] ;
        double r = state[2] ;

        double x = r * cos(theta) ;
        double y = r * sin(theta) ;

        double roll = state[3] ;
        double pitch = state[4] ;
        double yaw = state[5] ;

        double qx, qy, qz, qw ;

        rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw) ;

        Rotation3D rot(qx, qy, qz, qw) ;
        Translation3D trans(x, y, z) ;

        pose = frame * trans * rot ;
    }

    // transform pose of the end-effector to state in task space
    virtual void poseToTaskSpace(const Matrix4x4 &pose, std::vector<double> &state) const
    {
        double X, Y, Z, roll, pitch, yaw ;

        poseToXYZRPY(pose, X, Y, Z, roll, pitch, yaw) ;

        Vector4 pp = Vector3(X, Y, Z) * frame ;

        double r = sqrt(pp.x* pp.x + pp.y*pp.y) ;
        double theta = atan2(pp.y, pp.x) ;

        state.push_back(Z) ;
        state.push_back(theta) ;
        state.push_back(r) ;

        state.push_back(roll) ;
        state.push_back(pitch) ;
        state.push_back(yaw) ;
    }


    double lower[6], upper[6] ;
    Matrix4x4 frame ;
};

bool move_tip_above_table(Manipulator *manip, const Matrix4x4 &c, double height, double height_tol, double xy_tol, JointTrajectory &traj)
{


    CylinderTaskSpace ts(c, height - height_tol, height + height_tol, 0, xy_tol, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI) ;

    TaskSpacePlanner planner(manip) ;

    std::vector<double> goal, goal_tolerance ;

    goal.push_back(height) ;
    goal.push_back(0.0) ;
    goal.push_back(0.0) ;
    goal.push_back(0) ;
    goal.push_back(0) ;
    goal.push_back(0) ;

    goal_tolerance.push_back(height_tol) ;
    goal_tolerance.push_back(M_PI) ;
    goal_tolerance.push_back(xy_tol) ;
    goal_tolerance.push_back(M_PI) ;
    goal_tolerance.push_back(M_PI) ;
    goal_tolerance.push_back(M_PI) ;

    planner.setAlgorithm(PlannerBase::RRT) ;
    return planner.solve(goal, goal_tolerance, &ts, traj) ;

}





#endif








////////////////////////////////////////////////////////////////////////////
double JointTrajectory::lerp(const std::string &joint, double t) const
{
    //assert ( t >= 0.0  && t <= 1.0 ) ;

    t = std::max(0.0, std::min(1.0, t)) ;

    assert ( times.size() > 1 ) ;

    int idx = 0 ;
    for( ; idx < times.size() ; idx++ )
    {
        if ( times[idx+1] >= t ) break ;
    }

    const JointState &state1 = positions[idx] ;
    const JointState &state2 = positions[idx+1] ;

    map<string, JointState::DOF>::const_iterator dof1 = state1.data.find(joint),
            dof2 = state2.data.find(joint) ;

    assert( dof1 != state1.data.end() && dof2 != state2.data.end() ) ;

    return  (1 -t) * (*dof1).second.val + t * (*dof2).second.val ;

}
OMPLPlanner::OMPLPlanner()
{

}
#endif

