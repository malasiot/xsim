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

bool JointSpacePlanner::solve(const std::vector<Eigen::Isometry3f> &poses,
                                 JointTrajectory &traj) {
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplStateSpace(iplan_) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

  //  si->setStateValidityCheckingResolution(0.001);

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

