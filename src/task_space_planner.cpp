#include <xsim/task_space_planner.hpp>

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

ompl::base::StateSpacePtr createOmplTaskStateSpace(const TaskSpace *ts) {
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space;

    RealVectorBounds real_vector_bounds(0) ;

    int dim = ts->getDimension() ;

    RealVectorStateSpace *state_space = new RealVectorStateSpace(ts->getDimension()) ;

    for( int i=0 ; i<dim ; i++ ) {

        double lower = ts->getLowerLimit(i) ;
        double upper = ts->getUpperLimit(i) ;

        real_vector_bounds.low.push_back(lower);
        real_vector_bounds.high.push_back(upper);
    }

    state_space->setBounds(real_vector_bounds);
    ompl_state_space.reset(state_space);

    return ompl_state_space ;
}


class OmplTaskValidityChecker: public ompl::base::StateValidityChecker
{
public:

    OmplTaskValidityChecker(const ompl::base::SpaceInformationPtr &si,
                            PlanningInterface *manip, const TaskSpace *ts): StateValidityChecker(si), manip_(manip), ts_(ts),
        ompl_state_space(si->getStateSpace())
    {

    }

    bool isValid(const ompl::base::State *state) const ;

private:

    const ompl::base::StateSpacePtr &ompl_state_space ;
    PlanningInterface *manip_ ;
    const TaskSpace *ts_ ;
};


void getOmplTaskState(const ompl::base::StateSpacePtr &ompl_state_space,
             const ompl::base::RealVectorStateSpace::StateType *ompl_state,
             std::vector<double> &state)
{
    using namespace ompl::base ;

    RealVectorStateSpace *state_space =
            dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

    for( int i=0 ; i<state_space->getDimension() ; i++ ) {
        state.push_back((*ompl_state)[i]) ;
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

bool getOmplTaskTrajectory(const ompl::geometric::PathGeometric &path,
                       const ompl::base::StateSpacePtr &ompl_state_space,
                       const TaskSpace *ts,
                       const PlanningInterface *group,
                       JointTrajectory &joint_trajectory) {
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
        Isometry3f pose ;
        ts->taskSpaceToPose(tsv, pose) ;

        if ( i==0 )
        {
            vector<JointState> solutions ;
            if ( !group->solveIK(pose, solutions) )
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

            if ( !group->solveIK(pose, previous_state, solution) ) {
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


bool OmplTaskValidityChecker::isValid(const ompl::base::State *state) const
{
    using namespace ompl::base ;

    const RealVectorStateSpace::StateType *ompl_state =
            dynamic_cast<const RealVectorStateSpace::StateType *>(state) ;

    std::vector<double> tsv ;

    getOmplTaskState(ompl_state_space, ompl_state, tsv) ;

    Isometry3f pose ;
    ts_->taskSpaceToPose(tsv, pose) ;

    vector<JointState> ik_solutions ;
    if ( ! manip_->solveIK(pose, ik_solutions) ) return false ;

    // now for each solution find one that is valid with respect to constraints

    for( int i=0 ; i<ik_solutions.size() ; i++ )  {
        if ( manip_->isStateValid(ik_solutions[i]) ) {
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
                   const GoalRegion *goal,
                   const TaskSpace *ts,
                   const PlanningInterface *manip) ;

    bool sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state) ;
    void randomSample(std::vector<double> &sample) ;

    int sample_num_ = 0;
    const PlanningInterface *manip_ ;
    const TaskSpace *ts_ ;
    const GoalRegion *goal_ ;

    const ompl::base::SpaceInformationPtr &si_ ;
    const ompl::base::ProblemDefinitionPtr &pd_ ;
    int max_sample_count_ = 15;



};

OmplTaskGoalSampler::OmplTaskGoalSampler(const ompl::base::SpaceInformationPtr &space_information,
                           const ompl::base::ProblemDefinitionPtr &pd,
                           const GoalRegion *goal,
                           const TaskSpace *ts,
                           const PlanningInterface *manip): manip_(manip), goal_(goal),
    ts_(ts), si_(space_information), pd_(pd) { }

void OmplTaskGoalSampler::randomSample(std::vector<double> &sample)
{
    vector<double> sv ;
    Isometry3f pose ;
    pose.setIdentity() ;

    goal_->sample(sv) ;

    Quaternionf q = AngleAxisf(sv[3], Vector3f::UnitX())
            * AngleAxisf(sv[4], Vector3f::UnitY())
            * AngleAxisf(sv[5], Vector3f::UnitZ());

    pose.linear() = q.toRotationMatrix() ;
    pose.translation() = Vector3f(sv[0], sv[1], sv[2]) ;

    ts_->poseToTaskSpace(pose, sample) ;

}

bool OmplTaskGoalSampler::sampleGoal(const ompl::base::GoalLazySamples *gls, ompl::base::State *state)
{
    using namespace ompl::base ;

    const StateSpacePtr &ompl_state_space = si_->getStateSpace() ;

    while (sample_num_ < max_sample_count_) {
        std::vector<double> sample ;

        randomSample(sample) ;

        sample_num_ ++ ;

        double *vals = static_cast<RealVectorStateSpace::StateType*>(state)->values ;

        for(int i=0 ; i<sample.size() ; i++ ) vals[i] = sample[i] ;

        RealVectorStateSpace *state_space =
                dynamic_cast<RealVectorStateSpace *>(ompl_state_space.get()) ;

        if ( !ompl_state_space->satisfiesBounds(state) ) continue ;
        else break ;

   }

    return sample_num_ < max_sample_count_ && !pd_->hasSolution();
}

bool TaskSpacePlanner::solve(const GoalRegion &goal, const TaskSpace &ts, JointTrajectory &traj)
{
    using namespace ompl::base ;

    StateSpacePtr ompl_state_space = createOmplTaskStateSpace(&ts) ;

    // create a simple setup and set start and goal state
    ompl::geometric::SimpleSetupPtr ompl_planner_setup ;

    ompl_planner_setup.reset(new ompl::geometric::SimpleSetup(ompl_state_space)) ;

    SpaceInformationPtr si = ompl_planner_setup->getSpaceInformation() ;

    si->setStateValidityChecker(StateValidityCheckerPtr(new OmplTaskValidityChecker(si, iplan_, &ts)));

    // use the current joint state of the manipulator as the start state

    ScopedState<RealVectorStateSpace> start_state(ompl_state_space) ;

    Isometry3f pose = iplan_->getToolPose() ;

    std::vector<double> start_state_vec ;
    ts.poseToTaskSpace(pose, start_state_vec) ;

    setOmplTaskState(ompl_state_space, start_state, start_state_vec) ;

    bool r =  si->isValid(start_state.get());

    // set the goal region
    ompl::base::GoalPtr goalr;

    OmplTaskGoalSampler goal_sampler(si, ompl_planner_setup->getProblemDefinition(), &goal, &ts, iplan_) ;

    goalr.reset(new ompl::base::GoalLazySamples(si, std::bind(&OmplTaskGoalSampler::sampleGoal, &goal_sampler,
                                                              std::placeholders::_1, std::placeholders::_2)));

    ompl_planner_setup->setStartState(start_state);
    ompl_planner_setup->setGoal(goalr) ;

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
    case RRTConnect: {
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



    ompl_planner_setup->setPlanner(ompl_planner) ;

    // solve problem

    PlannerStatus res = ompl_planner_setup->solve(time_out_);

    if ( res  )
    {
        // get solution path

        ompl::geometric::PathGeometric &path = ompl_planner_setup->getSolutionPath() ;
        ompl::geometric::PathSimplifierPtr &pathSimplifier =
            ompl_planner_setup->getPathSimplifier() ;

        // simplify path
      //  pathSimplifier->simplifyMax(path);

        return getOmplTaskTrajectory(path, ompl_state_space, &ts, iplan_, traj) ;
    }

    return false ;
}
}
