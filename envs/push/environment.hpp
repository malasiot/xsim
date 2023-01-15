#pragma once

#include "world.hpp"
#include <opencv2//opencv.hpp>

struct PushAction {
    std::string box_id_ ;
    uint loc_ ; // 3 push locations per side
};

struct BoxState {
    float cx_, cy_, theta_ ;
};

enum StateType {
    STATE_VALID, STATE_OUTSIDE_OF_WORKSPACE, STATE_OBJECTS_TURNED, STATE_TARGET_REACHED, STATE_MAX_MOVES_REACHED, STATE_UNREACHABLE
};

struct State {
    std::map<std::string, BoxState> boxes_ ;
    StateType type_ = STATE_VALID ;

    bool isTerminal() const {
        return (type_ == STATE_OUTSIDE_OF_WORKSPACE) || (type_ == STATE_OBJECTS_TURNED) || (type_ == STATE_TARGET_REACHED) ||
                ( type_ == STATE_MAX_MOVES_REACHED ) ;
    }
};

class Environment {
public:
    Environment(World *world);

    void setTargetBox(const std::string &target) ;

    void reset() ;

    std::vector<PushAction> getActions() const ;

    std::pair<State, float> transition(const State &state, const PushAction &action) ;

    State getState() const ;
    cv::Mat renderState(const PushAction &a, const State &state) ;

    std::vector<std::string> getBoxNames() const ;

    const xsim::JointTrajectory &lastTrajectory() const { return push_ ; }

protected:

    void runSim(float t) ;
    float computeReward(const State &state) ;

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeMotion(const Eigen::Vector2f &c, float theta, int action_id) ;
    World *world_ ;
    xsim::JointTrajectory push_ ;
    int32_t trial_ = 0, max_trials_ = 100 ;
    std::string target_ ;
};
