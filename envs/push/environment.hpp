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
    STATE_VALID = 0, STATE_OUTSIDE_OF_WORKSPACE = 1, STATE_OBJECTS_TURNED = 2, STATE_TARGET_REACHED = 3, STATE_MAX_MOVES_REACHED = 4
};

struct State {
    std::map<std::string, BoxState> boxes_ ;
    StateType type_ = STATE_VALID ;
};

class Environment {
public:
    Environment(World *world);

    void reset() ;

    std::vector<PushAction> getActions() const ;

    float apply(const State &state, const PushAction &action) ;

    State getState() const ;
    cv::Mat renderState(const PushAction &a, const State &state) ;

    std::vector<std::string> getBoxNames() const ;

protected:

    void runSim(float t) ;

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeMotion(const Eigen::Vector2f &c, float theta, int action_id) ;
    World *world_ ;
};
