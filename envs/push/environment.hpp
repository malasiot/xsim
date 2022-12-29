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

struct State {
    std::map<std::string, BoxState> boxes_ ;
};

class Environment {
public:
    Environment(World *world): world_(world) {}

    bool apply(const State &state, const PushAction &action) ;
    State getState() const ;
    cv::Mat renderState(const PushAction &a, const State &state) ;

    std::vector<std::string> getBoxNames() const ;

protected:

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeMotion(const Eigen::Vector2f &c, float theta, int action_id) ;
    World *world_ ;
};
