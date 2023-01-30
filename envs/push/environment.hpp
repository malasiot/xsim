#pragma once

#include "world.hpp"

#include <opencv2/opencv.hpp>

struct PushAction {
    size_t box_id_ ;
    size_t loc_ ; // 3 push locations per side
};

struct BoxState {
    float cx_, cy_, theta_ ;
    bool moved_ = false ;
    std::string name_ ;
};

enum StateType {
    STATE_INITIAL, STATE_TARGET_MOVED, STATE_NON_TARGET_MOVED, STATE_OUTSIDE_OF_WORKSPACE, STATE_OBJECTS_TURNED, STATE_TARGET_REACHED, STATE_MAX_MOVES_REACHED, STATE_UNREACHABLE
};

struct State {
    std::vector<BoxState> boxes_ ;
    StateType type_ = STATE_INITIAL ;
    float movement_ ; // relative proximity between target box and target position
    bool target_moved_ = false ;
    bool non_target_moved_ = false ;

    const BoxState *box(const std::string &name) const {
        auto it = std::find_if(boxes_.begin(), boxes_.end(), [name](const BoxState &bs) { return bs.name_ == name ;}) ;
        return ( it == boxes_.end() ) ? nullptr : &(*it) ;
    }

    bool isTerminal() const {
        return ( type_ == STATE_TARGET_REACHED || type_ == STATE_OUTSIDE_OF_WORKSPACE) || (type_ == STATE_OBJECTS_TURNED) || (type_ == STATE_TARGET_REACHED) ||
                ( type_ == STATE_MAX_MOVES_REACHED ) ;
    }

    friend std::ostream &operator<<(std::ostream &strm, const State &state) ;
};

class Environment {
public:
    struct Parameters {
        Parameters(const cvx::Variant &config) ;
        float motion_start_offset_ = 0.05 ;
        float motion_push_offset_ = 0.05 ;
        Eigen::Vector2f target_pos_{0.0, 0.2} ;
        float target_radius_ = 0.05 ;
        std::string target_ = "box_0_0";
        int32_t max_trials_ = 100 ;
    };

    Environment(const Parameters &params, World *world);

    void reset() ;

    // perform action and return new state, reward and done flaf
    std::tuple<State, float, bool> transition(const State &state, int64_t action_index) ;

    // check if action is feasible for the current state
    bool isFeasible(const State &state, int64_t action_idx) ;

    State getState() const ;
    cv::Mat renderState(const PushAction &a, const State &state) ;

    std::vector<std::string> getBoxNames() const { return box_names_ ; }

    World *world() { return world_ ; }

    int64_t stateSpaceDim() const {
        return box_names_.size() * 3 ;
    }

    int64_t numActions() const {
        return actions_.size() ;
    }


    Eigen::VectorXf stateToTensor(const State &state) const ;

protected:

    void runSim(float t) ;
    float computeReward(const State &state) ;
    void checkMotion(const State &old_state, State &new_state);

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeMotion(const Eigen::Vector2f &c, float theta, int action_id) ;
    World *world_ ;
    int32_t trial_ = 0 ;
    int32_t target_idx_ ;
    Parameters params_ ;
    std::vector<PushAction> actions_ ;
    std::vector<std::string> box_names_ ;

};
