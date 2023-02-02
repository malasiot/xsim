#pragma once

#include <string>
#include <cvx/misc/variant.hpp>

#include "world.hpp"

class World ;

struct PushAction {
    size_t box_id_ ;
    size_t loc_ ; // 3 push locations per side
};

struct BoxState {
    float cx_, cy_, theta_ ;
    std::string name_ ;
};
enum StateType {
    STATE_INITIAL, STATE_TARGET_MOVED, STATE_NON_TARGET_MOVED, STATE_OUTSIDE_OF_WORKSPACE, STATE_OBJECTS_TURNED, STATE_TARGET_REACHED, STATE_MAX_MOVES_REACHED, STATE_UNREACHABLE
};

struct State {
    std::vector<BoxState> boxes_ ;
    StateType type_ = STATE_INITIAL ;

    bool isTerminal() const {
        return ( type_ == STATE_TARGET_REACHED || type_ == STATE_OUTSIDE_OF_WORKSPACE) || (type_ == STATE_OBJECTS_TURNED) || (type_ == STATE_TARGET_REACHED) ||
                ( type_ == STATE_MAX_MOVES_REACHED ) ;
    }

    std::string toString() const ;
};

class Player {
public:
    struct Parameters {
        Parameters(const cvx::Variant &config) ;
        float motion_start_offset_ = 0.05 ;
        float motion_push_offset_ = 0.05 ;
        std::string target_ = "box_0_0";
        int32_t max_trials_ = 100 ;
        Eigen::Vector2f target_pos_ ;
        float target_radius_ ;
    };

    Player(const Parameters &params, World *world);

    void reset() ;

    // perform action and return new state, reward and done flaf
    std::tuple<State, bool> step(const State &state, int64_t action_index) ;

    // check if action is feasible for the current state
    bool isFeasible(const State &state, int64_t action_idx) ;

    State getState() const ;

    std::vector<std::string> getBoxNames() const { return box_names_ ; }

    World *world() { return world_ ; }

    const Parameters &params() const { return params_ ; }

    int64_t numActions() const { return actions_.size() ; }
    int64_t numBoxes() const { return box_names_.size() ; }

    std::vector<int64_t> getFeasibleActions(const State &state);
protected:

    void runSim(float t) ;

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> computeMotion(const Eigen::Vector2f &c, float theta, int action_id) ;

    World *world_ ;
    int32_t trial_ = 0 ;
    int32_t target_idx_ ;
    Parameters params_ ;
    std::vector<PushAction> actions_ ;
    std::vector<std::string> box_names_ ;

};
