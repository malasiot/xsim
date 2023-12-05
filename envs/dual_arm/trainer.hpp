#pragma once

#include "experience_replay.hpp"
#include "agent.hpp"
#include "environment.hpp"

#include <cvx/math/rng.hpp>
#include <cvx/misc/variant.hpp>

class Trainer{
public:
    Trainer(DQNAgent *agent, const cvx::Variant &params);

    double epsilon_by_frame(int64_t frame_id);

    void train(int64_t num_episodes);

    DQNAgent *agent() { return agent_ ; }

private:
    DQNAgent *agent_ ;

    double epsilon_start_ = 1.0;
    double epsilon_final_ = 0.05;
    int64_t epsilon_decay_ = 20000;
    int future_k_ = 4 ;
    bool dense_reward_ = false ;
    int64_t eval_every_ = 10, eval_episodes_ = 5 ;

};
