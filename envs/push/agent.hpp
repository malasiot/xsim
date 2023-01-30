#pragma once

#include <cstdint>

#include "experience_replay.hpp"
#include "environment.hpp"

#include <cvx/math/rng.hpp>
#include <cvx/misc/variant.hpp>

class DQN ;

namespace torch {
namespace optim {
class Adam ;
}
}

class DQNAgent {
public:
    DQNAgent(Environment *e, const cvx::Variant &config) ;
    ~DQNAgent() ;

    // return action index based on e-greedy policy
    int64_t act(const State &state, float epsilon) ;

    void learn(const State &state, const State &new_state, int64_t action, float reward, bool done) ;

    Environment *env() { return env_ ; }

private:

    at::Tensor fit();

    std::unique_ptr<ExperienceReplay> buffer_;
    std::unique_ptr<DQN> network_, target_network_;
    std::unique_ptr<torch::optim::Adam> optimizer_ ;
    Environment *env_ ;
    int64_t input_channels_, num_actions_ ;


    int64_t batch_size_ = 32;
    float gamma_ = 0.99;
    int64_t experience_replay_capacity_ = 10000 ;

    cvx::RNG rng_ ;
};
