#pragma once

#include <torch/torch.h>

#include "experience_replay.hpp"
#include "dqn.hpp"
#include "environment.hpp"

#include <cvx/math/rng.hpp>

class Trainer{
public:
    Trainer(Environment *env, int64_t input_channels, int64_t num_actions, int64_t capacity);

    torch::Tensor compute_td_loss(int64_t batch_size, float gamma);

    double epsilon_by_frame(int64_t frame_id);

    torch::Tensor stateToTensor(const State &state);

    void loadStateDict(torch::nn::Module& model,
                       torch::nn::Module& target_model);

    void train(int64_t num_epochs);

private:
    ExperienceReplay buffer_;
    DQN network_, target_network_;
    torch::optim::Adam optimizer_ ;
    Environment *env_ ;

    double epsilon_start_ = 1.0;
    double epsilon_final_ = 0.01;
    int64_t epsilon_decay_ = 30000;
    int64_t batch_size_ = 32;
    float gamma_ = 0.99;
    cvx::RNG rng_ ;





};
