#pragma once

#include <torch/torch.h>

struct DQN : torch::nn::Module{
    DQN() = default ;
    DQN(int64_t input_channels, int64_t num_actions);

    torch::Tensor forward(torch::Tensor input);
    torch::Tensor act(torch::Tensor state);

    torch::nn::Linear layer1_ = nullptr, layer2_ = nullptr, layer3_ = nullptr ;
};
