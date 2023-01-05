#pragma once

#include <torch/torch.h>

struct DQN : torch::nn::Module{
    DQN(int64_t input_channels, int64_t num_actions);

    torch::Tensor forward(torch::Tensor input);
    torch::Tensor act(torch::Tensor state);

    torch::nn::Conv2d conv1_ = nullptr, conv2_ = nullptr, conv3_ = nullptr ;
    torch::nn::Linear linear1_ = nullptr, output_ = nullptr ;
};
