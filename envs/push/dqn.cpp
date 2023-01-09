#include "dqn.hpp"

using namespace torch::nn;

DQN::DQN(int64_t input_channels, int64_t num_actions) {
      conv1_ = Conv2d(Conv2dOptions(input_channels, 32, 8).stride(4));
      conv2_ = Conv2d(Conv2dOptions(32, 64, 4).stride(2)) ;
      conv3_ = Conv2d(Conv2dOptions(64, 64, 3).stride(1)) ;
      linear1_ = torch::nn::Linear(64*22*16, 512) ;
      output_ = torch::nn::Linear(512, num_actions) ;
}

at::Tensor DQN::forward(at::Tensor input) {
    input = torch::relu(conv1_(input));
    input = torch::relu(conv2_(input));
    input = torch::relu(conv3_(input));
    // Flatten the output
    input = input.view({input.size(0), -1});
    input = torch::relu(linear1_(input));
    input = output_(input);
    return input;
}

at::Tensor DQN::act(at::Tensor state) {
    torch::Tensor q_value = forward(state);
    torch::Tensor action = std::get<1>(q_value.max(1));
    return action;
}

#include "dqn.hpp"
