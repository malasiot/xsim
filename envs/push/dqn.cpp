#include "dqn.hpp"

#include <iostream>

using namespace torch::nn;
using namespace std ;

DQN::DQN(int64_t input_channels, int64_t num_actions) {

    layer1_ = register_module("layer1", Linear(input_channels, 128));
    layer2_ = register_module("layer2", Linear(128, 128));
    layer3_ = register_module("layer3", Linear(128, num_actions));

}

at::Tensor DQN::forward(at::Tensor input) {
    input = torch::relu(layer1_(input));
    input = torch::relu(layer2_(input));
    return layer3_(input);
}

at::Tensor DQN::act(at::Tensor state) {
    torch::Tensor q_value = forward(state);

    torch::Tensor action = std::get<1>(q_value.max(1));
    return action;
}

