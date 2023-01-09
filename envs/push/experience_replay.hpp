#pragma once

#include <torch/torch.h>
#include <memory>
#include <vector>
#include <iostream>


#include <algorithm>
#include <iterator>
#include <random>

class ExperienceReplay {
public:
    using Sample = std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> ;

    ExperienceReplay (int64_t capacity);
    void push(torch::Tensor state, torch::Tensor new_state, torch::Tensor action, torch::Tensor done, torch::Tensor reward);
    int64_t size() const ;
    std::vector<Sample> sample(int64_t batch_size) const;

public:
    std::deque<Sample> buffer_ ;
private:
    int64_t capacity_;
};
