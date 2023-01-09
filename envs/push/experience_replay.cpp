#include "experience_replay.hpp"

#include <memory>
#include <vector>
#include <iostream>

#include <torch/torch.h>
#include <c10/util/ArrayRef.h>

#include <algorithm>
#include <iterator>
#include <random>

ExperienceReplay::ExperienceReplay(int64_t size) {
    capacity_ = size;
}

void ExperienceReplay::push(torch::Tensor state, torch::Tensor new_state, torch::Tensor action, torch::Tensor done, torch::Tensor reward ){

    Sample sample (state, new_state, action, reward, done);
    if (buffer_.size() < capacity_){
        buffer_.push_back(sample);
    }
    else {
        while (buffer_.size() >= capacity_) {
            buffer_.pop_front();
        }
        buffer_.push_back(sample);
    }
}

std::vector<ExperienceReplay::Sample> ExperienceReplay::sample(int64_t batch_size) const {
    std::vector<Sample> b(batch_size);
    std::sample(buffer_.begin(), buffer_.end(),
                b.begin(), b.size(),
                std::mt19937{std::random_device{}()});
    return b;
}

int64_t ExperienceReplay::size() const {
    return buffer_.size();
}
