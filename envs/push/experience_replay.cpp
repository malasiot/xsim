#include "experience_replay.hpp"

#include <memory>
#include <vector>
#include <iostream>

#include <torch/torch.h>
#include <c10/util/ArrayRef.h>

#include <algorithm>
#include <iterator>
#include <random>
#include <iostream>

using namespace std ;

ExperienceReplay::ExperienceReplay(int64_t size) {
    capacity_ = size;
}

void ExperienceReplay::push(torch::Tensor &state, torch::Tensor &new_state, int64_t action, float reward, bool done){
    if (buffer_.size() < capacity_){
        buffer_.emplace_back(state, new_state, action, reward, done);
    }
    else {
        while (buffer_.size() >= capacity_) {
            buffer_.pop_front();
        }
        buffer_.emplace_back(state, new_state, action, reward, done);
    }
}

std::vector<ExperienceReplay::Sample> ExperienceReplay::sample(int64_t batch_size) const {
    std::vector<Sample> b;

    std::sample(buffer_.begin(), buffer_.end(),
                std::back_inserter(b), batch_size,
                std::mt19937{std::random_device{}()});
    return b;
}

int64_t ExperienceReplay::size() const {
    return buffer_.size();
}
