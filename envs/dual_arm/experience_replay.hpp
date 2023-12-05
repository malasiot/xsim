#pragma once

#include <ATen/Tensor.h>
#include <memory>
#include <vector>
#include <iostream>

#include <algorithm>
#include <iterator>
#include <random>
#include <Eigen/Core>



class ExperienceReplay {
public:
    struct Sample {
        Sample(const at::Tensor &state, const at::Tensor &new_state, int64_t action,
               float reward, bool done):
            state_(state), new_state_(new_state), action_(action), done_(done), reward_(reward) {
        }

        at::Tensor state_ ;
        at::Tensor new_state_ ;
        int64_t action_ ;
        float reward_ ;
        bool done_ ;

    };

    ExperienceReplay (int64_t capacity);
    void push(at::Tensor &state, at::Tensor &new_state, int64_t action, float reward, bool done);
    int64_t size() const ;
    std::vector<Sample> sample(int64_t batch_size) const;

public:
    std::deque<Sample> buffer_ ;
private:
    int64_t capacity_;
};
