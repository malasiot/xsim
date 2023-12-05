#include "agent.hpp"
#include "dqn.hpp"

#include <torch/torch.h>

using namespace std ;
using namespace Eigen ;


static void load_state_dict(torch::nn::Module& model, torch::nn::Module& target_model, float tau) {
    torch::NoGradGuard no_grad;
    auto new_params = target_model.named_parameters(); // implement this
    auto params = model.named_parameters(true /*recurse*/);
    auto buffers = target_model.named_buffers(true /*recurse*/);

    for (auto& val : params) {
        auto name = val.key();
        auto* t = new_params.find(name);
        if (t != nullptr) {
            t->copy_(val.value() * tau + (1.0 -tau) * (*t));
        } else {
            t = buffers.find(name);
            if (t != nullptr) {
                t->copy_(val.value() * tau + (1.0 - tau) * (*t));
            }
        }
    }
}

DQNAgent::DQNAgent(Environment *e, const cvx::Variant &config): env_(e) {

    config.lookup("batch_size", batch_size_) ;
    config.lookup("gamma", gamma_) ;
    config.lookup("er_capacity", experience_replay_capacity_ ) ;

    input_channels_ = env_->stateSpaceDim() + env_->goalSpaceDim();
    num_actions_ = env_->numActions() ;

    buffer_.reset(new ExperienceReplay(experience_replay_capacity_)) ;
    network_.reset(new DQN(input_channels_, num_actions_)) ;
    target_network_.reset(new DQN(input_channels_, num_actions_)) ;
    load_state_dict(*network_, *target_network_, 1.0) ;
    optimizer_.reset(new torch::optim::Adam(network_->parameters(), torch::optim::AdamOptions(0.0001).betas(std::make_tuple(0.5, 0.999)))) ;
}

DQNAgent::~DQNAgent()
{

}



static at::Tensor to_tensor(const VectorXf &v) {
   return at::from_blob((void *)v.data(), { 1, v.size()}, at::TensorOptions().dtype(at::kFloat)).clone();

}

static at::Tensor state_goal_tensor(const VectorXf &state, const VectorXf &goal) {
    VectorXf cat(state.size() + goal.size()) ;
    int k = 0 ;
    for(int i=0 ; i<state.size()  ; i++, k++ ) cat[k] = state[i];
    for(int i=0 ; i<goal.size()  ; i++, k++ ) cat[k] = goal[i];
    return to_tensor(cat) ;
}

int64_t DQNAgent::act(const State &state, const VectorXf &goal, float epsilon, bool use_target) {
    auto r = rng_.uniform<double>() ;

    int64_t action_index ;

    if ( r <= epsilon ){ // exploration
        do {
        action_index = rng_.uniform<int64_t>(0, num_actions_-1) ;
        } while ( !env_->isFeasible(state, action_index)) ;
    }
    else{
        auto t = state_goal_tensor(env_->stateToTensor(state), goal) ;


        network_->eval() ;

        torch::Tensor q_value ;

        { torch::NoGradGuard ng ;
            q_value = network_->forward(t) ;
        }

        network_->train() ;

        // sort q_values in descending order

        float *data = (float *)q_value.data_ptr() ;

        vector<int> q_index(num_actions_) ;
        std::iota (std::begin(q_index), std::end(q_index), 0);
        vector<float> q_values(data, data + num_actions_) ;
        std::sort(q_index.begin(), q_index.end(), [&] (int a, int b) {
            return q_values[a] > q_values[b] ;
        });

        for(auto idx: q_index ) {
            if ( env_->isFeasible(state, idx) ) {
                action_index = idx ;
                break ;
            }
        }

        cout << action_index << endl ;
    }

    return action_index ;

}

void DQNAgent::learn(const State &state, const State &new_state, int64_t action, float reward, bool done, const VectorXf &goal) {

    auto state_tensor = state_goal_tensor(env_->stateToTensor(state), goal) ;
    auto new_state_tensor = state_goal_tensor(env_->stateToTensor(new_state), goal) ;

    buffer_->push(state_tensor, new_state_tensor, action, reward, done);

    if ( buffer_->size() >= batch_size_ ) {

        at::Tensor loss = fit();
//        cout << loss.item<float>() << endl ;
        load_state_dict(*network_, *target_network_, tau_);

    }


}

void DQNAgent::save(const std::string &out_path) {
    torch::serialize::OutputArchive output_model_archive;
    target_network_->to(torch::kCPU);
    target_network_->save(output_model_archive);
    output_model_archive.save_to(out_path);
}

at::Tensor DQNAgent::fit(){
    std::vector<ExperienceReplay::Sample> batch = buffer_->sample(batch_size_);

    std::vector<torch::Tensor> states;
    std::vector<torch::Tensor> new_states;
    std::vector<int64_t> actions;
    std::vector<float> rewards;
    std::vector<float> dones;

    for (const auto &i : batch){
        states.push_back(i.state_);
        new_states.push_back(i.new_state_);
        actions.push_back(i.action_);
        rewards.push_back(i.reward_);
        dones.push_back(i.done_);
    }

    torch::Tensor states_tensor;
    torch::Tensor new_states_tensor;
    torch::Tensor actions_tensor;
    torch::Tensor rewards_tensor;
    torch::Tensor dones_tensor;

    rewards_tensor = torch::from_blob(rewards.data(), { rewards.size() }, torch::TensorOptions().dtype(at::kFloat)) ;
    actions_tensor = torch::from_blob(actions.data(), { actions.size() }, torch::TensorOptions().dtype(at::kLong)) ;
    dones_tensor = torch::from_blob(dones.data(), { dones.size() }, torch::TensorOptions().dtype(at::kFloat)) ;

    states_tensor = torch::cat(states, 0);
    new_states_tensor = torch::cat(new_states, 0);

    torch::Tensor q_values = network_->forward(states_tensor);
    torch::Tensor next_target_q_values = target_network_->forward(new_states_tensor);
    torch::Tensor next_q_values = network_->forward(new_states_tensor);

    torch::Tensor q_value = q_values.gather(1, actions_tensor.unsqueeze(1)).squeeze(1);
    torch::Tensor maximum = std::get<1>(next_q_values.max(1));
    torch::Tensor next_q_value = next_target_q_values.gather(1, maximum.unsqueeze(1)).squeeze(1);
    torch::Tensor expected_q_value = rewards_tensor + gamma_*next_q_value*(1-dones_tensor);

    float clip_return = 1 / (1 - gamma_) ;
    expected_q_value = torch::clamp(expected_q_value, -clip_return, clip_return) ;



    torch::Tensor loss = torch::mse_loss(q_value, expected_q_value);

    optimizer_->zero_grad();
    loss.backward();
    optimizer_->step();

    return loss;

}


