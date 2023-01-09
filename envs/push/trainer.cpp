#include "trainer.hpp"
#include <cvx/math/rng.hpp>

Trainer::Trainer(Environment *env, int64_t input_channels, int64_t num_actions, int64_t capacity):
    env_(env), buffer_(capacity), network_(input_channels, num_actions), target_network_(input_channels, num_actions),
    optimizer_(network_.parameters(), torch::optim::AdamOptions(0.0001).betas(std::make_tuple(0.5, 0.999))) {

}

torch::Tensor Trainer::compute_td_loss(int64_t batch_size, float gamma){
    std::vector<ExperienceReplay::Sample> batch = buffer_.sample(batch_size);

    std::vector<torch::Tensor> states;
    std::vector<torch::Tensor> new_states;
    std::vector<torch::Tensor> actions;
    std::vector<torch::Tensor> rewards;
    std::vector<torch::Tensor> dones;

    for (auto i : batch){
        states.push_back(std::get<0>(i));
        new_states.push_back(std::get<1>(i));
        actions.push_back(std::get<2>(i));
        rewards.push_back(std::get<3>(i));
        dones.push_back(std::get<4>(i));
    }

    torch::Tensor states_tensor;
    torch::Tensor new_states_tensor;
    torch::Tensor actions_tensor;
    torch::Tensor rewards_tensor;
    torch::Tensor dones_tensor;

    states_tensor = torch::cat(states, 0);
    new_states_tensor = torch::cat(new_states, 0);
    actions_tensor = torch::cat(actions, 0);
    rewards_tensor = torch::cat(rewards, 0);
    dones_tensor = torch::cat(dones, 0);

    torch::Tensor q_values = network_.forward(states_tensor);
    torch::Tensor next_target_q_values = target_network_.forward(new_states_tensor);
    torch::Tensor next_q_values = network_.forward(new_states_tensor);

    actions_tensor = actions_tensor.to(torch::kInt64);

    torch::Tensor q_value = q_values.gather(1, actions_tensor.unsqueeze(1)).squeeze(1);
    torch::Tensor maximum = std::get<1>(next_q_values.max(1));
    torch::Tensor next_q_value = next_target_q_values.gather(1, maximum.unsqueeze(1)).squeeze(1);
    torch::Tensor expected_q_value = rewards_tensor + gamma*next_q_value*(1-dones_tensor);
    torch::Tensor loss = torch::mse_loss(q_value, expected_q_value);

    optimizer_.zero_grad();
    loss.backward();
    optimizer_.step();

    return loss;

}


double Trainer::epsilon_by_frame(int64_t frame_id){
    return epsilon_final_ + (epsilon_start_ - epsilon_final_) * exp(-1. * frame_id / epsilon_decay_);
}

torch::Tensor Trainer::stateToTensor(const State &state) {
    std::vector<double> state_dbl;
    state_dbl.reserve(state.boxes_.size() * 3);

    for (const auto &bp: state.boxes_ ){
        const auto &box = bp.second ;
        state_dbl.push_back(box.cx_);
        state_dbl.push_back(box.cy_);
        state_dbl.push_back(box.theta_);
    }

    torch::Tensor state_tensor = torch::from_blob(state_dbl.data(), { state_dbl.size()});
    return state_tensor;
}

void Trainer::loadStateDict(torch::nn::Module& model, torch::nn::Module& target_model) {
    torch::autograd::GradMode::set_enabled(false);  // make parameters copying possible
    auto new_params = target_model.named_parameters(); // implement this
    auto params = model.named_parameters(true /*recurse*/);
    auto buffers = model.named_buffers(true /*recurse*/);
    for (auto& val : new_params) {
        auto name = val.key();
        auto* t = params.find(name);
        if (t != nullptr) {
            t->copy_(val.value());
        } else {
            t = buffers.find(name);
            if (t != nullptr) {
                t->copy_(val.value());
            }
        }
    }
}

void Trainer::train(int64_t num_epochs){

    auto actions = env_->getActions() ;

    float episode_reward = 0.0;
    std::vector<float> all_rewards;
    std::vector<torch::Tensor> losses;
    auto start = std::chrono::high_resolution_clock::now();

    auto state = env_->getState() ;

    for(int i=1; i<=num_epochs; i++) {
         double epsilon = epsilon_by_frame(i);
         auto r = rng_.uniform<double>() ;
         torch::Tensor state_tensor = stateToTensor(state);
         PushAction a;
         int64_t action_index ;

         if ( r <= epsilon ){ // exploration
             action_index = rng_.uniform<int64_t>(0, actions.size()-1) ;
         }
         else{
             torch::Tensor action_tensor = network_.act(state_tensor);
             action_index = action_tensor[0].item<int64_t>();
         }

         a = actions[action_index];

         float reward = env_->apply(state, a) ;
         episode_reward += reward;
         State new_state = env_->getState() ;

         torch::Tensor new_state_tensor = stateToTensor(new_state);
         bool done = ( new_state.type_ != StateType::STATE_VALID ) ;

         torch::Tensor reward_tensor = torch::tensor(reward);
         torch::Tensor done_tensor = torch::tensor(done);
         done_tensor = done_tensor.to(torch::kFloat32);
         torch::Tensor action_tensor_new = torch::tensor(action_index);

         buffer_.push(state_tensor, new_state_tensor, action_tensor_new, done_tensor, reward_tensor);
         state = new_state;

         if (done) {
             env_->reset() ;
             all_rewards.push_back(episode_reward);
             episode_reward = 0.0;
         }

         if (buffer_.size() >= 10000){
             torch::Tensor loss = compute_td_loss(batch_size_, gamma_);
             losses.push_back(loss);
         }

         if (i%1000==0){
             std::cout<<episode_reward<<std::endl;
             loadStateDict(network_, target_network_);
         }

     }
     auto stop = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
     std::cout << "Time taken by function: "
          << duration.count() << " microseconds" << std::endl;


 }
