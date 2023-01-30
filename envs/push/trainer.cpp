#include "trainer.hpp"

#include <iostream>

using namespace cvx;
using namespace std ;

Trainer::Trainer(DQNAgent *agent, const cvx::Variant &config): agent_(agent) {
    config.lookup("epsilon.start", epsilon_start_) ;
    config.lookup("epsilon.final", epsilon_final_) ;
    config.lookup("epsilon.decay", epsilon_decay_) ;
}

double Trainer::epsilon_by_frame(int64_t frame_id){
    return epsilon_final_ + (epsilon_start_ - epsilon_final_) * exp(-1. * frame_id / epsilon_decay_);
}


void Trainer::train(int64_t num_episodes){

    float episode_reward = 0.0;

    Environment *env = agent_->env() ;

    auto state = env->getState() ;

    for(int i=1; i<=num_episodes; i++) {
         int64_t action = agent_->act(state, epsilon_by_frame(i)) ;

         auto [new_state, reward, done] = env->transition(state, action) ;
         episode_reward += reward;

         agent_->learn(state, new_state, action, reward, done) ;

         state = new_state;

         if (done) {
             env->reset() ;
             state = env->getState() ;
             cout << episode_reward << endl;
             episode_reward = 0.0;
         }
     }



 }
