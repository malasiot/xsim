#include "trainer.hpp"

#include <iostream>

using namespace cvx;
using namespace std ;
using namespace Eigen ;

Trainer::Trainer(DQNAgent *agent, const cvx::Variant &config): agent_(agent) {
    config.lookup("epsilon.start", epsilon_start_) ;
    config.lookup("epsilon.final", epsilon_final_) ;
    config.lookup("epsilon.decay", epsilon_decay_) ;
}

double Trainer::epsilon_by_frame(int64_t frame_id){
    return epsilon_final_ + (epsilon_start_ - epsilon_final_) * exp(-1. * frame_id / epsilon_decay_);
}

struct ExperienceSample {
    ExperienceSample(const State &state, const State &next_state, int64_t action,
                     float reward, bool done):
        state_(state), next_state_(next_state), action_(action), done_(done), reward_(reward) {
    }

    State state_ ;
    State next_state_ ;
    int64_t action_ ;
    float reward_ ;
    bool done_ ;
};

static vector<ExperienceSample> sampleTransitions(const std::vector<ExperienceSample> &trajectory, int from_t, int future_k) {
    static std::mt19937 rng{std::random_device{}()};

    vector<ExperienceSample> selected ;
    if ( future_k == 1 )// 'final' strategy
        selected.push_back(trajectory.back()) ;
    else { // 'future' strategy
        int steps_taken = trajectory.size() ;
        int k = std::min(future_k, steps_taken-1 - from_t) ;

        if ( k > 0 )
            std::sample(trajectory.begin() + from_t+1,  trajectory.end(),
                        std::back_inserter(selected), k, rng);
    }

    return selected ;
}

void Trainer::train(int64_t num_episodes){

    float episode_reward = 0.0;

    Environment *env = agent_->env() ;


    for(int i=1; i<=num_episodes; i++) {

        cout << "episode: " <<i << endl ;

        // start a new episode
        env->reset() ;
        auto state = env->getState() ;
        auto goal = env->desiredGoal() ;

        bool done = false ;
        std::vector<ExperienceSample> trajectory ;

        while ( !done ) { // play single episode and record experience

            int64_t action = agent_->act(state, goal, epsilon_by_frame(i)) ; // select action

            State new_state ;
            float reward ;

            std::tie(new_state, reward, done) = env->transition(state, action) ; // get new state

            agent_->learn(state, new_state, action, reward, done, goal) ; // learn experience

            trajectory.emplace_back(state, new_state, action, reward, done) ; // save experience to be used for additional goal sampling

            state = new_state ;
        }

        for( size_t t = 0 ; t<trajectory.size() ; t++ ) {
            const auto &e = trajectory[t] ;

            auto selected_transitions = sampleTransitions(trajectory, t, future_k_);
            for (const auto &transition: selected_transitions ) {
                VectorXf additional_goal = env->stateToGoal(transition.state_) ;

                float reward = env->computeRewardForGoal(e.next_state_, additional_goal) ;

                agent_->learn(e.state_, e.next_state_, e.action_, reward, false, additional_goal) ; // learn experience conditioned on new goal
            }

        }

        if ((i + 1) % eval_every_ == 0 ) {
            cout << "Running evaluation" << endl ;

            float total_reward = 0 ;

            for( int k=0 ; k<eval_episodes_  ; k++ ) {
                env->reset() ;
                auto state = env->getState() ;
                auto goal = env->desiredGoal() ;

                bool done = false ;

                while ( !done ) { // play single episode and record experience

                    int64_t action = agent_->act(state, goal, 0.0) ; // select action

                    cout << action << endl ;

                    float reward ;

                    std::tie(state, reward, done) = env->transition(state, action) ; // get new state

                    total_reward += reward ;
                }
            }

            float score = total_reward / eval_episodes_ ;

            cout << "Evaluation score: " << score << endl ;

            stringstream fs ;
            fs << "weights_" << setfill('0') << setw(5) << i << ".pth";

            agent_->save(fs.str()) ;
        }
    }

}
