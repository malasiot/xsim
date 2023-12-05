import copy
import random
import numpy as np
import warnings
import torch
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.tensorboard import SummaryWriter
from collections import deque
from replay_buffer import ReplayBuffer
from utils import make_experience, from_experience, seed_all, sample_transitions
from device import device
from network import DQNetwork
import math


class DQNAgent:
    def __init__(self, config, env):
        self.config = config
        self.env = env

        lr = config.lr
        use_her = config.use_her
        state_size = env.state_size
        goal_size = env.goal_size
        action_size = env.action_size
        buffer_size = config.buffer_size
        batch_size = config.batch_size

        net_state_size = state_size + (goal_size if use_her else 0)
        self.qn_local = DQNetwork(net_state_size, action_size).to(device)
        self.qn_target = DQNetwork(net_state_size, action_size).to(device)

        self.soft_update(1.0)

        self.optimizer = optim.Adam(self.qn_local.parameters(), lr=lr)
        self.memory = ReplayBuffer(buffer_size, batch_size)

        self.losses = deque(maxlen=100)
        self.t_step = 0

    def process_input(self, state, goal=None):
        use_her = self.config.use_her

        if use_her and goal is not None:
            state = np.concatenate([self.env.stateVec(state), self.env.goalVec(goal)])

        return state

    def act(self, state, goal, feasible, eps=0.0, use_target=False):
       
        state_input = self.process_input(state, goal)
        state_input = torch.from_numpy(state_input).float().unsqueeze(0).to(device)

        model = self.qn_target if use_target else self.qn_local

        model.eval()
        with torch.no_grad():
            action_values = model(state_input)
        model.train()

        mask = [j for j in range(self.env.action_size) if j not in feasible  ]
        action_values[:,mask] = float('-inf') 
        
        # ε-greedy
        if random.random() > eps:
            return int(np.argmax(action_values.cpu().numpy()))
        else:
            return random.choice(feasible)

    def step(self, state, action, reward, next_state, done, goal, feasible):
        update_every = self.config.update_every
        batch_size = self.config.batch_size

        self.add_experience(state, action, reward, next_state, done, goal, feasible)

        self.t_step = (self.t_step + 1) % update_every

        if self.t_step == 0:
            if len(self.memory) > batch_size:
                experiences = self.memory.sample()
                return self.learn(experiences)

    def learn(self, experiences):
        (states, actions, rewards, next_states, dones, feasible) = from_experience(experiences)

        tau = self.config.tau
        gamma = self.config.gamma
        use_her = self.config.use_her
        use_double = self.config.use_double
        use_huber_loss = self.config.use_huber_loss

        if use_double:
            # Double DQN: https://arxiv.org/abs/1509.06461
            
            q = self.qn_local(next_states)
            
            qs = q.detach()
            mask = []
            for i in range(len(experiences)):
                e = experiences[i]
                mask = [j for j in range(self.env.action_size) if j not in e.feasible]
                qs[i, mask] = -100

            #masking of non feasible actions            
 #           mask = [i for i in range(self.env.action_size) if i not in feasible]
  #          q[:, mask] = float('-inf')
          
            best_action = qs.argmax(-1, keepdim=True)
            max_q = self.qn_target(next_states).detach().gather(-1, best_action)
        else:
            max_q = self.qn_target(next_states).detach().max(-1, keepdim=True)[0]

        q_targets = rewards + (gamma * max_q * (1 - dones))

        # Clipping targets as suggested in the paper: 
        # See A Experiment details / Training procedure
        if use_her:
            clip_return = 1 / (1 - gamma)
            q_targets = torch.clamp(q_targets, -clip_return, clip_return)

        q_expected = self.qn_local(states).gather(-1, actions)

        if use_huber_loss:
            loss = F.smooth_l1_loss(q_expected, q_targets)
        else:
            loss = F.mse_loss(q_expected, q_targets)

        self.losses.append(loss.item())

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        self.soft_update(tau)

    def soft_update(self, tau):
        for target_param, local_param in zip(
            self.qn_target.parameters(), 
            self.qn_local.parameters()
        ):
            target_param.data.copy_(tau * local_param + (1.0 - tau) * target_param)

    def add_experience(self, state, action, reward, next_state, done, goal, feasible):
        use_her = self.config.use_her

        state_ = self.process_input(state, goal)
        next_state_ = self.process_input(next_state, goal)

        experience = make_experience(state_, action, reward, next_state_, done, None, feasible)
        self.memory.add(experience)

    def eval_episode(self, use_target=False):

        times_eval = self.config.times_eval
        max_steps = self.config.max_steps

        total_reward = 0

        for _ in range(times_eval):
            goal = self.env.goal()
            state, feasible = self.env.reset()
            done = False

            while not done:
                action = self.act(state, goal, feasible, use_target=use_target)
                state, done, feasible = self.env.step(action)
                reward, success = self.env.computeReward(state, goal, done)
                done = True if success else done
                total_reward += reward

        return total_reward / times_eval

    def train(self):
        print("Training on {}".format(device))

        eps_start = self.config.eps_start
        eps_end = self.config.eps_end
        eps_decay = self.config.eps_decay
        episodes = self.config.episodes
        max_steps = self.config.max_steps
        use_her = self.config.use_her
        future_k = self.config.future_k
        eval_every = self.config.eval_every

        writer = SummaryWriter(comment="Push")

        most_reached = 1
        best_score = -np.inf
        eps = eps_start

        ### DQN algorithm ###
        for episode in range(episodes):

            info = None
            total_reward = 0
            target_reached = 0
            trajectory = []
            goal = self.env.goal()
            state, feasible = self.env.reset()

            for _ in range(max_steps):
                # With probability eps select a random action, otherwise select max
                action = self.act(state, goal, feasible, eps)
                
            
                # Execute action and observe next state and reward
                next_state, done, feasible = self.env.step(action)
                reward, success = self.env.computeReward(next_state, goal, done)
               
                done = True if success else done
                # Store transition and perform optimization step
                self.step(state, action, reward, next_state, done, goal, feasible)

                # Store for potential use in HER
                trajectory.append(
                    make_experience(state, 
                                    action, 
                                    reward, 
                                    next_state, 
                                    done, 
                                    success,
                                    feasible
                                    )
                )

                total_reward += reward
                target_reached += int(success)
                state = next_state

                if done:
                    break
            
            ### HER does her magic here ###
            if use_her:
                steps_taken = len(trajectory)
                
                # Replay transitions with different goals
                for t in range(steps_taken):
                    state, action, _, next_state, done, success, feasible = copy.deepcopy(trajectory[t])
                    
                    # Will sample final or future random transitions depending on 'future_k'
                    #   future_k = 1 => final strategy
                    #   future_k > 1 => future strategy
                    selected_transitions = sample_transitions(trajectory, t, future_k)

                    # Loop over virtual goals. These are achieved goals along the episode
                    for transition in selected_transitions:
                        additional_goal = self.env.to_goal(transition.state)

                        # Recompute reward
                        
                        reward, _ = self.env.computeReward(next_state, 
                                                       additional_goal,
                                                       done)

                        # Store in buffer, with a new goal, and perform optimization step
                        self.step(state, 
                                  action, 
                                  reward, 
                                  next_state, 
                                  False, # we're not done even if goal is reached (keep hovering)
                                  additional_goal,
                                  feasible
                                  )
            ### End HER ###

            avg_loss = np.mean(self.losses)

            print(
                "episode: {}, exploration: {:.2f}%, target reached: {}, total reward: {:.2f}, avg Loss: {:.3f}".format(
                    episode + 1, 
                    100 * eps, 
                    target_reached, 
                    total_reward, 
                    avg_loss
                )
            )
            
            writer.add_scalar("Target Reached", target_reached, episode)
            writer.add_scalar("Episode Reward", total_reward, episode)
            writer.add_scalar("Avg Loss", avg_loss, episode)

            if (episode+1) % eval_every == 0:
                print("\nRunning evaluation...")

                score = self.eval_episode(use_target=False)

                print("eval score: {:.2f}".format(score))
                writer.add_scalar("Evaluation Score", score, episode)

                if score > best_score:
                    best_score = score
                    torch.save(self.qn_local.state_dict(), "best_weights.pth")
            
            eps = eps_end + (eps_start - eps_end) * math.exp(-1. * episode / eps_decay);
            

        ### End DQN ###

