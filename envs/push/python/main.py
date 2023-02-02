from env import Env
from config import Config
from utils import seed_all
from dqn import DQNAgent
 
config = Config()

env = Env(config)

config.episodes = 3000
config.max_steps = 1000
config.buffer_size = int(1e6)
config.batch_size = 64
config.gamma = 0.999
config.tau = 0.05
config.update_every = 1
config.use_double = True
config.use_huber_loss = False
config.lr = 1e-3
config.eps_start = 0.9
config.eps_decay = 1000
config.eps_end = 0.1
config.use_her = True
config.future_k = 8
config.dist_tolerance = 0.2
config.dense_reward = False
config.times_eval = 10
config.eval_every = 20

seed_all(0)

agent = DQNAgent(config, env)

agent.train()
