class Config:
    buffer_size = int(1e6)
    batch_size = 64
    gamma = 0.99
    tau = 1e-3
    episodes = 2000
    max_steps = 1000
    future_k = 4
    state_size = None
    action_size = None
    goal_size = None
    update_every = 4
    lr = 1e-4
    eps_start = 0.2
    eps_end = 0.1
    eps_decay = 1.
    dist_tolerance = 0.05
    dense_reward = False
    use_her = True
    use_double = False
    use_huber_loss = False
    times_eval = 100
    eval_every = 100
    target = "box_0_0"
    target_pos = [0.0, 0.3]
    target_radius = 0.05
    