def train(agent, env, steps):
    # TODO reset environment and get initial observation
    obs = env.reset()
    for i in range(steps):
        # TODO get action from agent, given the observation
        action = agent.act(obs)
        # TODO execute action, get reward, new observation and termination flag
        obs_1, r, done = env.step(action)
        # TODO learn from the gathered experience
        agent.learn(obs, action, r, obs_1)
        # TODO set obs to new observation obs_1
        obs = obs_1
        # TODO reset environment if game terminated
        if done:
            obs = env.reset()