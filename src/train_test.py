def train(agent, env, steps):
    print("Start train")
    # reset environment and get initial observation
    obs = env.reset()
    for i in range(steps):
        # get action from agent, given the observation
        action = agent.act(obs)
        # execute action, get reward, new observation and termination flag
        obs_1, r, done = env.step(action)
        # learn from the gathered experience
        agent.learn(obs, action, r, obs_1)
        # set obs to new observation obs_1
        obs = obs_1
        # reset environment if game terminated
        if done:
            obs = env.reset()

    print("end train")

def test(agent, env, max_steps=162):
    print("Start test")
    # reset environment and get initial observation
    obs = env.reset()
    
    # record path of agent in a list of positions; the fist position is the initial observation
    path = [obs]
    moves = []

    # initialize the cumulated reward as 0
    cumulated_reward = 0.0
    
    # we want to execute only one episode --> until the game is done or until a maximum nuber of steps is reached
    done = False
    n_steps = 0
    while not done and n_steps < max_steps:
        # get action from agent, don't forget to set the explore flag to False
        action = agent.act(obs, explore=False)
        # execute action, get reward and new observation
        obs_1, r, done = env.step(action)
        # record path of agent
        path.append(obs_1)
        moves.append(action)
        # increment cumulated reward by received reward
        cumulated_reward += r
        # set obs to new observation obs_1
        obs = obs_1
        n_steps += 1

    print("end test")
    return cumulated_reward, path, moves
