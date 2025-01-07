import numpy as np

class QAgent():
    def __init__(self, n_actions, world_shape, learning_rate=0.2, discount_factor=0.9):
        # TODO set class variables: n_actions, learning_rate and discount_factor
        self.n_actions = n_actions
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        
        # TODO initialize q-table with zeros of shape [n_actions, n_rows, n_columns]
        self.q_table = np.zeros([n_actions, *world_shape])

    def act(self, observation, explore=True):
        if explore:
            # TODO sample random action during training e.g. a random integer in [0, n_actions - 1]
            action = np.random.randint(self.n_actions)
        else:
            # TODO get the q-values from the q-table at the given position (observation is the same as the agent's position)
            q_values = self.q_table[:, observation[0], observation[1]]
            # TODO find the action with the maximal q-value
            action = np.argmax(q_values)
        return action

    def learn(self, obs_0, action_0, reward_0, obs_1):
        # TODO retrieve the current q-value approximation for obs_0 and action_0 from the q-table
        current_q_value = self.q_table[action_0, obs_0[0], obs_0[1]]
        
        # TODO compute the value of the next state obs_1
        next_value = np.max(self.q_table[:, obs_1[0], obs_1[1]])
        
        # TODO compute new estimation of q-value for obs_0 and action_0
        new_q_value = reward_0 + self.discount_factor * next_value
        
        # TODO adjust old estimation using the learning rate
        adjusted_q_value = (1 - self.learning_rate) * current_q_value + self.learning_rate * new_q_value
        # TODO set the new q-value approximation in the q-table
        self.q_table[action_0, obs_0[0], obs_0[1]] = adjusted_q_value