import numpy as np


class QAgent():
    def __init__(self, n_actions, world_shape, learning_rate=0.2, discount_factor=0.9):
        self.n_actions = n_actions
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        
        self.q_table = np.zeros([n_actions, *world_shape])

    def act(self, observation, explore=True):
        if explore:
            # sample random action during training e.g. a random integer in [0, n_actions - 1]
            action = np.random.randint(self.n_actions)
        else:
             # Initialize an array to accumulate Q-values for each action
            accumulated_q_values = np.zeros(self.q_table.shape[0])

            # Iterate over each point in the observations array
            for obs in observation:
                # Retrieve Q-values for the current point
                q_values = self.q_table[:, obs[0], obs[1]]
                # Add the Q-values to the accumulator
                accumulated_q_values += q_values

            # Compute the average Q-values for each action
            average_q_values = accumulated_q_values / len(observation)

            # Use np.argmax to find the action with the highest average Q-value
            action = np.argmax(average_q_values)
        return action

    def learn(self, obs_0, action_0, reward_0, obs_1):
        # Initialize accumulators for current Q-values and next values
        accumulated_current_q_value = 0
        accumulated_next_value = 0

        # Iterate over each point in obs_0 and obs_1
        for o0, o1 in zip(obs_0, obs_1):
            # Retrieve the current Q-value approximation for obs_0 and action_0 from the Q-table
            accumulated_current_q_value += self.q_table[action_0, o0[0], o0[1]]
            
            # Compute the value of the next state obs_1
            accumulated_next_value += np.max(self.q_table[:, o1[0], o1[1]])

        # Compute the average current Q-value and next value
        average_current_q_value = accumulated_current_q_value / len(obs_0)
        average_next_value = accumulated_next_value / len(obs_1)

        # Compute new estimation of Q-value for obs_0 and action_0
        new_q_value = reward_0 + self.discount_factor * average_next_value
        
        # Adjust old estimation using the learning rate
        adjusted_q_value = (1 - self.learning_rate) * average_current_q_value + self.learning_rate * new_q_value
        
        # Update the Q-values in the Q-table for each point in obs_0
        for o0 in obs_0:
            self.q_table[action_0, o0[0], o0[1]] = adjusted_q_value
