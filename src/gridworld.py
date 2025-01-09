import numpy as np
import pandas as pd

class GridWorld:

    def __init__(self,bullet_client, object_id, obstacle_ids):
        # Dimensions of the GridWorld.
        self.world_shape = (62, 50)

        # Dictionary of rewards with key: position and value: reward.
        self.reward_states = {(x, y): 1 for x in range(52, 60) for y in range(21, 29)}

        # initial position of the agent
        self.agent_init_pos = self.get_blocking_states(bullet_client, [object_id])

        # list of blocking state positions
        self.blocking_states = self.get_blocking_states(bullet_client, obstacle_ids)
        
        # the action representations are now integers, to make indexing and sampling for TD learning simpler
        self.possible_actions = {
            0: np.array([-1, 0]), # up
            1: np.array([1, 0]),  # down
            2: np.array([0, 1]),  # right
            3: np.array([0, -1])  # left
        }

        # set initial agent position
        self.agent_current_pos = self.agent_init_pos

        # list of collected rewards, to not collect rewards twice
        self.collected_rewards = []
        
    
    # Initial position in grid world.
    def position_to_world_index(self, coordinates):
        x_min, x_max = 0.3, 1.05
        y_min, y_max = -0.3, 0.3

        x = coordinates[0]
        y = coordinates[1]
        x_pos = int((x - x_min) / (x_max - x_min) * self.world_shape[0])
        y_pos = int((y - y_min) / (y_max - y_min) * self.world_shape[1])
        return x_pos, y_pos
    
    # List of blocking state positions.
    def get_blocking_states(self, bullet_client, obstacle_ids):
        blocking_states = []
        for obstacle_id in obstacle_ids:
            position, _ = bullet_client.getBasePositionAndOrientation(obstacle_id["id"])
            size_x = obstacle_id["size_x"]
            size_y = obstacle_id["size_y"]

            x_min, x_max = position[0] - size_x, position[0] + size_x
            y_min, y_max = position[1] - size_y, position[1] + size_y

            x_pos_min, y_pos_min = self.position_to_world_index((x_min, y_min))
            x_pos_max, y_pos_max = self.position_to_world_index((x_max, y_max))

            blocking_states = blocking_states + [(x, y) for x in range(x_pos_min, x_pos_max) for y in range(y_pos_min, y_pos_max)]
        return blocking_states

    def render_environment(self):
        legend = {
            'empty': 0,
            'agent': 4,
            'blocking': 8
        }

        # Initialize empty states.
        states = np.ones(self.world_shape) * legend['empty']
        
        # Add agent.
        # We can index states with agent_init_pos because is a tuple of ints.
        # You can not index a dictionary with numpy arrays.
        # Make sure, that everytime you call this function, the indexing values are tuples of ints.
        for pos in self.agent_init_pos:
            states[pos] = legend['agent']
        
        # Iterate over blocking_states, and set the states value according to the legend.
        # blocking_states is a list of tuples of ints.
        for blocking_state in self.blocking_states:
            states[blocking_state] = legend['blocking']
        
        # Iterate over reward_states dictionary's items.
        for reward_state, reward in self.reward_states.items():
            states[reward_state] = reward
        
        # Save states as Excel file
        df = pd.DataFrame(states)
        df.to_excel('./gridworld_states.xlsx', index=False, header=False)

        return states
    
    def move_agent(self, action):
        # move agent
        new_agent_pos = [np.array(pos) + self.possible_actions[action] for pos in self.agent_current_pos]

        # check if new position is blocked
        if any(tuple(pos) in self.blocking_states for pos in new_agent_pos):
            return self.agent_current_pos

        # check if new position is out of bounds
        for pos in new_agent_pos:
            if (pos < 0).any() or (pos >= self.world_shape).any():
                return self.agent_current_pos

        return new_agent_pos
    
    def reset(self):
        # reset agent position
        self.agent_current_pos = self.agent_init_pos
        # reset list of collected rewards
        self.collected_rewards = []
            
        # render initial observation
        observation = np.copy(self.agent_current_pos)
        return observation
    
    #TODO: Fit the step method.
    def step(self, action):
        # execute action
        self.agent_current_pos = self.move_agent(action)
        
        reward = 0.0
        done = False
        
        # check if there is any reward
        if tuple(self.agent_current_pos) in self.reward_states.keys() and tuple(self.agent_current_pos) not in self.collected_rewards:
            reward += self.reward_states[tuple(self.agent_current_pos)]
            self.collected_rewards.append(tuple(self.agent_current_pos))
        
        # check if there is any reward and whether the game ended
        if tuple(self.agent_current_pos) in self.terminal_states:
            done = True
            
        # render observation
        observation = np.copy(self.agent_current_pos)
        return observation, reward, done