import time
import numpy as np
import pandas as pd
from environment import create_environment
from gridworld import GridWorld
from q_learning import QAgent
from train_test import *
from transform import Affine


def move_robot(actual_pose, direction, robot):

    step = 0.012
    directions = {
        0: [step, 0, 0],
        1: [-step, 0, 0],
        2: [0, step, 0],
        3: [0, -step, 0], 
    }

    relative_pose = Affine(translation=directions[direction])
    target_pose = actual_pose * relative_pose
    robot.ptp(target_pose)
    return robot.get_eef_pose()

def render_path(env, path, file_name):
        states = env.render_environment()
        for position in path:
            for pos in position:
                states[tuple(pos)] = 3
        
        df = pd.DataFrame(states)
        file_path = "./rendered_path_" + file_name
        df.to_excel(file_path, index=False, header=False)

def main():
    """Main function to create and simulate the environment."""
    bullet_client, robot, gripper, object_ids, obstacle_ids, target_area_id = create_environment(render=True)
    home_pose = robot.get_eef_pose()

    # Simulate the environment for 1000 steps
    for _ in range(100):
        bullet_client.stepSimulation()
        time.sleep(1 / 100)

    robot.ptp(home_pose)

    # Sort object_ids by the x value of their positions in descending order
    object_ids.sort(key=lambda obj_id: bullet_client.getBasePositionAndOrientation(obj_id["id"])[0][0], reverse=True)

    i = 0     
    steps = 30000
    for object_id in object_ids:
        i = i + 1
        env = GridWorld(bullet_client, object_id, obstacle_ids)
        # env.render_environment()
        obs = env.reset()
        agent = QAgent(len(env.possible_actions), env.world_shape)
        train(agent, env, steps)
        steps += 10000
        df = pd.DataFrame(agent.q_table.argmax(axis=0))
        df.to_excel('./argmax_q.xlsx', index=False, header=False)

        cumulated_reward, path, moves = test(agent, env)
        print('cumulated reward', cumulated_reward)
        file_name = str(i) + ".xlsx"
        render_path(env, path, file_name)

        # get current object pose
        object_position, object_quat = bullet_client.getBasePositionAndOrientation(object_id["id"])
        object_pose = Affine(object_position, object_quat)

        gripper_rotation = Affine(rotation=[0, np.pi, 0])
        target_pose = object_pose * gripper_rotation * Affine(translation=[0, 0, -0.01])
        pre_grap_offset = Affine(translation=[0, 0, -0.1])
        pre_gasp_pose = target_pose * pre_grap_offset
        robot.ptp(pre_gasp_pose)
        gripper.open()
        robot.lin(target_pose)
        gripper.close()

        print(moves)
        actual_pose = robot.get_eef_pose()
        for move in moves:
             actual_pose = move_robot(actual_pose, move, robot) 
             table_x_min, table_x_max = 0.3, 1.05
             table_y_min, table_y_max = -0.3, 0.3
             if not (table_x_min <= actual_pose.translation[0] <= table_x_max and table_y_min <= actual_pose.translation[1] <= table_y_max):
                 break

        gripper.open()

        robot.ptp(home_pose)

    input()
    # Close the simulation
    bullet_client.disconnect()

if __name__ == "__main__":
    main()
