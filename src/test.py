import time
import numpy as np
from environment import create_environment
from transform import Affine


def move_robot(actual_pose, direction, robot):

    step = 0.1
    directions = {
        "left": [0, -step, 0],
        "right": [0, step, 0],
        "forward": [-step, 0, 0],
        "backward": [step, 0, 0]
    }
    relative_pose = Affine(translation=directions[direction])
    target_pose = actual_pose * relative_pose
    robot.ptp(target_pose)
    return robot.get_eef_pose()

def main():
    """Main function to create and simulate the environment."""
    bullet_client, robot, gripper, object_ids, obstacle_ids, target_area_id = create_environment(render=True)
    home_pose = robot.get_eef_pose()

    # Simulate the environment for 1000 steps
    for _ in range(100):
        bullet_client.stepSimulation()
        time.sleep(1 / 100)

    for object_id in object_ids:
        # get current object pose
        object_position, object_quat = bullet_client.getBasePositionAndOrientation(object_id)
        object_pose = Affine(object_position, object_quat)

        end_position, end_quat = bullet_client.getBasePositionAndOrientation(target_area_id)
        end_pose = Affine(end_position, end_quat)

        # implement grasping the object
        # keep in mind, that the object pose is defined in the world frame, and the eef points downwards
        # also, make sure that before grasping the gripper is open
        # consider adding a pre-grasp pose to ensure the object is grasped correctly without collision during approach

        gripper_rotation = Affine(rotation=[0, np.pi, 0])
        target_pose = object_pose * gripper_rotation
        pre_grap_offset = Affine(translation=[0, 0, -0.1])
        pre_gasp_pose = target_pose * pre_grap_offset
        robot.ptp(pre_gasp_pose)
        gripper.open()
        robot.lin(target_pose)
        gripper.close()

        # actual_pose = robot.get_eef_pose()
        # actual_pose = move_robot(actual_pose, "left", robot)
        # actual_pose = move_robot(actual_pose, "forward", robot)
        # actual_pose = move_robot(actual_pose, "right", robot)
        # actual_pose = move_robot(actual_pose, "backward", robot)    

        # Move the object to the target area
        end_pose = end_pose * gripper_rotation
        robot.ptp(end_pose)
        gripper.open()

        robot.ptp(home_pose)

    input()
    # Close the simulation
    bullet_client.disconnect()

if __name__ == "__main__":
    main()
