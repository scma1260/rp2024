import time
import pybullet as p
import numpy as np
from pybullet_utils.bullet_client import BulletClient
import cv2

from bullet_env.bullet_robot import BulletRobot, BulletGripper
from transform import Affine

def create_environment(render=True, urdf_path="/home/jovyan/workspace/assets/urdf/robot.urdf"):
    """
    Creates a simulation environment with a robot, random objects, and obstacles.

    Parameters:
        render (bool): If True, enables rendering in the simulation.
        urdf_path (str): Path to the URDF file of the robot.

    Returns:
        BulletClient: The pybullet client for the simulation.
        BulletRobot: The robot instance.
        list: List of object IDs.
        list: List of obstacle IDs.
    """
    # Initialize pybullet client
    bullet_client = BulletClient(connection_mode=p.GUI)
    bullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    if not render:
        bullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # Set the camera view
    camera_distance = 1.5
    camera_yaw = 180
    camera_pitch = -30
    camera_target_position = [0.6, 0, 0.5]
    bullet_client.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Reset the simulation
    bullet_client.resetSimulation()

    # Create the robot
    robot = BulletRobot(bullet_client=bullet_client, urdf_path=urdf_path)
    gripper = BulletGripper(bullet_client=bullet_client, robot_id=robot.robot_id)

    # Move the robot to the home position
    robot.home()

    # Define table boundaries
    table_x_min, table_x_max = 0.3, 0.9
    table_y_min, table_y_max = -0.3, 0.3
    min_distance = 0.13

    def is_valid_position(new_position, existing_positions, min_distance):
        for pos in existing_positions:
            if np.linalg.norm(np.array(new_position) - np.array(pos)) < min_distance:
                return False
        return True

    def generate_non_gray_color():
        while True:
            color = [np.random.random(), np.random.random(), np.random.random(), 1]
            if not (color[0] == color[1] == color[2] == 0.5):
                return color

    def spawn_random_primitive(existing_positions):
        while True:
            object_type = np.random.choice(['box', 'sphere', 'cylinder'])
            random_translation = [
                np.random.uniform(table_x_min, table_x_max),
                np.random.uniform(table_y_min, table_y_max),
                0
            ]

            if not is_valid_position(random_translation, existing_positions, min_distance):
                continue

            max_size = 0.05
            min_size = 0.012
            color = generate_non_gray_color()

            if object_type == 'box':
                half_extents = np.random.uniform(min_size, max_size, 3)
                collision_shape = bullet_client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
                visual_shape = bullet_client.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
            elif object_type == 'sphere':
                radius = np.random.uniform(min_size, max_size)
                collision_shape = bullet_client.createCollisionShape(p.GEOM_SPHERE, radius=radius)
                visual_shape = bullet_client.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
            elif object_type == 'cylinder':
                radius = np.random.uniform(min_size, max_size)
                height = np.random.uniform(min_size, max_size)
                collision_shape = bullet_client.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
                visual_shape = bullet_client.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)

            object_id = bullet_client.createMultiBody(
                baseMass=0.5,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=random_translation
            )

            existing_positions.append(random_translation)
            return object_id

    def spawn_random_obstacle(existing_positions, min_distance):
        while True:
            obstacle_type = np.random.choice(['box', 'cylinder'])
            random_translation = [
                np.random.uniform(table_x_min, table_x_max),
                np.random.uniform(table_y_min, table_y_max),
                0
            ]

            if not is_valid_position(random_translation, existing_positions, min_distance):
                continue

            max_size = 0.08
            min_size = 0.02
            color = [0.5, 0.5, 0.5, 1]

            if obstacle_type == 'box':
                half_extents = np.random.uniform(min_size, max_size, 3)
                collision_shape = bullet_client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
                visual_shape = bullet_client.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
            elif obstacle_type == 'cylinder':
                radius = np.random.uniform(min_size, max_size)
                height = np.random.uniform(min_size, max_size)
                collision_shape = bullet_client.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
                visual_shape = bullet_client.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)

            obstacle_id = bullet_client.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=random_translation
            )

            existing_positions.append(random_translation)
            return obstacle_id

    # Spawn objects and obstacles
    num_objects = np.random.randint(2, 5)
    object_ids = []
    existing_positions = []

    for _ in range(num_objects):
        object_id = spawn_random_primitive(existing_positions)
        object_ids.append(object_id)

    num_obstacles = np.random.randint(5, 8)
    obstacle_ids = []

    for _ in range(num_obstacles):
        obstacle_id = spawn_random_obstacle(existing_positions, min_distance)
        obstacle_ids.append(obstacle_id)

    # Define the target area
    target_area_center = [1, 0, 0]
    target_area_size = [0.15, 0.15, 0]

    target_area_visual_shape = bullet_client.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size / 2 for size in target_area_size],
        rgbaColor=[0, 1, 0, 0.3]
    )

    target_area_id = bullet_client.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=target_area_visual_shape,
        basePosition=target_area_center
    )

    return bullet_client, robot, gripper, object_ids, obstacle_ids, target_area_id
