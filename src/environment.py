import time
import pybullet as p
import numpy as np
from pybullet_utils.bullet_client import BulletClient
import cv2

from bullet_env.bullet_robot import BulletRobot, BulletGripper
from transform import Affine

# setup
RENDER = True
URDF_PATH = "/home/jovyan/workspace/assets/urdf/robot.urdf"

bullet_client = BulletClient(connection_mode=p.GUI)
bullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
if not RENDER:
    bullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# Set the camera view to focus on the robot and objects
camera_distance = 1.5  # Distance from the scene
camera_yaw = 90        # Camera yaw (rotation around the vertical axis)
camera_pitch = -30     # Camera pitch (tilt angle)
camera_target_position = [0.6, 0, 0.5]  # Position to look at (near the robot and objects)

bullet_client.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

bullet_client.resetSimulation()

robot = BulletRobot(bullet_client=bullet_client, urdf_path=URDF_PATH)
gripper = BulletGripper(bullet_client=bullet_client, robot_id=robot.robot_id)

# robot commands
# move the robot to the home position instantly, without real execution
robot.home()

# spawn an object
# Define table boundaries based on URDF dimensions
table_x_min, table_x_max = 0.3, 0.9
table_y_min, table_y_max = -0.3, 0.3

# Minimum distance between objects and obstacles
min_distance = 0.05

# Function to check if a position is valid (not colliding with existing objects)
def is_valid_position(new_position, existing_positions, min_distance):
    for pos in existing_positions:
        if np.linalg.norm(np.array(new_position) - np.array(pos)) < min_distance:
            return False
    return True

# Create a random object spawn function for primitive shapes
def spawn_random_primitive(existing_positions):
    while True:
        # Randomly choose an object type
        #object_type = np.random.choice(['box', 'sphere', 'cylinder', 'capsule'])
        object_type = np.random.choice(['box', 'sphere', 'cylinder'])

        # Define random position and scale
        random_translation = [
            np.random.uniform(table_x_min, table_x_max),  # x position
            np.random.uniform(table_y_min, table_y_max), # y position
            0.01  # z position
        ]
        
        if not is_valid_position(random_translation, existing_positions, min_distance):
            continue
        
        scale = np.random.uniform(0.1, 0.5)  # Size scale

        # Define colors
        def generate_non_gray_color():
            while True:
                # Random values for Red, Green, and Blue
                color = [np.random.random(), np.random.random(), np.random.random(), 1]
            
                # Check if the color is not gray
                if not (color[0] == color[1] == color[2] == 0.5):
                    return color
                
        color = generate_non_gray_color()
        
        if object_type == 'box':
            half_extents = np.random.uniform(0.05, 0.1, 3) * scale  # Random size for the box
            collision_shape = bullet_client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            visual_shape = bullet_client.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        elif object_type == 'sphere':
            radius = np.random.uniform(0.05, 0.1) * scale
            collision_shape = bullet_client.createCollisionShape(p.GEOM_SPHERE, radius=radius)
            visual_shape = bullet_client.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
        elif object_type == 'cylinder':
            radius = np.random.uniform(0.05, 0.1) * scale
            height = np.random.uniform(0.1, 0.3) * scale
            collision_shape = bullet_client.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
            visual_shape = bullet_client.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        elif object_type == 'capsule':
            radius = np.random.uniform(0.05, 0.1) * scale
            height = np.random.uniform(0.1, 0.3) * scale
            collision_shape = bullet_client.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)
            visual_shape = bullet_client.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=height, rgbaColor=color)
    
        # Create the object with zero mass (kinematic object)
        object_id = bullet_client.createMultiBody(
            baseMass=10,  # Setting mass to make the object kinematic
            baseCollisionShapeIndex=collision_shape, 
            baseVisualShapeIndex=visual_shape, 
            basePosition=random_translation
        )

        existing_positions.append(random_translation)
        return object_id

# Spawn a random number of objects
num_objects = np.random.randint(2, 5)
object_ids = []
existing_positions = []

for _ in range(num_objects):
    object_id = spawn_random_primitive(existing_positions)
    object_ids.append(object_id)

# Create a random obstacle spawn function for primitive shapes
def spawn_random_obstacle(existing_positions, min_distance):
    while True:
        # Randomly choose an obstacle type
        obstacle_type = np.random.choice(['box', 'cylinder'])
        
        # Define random position and scale
        random_translation = [
            np.random.uniform(table_x_min, table_x_max),  # x position
            np.random.uniform(table_y_min, table_y_max), # y position
            0.1  # z position
        ]
        
        if not is_valid_position(random_translation, existing_positions, min_distance):
            continue
        
        scale = np.random.uniform(0.1, 0.5)  # Size scale
             
        color = [0.5, 0.5, 0.5, 1]  # Gray color
        
        if obstacle_type == 'box':
            half_extents = np.random.uniform(0.05, 0.1, 3) * scale  # Random size for the box
            collision_shape = bullet_client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            visual_shape = bullet_client.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        elif obstacle_type == 'cylinder':
            radius = np.random.uniform(0.05, 0.1) * scale
            height = np.random.uniform(0.1, 0.3) * scale
            collision_shape = bullet_client.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
            visual_shape = bullet_client.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        
        # Create the object with zero mass (kinematic object)
        obstacle_id = bullet_client.createMultiBody(
            baseMass=0,  # Setting mass to make the object kinematic
            baseCollisionShapeIndex=collision_shape, 
            baseVisualShapeIndex=visual_shape, 
            basePosition=random_translation
        )

        existing_positions.append(random_translation)
        return obstacle_id

# Spawn a random number of obstacles
num_obstacles = np.random.randint(8, 15)
obstacle_ids = []

for _ in range(num_obstacles):
    obstacle_id = spawn_random_obstacle(existing_positions, min_distance)
    obstacle_ids.append(obstacle_id)

# Define the target area
target_area_center = [1, 0, 0]  # Center of the target area
target_area_size = [0.15, 0.15, 0]  # Size of the target area (length, width, height)

# Create a visual representation of the target area
target_area_visual_shape = bullet_client.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[size / 2 for size in target_area_size],
    rgbaColor=[0, 1, 0, 0.3]  # Green color with transparency
)

# Create the target area as a static object
target_area_id = bullet_client.createMultiBody(
    baseMass=0,  # Static object
    baseVisualShapeIndex=target_area_visual_shape,
    basePosition=target_area_center
)

# simulate the scene for 1000 steps and wait for the object to settle
for _ in range(1000):
    bullet_client.stepSimulation()
    time.sleep(1 / 100)

# close the simulation
bullet_client.disconnect()
