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

# get the current end effector pose
home_pose = robot.get_eef_pose()
print(home_pose)

# create a relative pose to the current end effector pose
relative_pose = Affine(translation=[0, 0, 0.1])
# apply the relative pose to the current end effector pose (in the eef frame)
target_pose = home_pose * relative_pose
# move to the target pose
robot.ptp(target_pose)

# per default, the z axis of the eef frame is pointing downwards
# applying the same relative transformation to the target pose in the base frame
current_pose = robot.get_eef_pose()
target_pose = relative_pose * current_pose
# now with linear motion
robot.lin(target_pose)

# open the gripper
gripper.open()
# close the gripper
gripper.close()

# spawn an object
# Define table boundaries based on URDF dimensions
table_x_min, table_x_max = 0.3, 0.9
table_y_min, table_y_max = -0.3, 0.3

# Create a random object spawn function for primitive shapes
def spawn_random_primitive():
    # Randomly choose an object type
    object_type = np.random.choice(['box', 'sphere', 'cylinder', 'capsule'])
    
    # Define random position and scale
    random_translation = [
        np.random.uniform(table_x_min, table_x_max),  # x position
        np.random.uniform(table_y_min, table_y_max), # y position
        0.1  # z position
    ]
    
    scale = np.random.uniform(0.1, 0.5)  # Size scale

    # Define colors
    color = [np.random.random(), np.random.random(), np.random.random(), 1]  # RGBA
    
    if object_type == 'box':
        half_extents = np.random.uniform(0.05, 0.1, 3) * scale  # Random size for the box
        collision_shape = bullet_client.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    elif object_type == 'sphere':
        radius = np.random.uniform(0.05, 0.1) * scale
        collision_shape = bullet_client.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    elif object_type == 'cylinder':
        radius = np.random.uniform(0.05, 0.1) * scale
        height = np.random.uniform(0.1, 0.3) * scale
        collision_shape = bullet_client.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    elif object_type == 'capsule':
        radius = np.random.uniform(0.05, 0.1) * scale
        height = np.random.uniform(0.1, 0.3) * scale
        collision_shape = bullet_client.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)

    # Create the visual shape (for appearance)
    visual_shape = bullet_client.createVisualShape(shapeType=p.GEOM_MESH, fileName="", rgbaColor=color)
    
    # Create the object with zero mass (kinematic object)
    object_id = bullet_client.createMultiBody(
        baseMass=0.2,  # Setting mass to make the object kinematic
        baseCollisionShapeIndex=collision_shape, 
        baseVisualShapeIndex=visual_shape, 
        basePosition=random_translation
    )

    return object_id

# Spawn a random number of objects
num_objects = np.random.randint(2, 5)
object_ids = []

for _ in range(num_objects):
    object_id = spawn_random_primitive()
    object_ids.append(object_id)
    object_id = object_id

    
# simulate the scene for 100 steps and wait for the object to settle
for _ in range(100):
    bullet_client.stepSimulation()
    time.sleep(1 / 100)

# get current object pose
position, quat = bullet_client.getBasePositionAndOrientation(object_id)
object_pose = Affine(position, quat)

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

robot.ptp(home_pose)

# close the simulation
bullet_client.disconnect()
