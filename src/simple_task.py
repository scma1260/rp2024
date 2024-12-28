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
object_urdf_path = "/home/jovyan/workspace/assets/objects/cube/object.urdf"
object_pose = Affine(translation=[0.5, 0, 0.1])
object_id = bullet_client.loadURDF(
    object_urdf_path,
    object_pose.translation,
    object_pose.quat,
    flags=bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

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

#random_noise = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
#cv2.imshow("random noise", random_noise)
#cv2.waitKey(0)

robot.ptp(home_pose)

# close the simulation
bullet_client.disconnect()
