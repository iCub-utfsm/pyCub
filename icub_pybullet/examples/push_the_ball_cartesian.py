"""
Example of moving the robot in cartesian space to push the ball. It is more robust than the pure joint control.

:Author: Lukas Rustler
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pycub import pyCub
from utils import Pose


def push_the_ball():
    """
    Function to move the ball with cartesian control. The robot is moved 15cm lower and 10cm closer and the moved left to
    push the ball.

    """
    # Get current pose
    cur_pose = client.end_effector.get_position()

    # Copy, the pose and go 15cm lower and 10cm closer to the robot
    new_pose = Pose(cur_pose.pos, cur_pose.ori)
    new_pose.pos[2] -= 0.15
    new_pose.pos[0] -= 0.1

    # move
    client.move_cartesian(new_pose)

    # get current pose
    pose = client.end_effector.get_position()
    # assign straight to it to move hand left
    pose.pos[1] -= 0.2
    # move; do not wait for completion; and move a bit faster
    client.move_cartesian(pose, wait=False, velocity=2)
    # wait manually
    while not client.motion_done():
        client.update_simulation()

    """
    Wait could be also achieved with:
    client.wait_motion_done()
    """
    print(client.end_effector.get_position())
    client.logger.info("Moved the ball!")


if __name__ == "__main__":
    # load the robot with correct world/config
    client = pyCub(config="with_ball.yaml")
    push_the_ball()
    # delete URDF
    client.removeBody(client.robot)
    # load URDF with initial positions
    client.robot, client.joints, client.links = client.init_robot()
    print(len(client.get_camera_images()[0])) #number of rows
    print(len(client.get_camera_images()[0][1])) #number of pixels in a row
    print(len(client.get_camera_images()[0][1][0])) #a pixel (rgb)

    push_the_ball()
    
    # just wait until the gui is closed
    # while client.is_alive():
    #     client.update_simulation()
