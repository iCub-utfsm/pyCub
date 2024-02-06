import gymnasium as gym
import numpy as np


class pyCubEnv(gym.Env):
    metadata = {'render.modes': ['human']}  
  
    joints_names = ["r_hip_pitch","r_hip_roll","r_hip_yaw",
                    "r_knee",
                    "r_ankle_pitch","r_ankle_roll",
                    "l_hip_pitch","l_hip_roll","l_hip_yaw",
                    "l_knee",
                    "l_ankle_pitch","l_ankle_roll",
                    "torso_pitch","torso_roll","torso_yaw",
                    "r_shoulder_pitch","r_shoulder_roll","r_shoulder_yaw",
                    "r_elbow",
                    "r_wrist_prosup","r_wrist_pitch","r_wrist_yaw",
                    "l_shoulder_pitch","l_shoulder_roll","l_shoulder_yaw",
                    "l_elbow",
                    "l_wrist_prosup","l_wrist_pitch","l_wrist_yaw",
                    "neck_pitch","neck_roll","neck_yaw"]
    

    def __init__(self,client):
        self.client = client
        joints_limits = self.client.joints
        for joint in self.joints_names:
            robot_joint_id, joint_id = self.client.find_joint_id(joint)
        pass

    def step(self, action):

        # self.client.move_position()
        pass

    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        pass    
    
    def seed(self, seed=None): 
        pass

