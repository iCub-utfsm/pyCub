import gymnasium as gym
import numpy as np
from gymnasium import spaces
import time

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
        # DICCIONARIO DE TODOS LOS JOINTS DEL ICUB CON SUS RESPECTIVOS LÍMITES
        self.joint_dict = {}
        for joint in joints_limits:
            self.joint_dict[joint.name] = (joint.lower_limit,joint.upper_limit)
        # DICCIONARIO DE LOS LÍMITES DE LOS JOINTS QUE NOS INTERESAN (joints_names)
        selected_joint_limits = {}
        for joint_name in self.joints_names:
            if joint_name in self.joint_dict:
                selected_joint_limits[joint_name] = self.joint_dict[joint_name]
        # DEFINICIÓN DE ESPACIOS
        # orden of the joints: ["right_leg","left_leg","torso","right_arm","left_arm","neck"]
        self.orden = [["r_hip_pitch","r_hip_roll","r_hip_yaw","r_knee","r_ankle_pitch","r_ankle_roll"],["l_hip_pitch","l_hip_roll","l_hip_yaw","l_knee","l_ankle_pitch","l_ankle_roll"],["torso_pitch","torso_roll","torso_yaw"],["r_shoulder_pitch","r_shoulder_roll","r_shoulder_yaw","r_elbow","r_wrist_prosup","r_wrist_pitch","r_wrist_yaw"],["l_shoulder_pitch","l_shoulder_roll","l_shoulder_yaw","l_elbow","l_wrist_prosup","l_wrist_pitch","l_wrist_yaw"],["neck_pitch","neck_roll","neck_yaw"]]
                

        #límites
        leg_low = [-0.785398163397, -0.349065850399, -1.3962634016, -2.16420827247, -0.610865238198, -0.436332312999]
        torso_low = [-0.349065850399, -0.523598775598, -0.872664625997]
        arm_low = [-1.66678943565, 0.0, -0.645771823238, 0.261799387799, -1.0471975512, -1.3962634016, -0.349065850399]
        neck_low = [-0.698131700798, -0.349065850399, -0.872664625997]

        leg_high = [2.33874119767, 2.09439510239, 1.3962634016, 0.0698131700798, 0.610865238198, 0.436332312999]
        torso_high = [1.2217304764, 0.523598775598, 0.872664625997]
        arm_high = [0.174532925199, 2.80648943721, 1.3962634016, 1.85004900711, 1.0471975512, 0.436332312999, 0.436332312999]
        neck_high = [0.383972435439, 0.349065850399, 0.872664625997]
        # Definición de las categorías de articulaciones
 
        self.observation_space = spaces.Dict({
        "joints": spaces.Dict({
            "right_leg": spaces.Box(
            low= np.double(np.array(leg_low)),
            high=np.double(np.array(leg_high)),
            dtype=np.double
            ),
            "left_leg": spaces.Box(
            low= np.double(np.array(leg_low)),
            high=np.double(np.array(leg_high)),
            dtype=np.double
            ),
            "torso": spaces.Box(
            low= np.double(np.array(torso_low)),
            high=np.double(np.array(torso_high)),
            dtype=np.double
            ),
            "right_arm": spaces.Box(
            low= np.double(np.array(arm_low)),
            high=np.double(np.array(arm_high)),
            dtype=np.double
            ),
            "left_arm": spaces.Box(
            low= np.double(np.array(arm_low)),
            high=np.double(np.array(arm_high)),
            dtype=np.double
            ),
            "neck": spaces.Box(
            low= np.double(np.array(neck_low)),
            high=np.double(np.array(neck_high)),
            dtype=np.double
            ),
        }),
            "skin": spaces.Dict({
                # ...
            }),
            "touch": spaces.Dict({
                # ...
            })
        })
        print(self.observation_space)

        self.last_ctrl_step = time.time()
        pass

    def step(self, action):
        # self.client.move_position()
        sleep_duration=1
        start_time = time.time()
        # the step runs with the frecuency declared in "sleep_duration"
        while time.time() - start_time <  sleep_duration:
            self.client.update_simulation(0.00025)
        positions = action
        # self.client.move_position(self.joints_names, positions, wait=False, velocity=1, set_col_state=True, check_collision=True)
        self.client.move_position(self.joints_names, positions, wait=False, velocity=1, set_col_state=True, check_collision=True)
        obs = self.get_obs()
        terminated = False #false for now, we want to try step() first
        reward = 1 if terminated else 0  # binary sparse rewards
        #info is missing
        self.client.update_simulation(0.00025)
        done = None
        info = None
        return obs,reward,terminated,done,info

    def reset(self, seed=None, options=None):
        self.client.removeBody(self.client.robot)
        self.client.robot, self.client.joints, self.client.links = self.client.init_robot()
        super().reset(seed=seed)
        obs = self.get_obs()
        info = None
        return obs, info

    def render(self):
        self.client.update_simulation(0.00025)

    def close(self):
        pass    
    
    def seed(self, seed=None): 
        pass

########################################
    def get_obs(self):
        observation = {
            "joints": {
                "right_leg":[],
                "left_leg":[],
                "torso":[],
                "right_arm":[],
                "left_arm":[],
                "neck":[],
            },
            "joints_velocity": {
                "right_leg":[],
                "left_leg":[],
                "torso":[],
                "right_arm":[],
                "left_arm":[],
                "neck":[],
            },
            "eyes": {
                "left_eye":[],
                "right_eye":[]
            },
            "effector_pose": np.zeros(7)
        }
        i = 0
        #following the orden of the observation space defined in innit
        for joint in observation["joints"]:
            # Obtener el estado del joint del cliente
            joint_state = self.client.get_joint_state(self.orden[i])
            #joint_velocity = self.client.get_joint_velocity(self.orden[i])
            # Convertir la lista en un arreglo NumPy
            joint_state_np = np.array(joint_state)
            #joint_velocity_np = np.array(joint_velocity)
            # Asignar el arreglo NumPy al diccionario de observación
            observation["joints"][joint] = joint_state_np
            #observation["joints_velocity"][joint_velocity] = joint_velocity_np
            i += 1

        pos = np.array(self.client.end_effector.get_position().pos)
        ori = np.array(self.client.end_effector.get_position().ori)
        #falta hacer que las ventanas de los ojos se muestren siempre, si no, salta error
        observation["eyes"]["left_eye"] = self.client.get_camera_images()[0]
        observation["eyes"]["right_eye"] = self.client.get_camera_images()[1]
        observation["effector_pose"] = np.concatenate([pos,ori])
        return observation
    
    def _update_control(self, joint_pos, sleep_duration=0.001):
        
        if sleep_duration is None or time.time()-self.last_ctrl_step > sleep_duration:
            self.client.move_position(self.joints_names, joint_pos, wait=False, velocity=1, set_col_state=True, check_collision=True)
            self.last_ctrl_step = time.time()
