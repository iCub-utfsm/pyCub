import numpy as np

from pycub import pyCub
from pycub_gym.envs import pyCubEnv

# load the robot with correct world/config
client = pyCub(config="with_ball.yaml")

env = pyCubEnv(client)
env.reset()
observation, reward, done, info = env.step({
"head":      np.double(np.array([0, 0, 0, 0, 0, 0])),
"left_arm":  np.double(np.array([-30, 30, 0, 60, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0])),
"right_arm": np.double(np.array([-30, 30, 0, 45, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0])),
"torso":     np.double(np.array([0, 0, 0]))
})
print(observation)
