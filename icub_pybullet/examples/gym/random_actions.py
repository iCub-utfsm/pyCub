import numpy as np

from pycub import pyCub
from pycub_gym.envs import pyCubEnv
import random

# load the robot with correct world/config
client = pyCub(config="with_ball.yaml")

env = pyCubEnv(client)

#devuelve array de posiciones aleatorias para cada joint en joints_names, considerando los límites de posición que están en joint_dict
# Lista para almacenar los números aleatorios
random_numbers_list = []
# Generar números aleatorios para cada articulación y almacenarlos en la lista
for joint_name in env.joints_names:
    # Obtener los límites de la articulación desde el diccionario
    lower_limit, upper_limit = env.joint_dict.get(joint_name, (0, 0))

    # Generar un número aleatorio dentro de los límites
    random_number = random.uniform(lower_limit, upper_limit)

    # Agregar el número aleatorio a la lista
    random_numbers_list.append(random_number)        

env.reset()
observation, reward, done, info = env.step(random_numbers_list)

# observation, reward, done, info = env.step({
# "head":      np.double(np.array([0, 0, 0, 0, 0, 0])),
# "left_arm":  np.double(np.array([-30, 30, 0, 60, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0])),
# "right_arm": np.double(np.array([-30, 30, 0, 45, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0])),
# "torso":     np.double(np.array([0, 0, 0]))
# })
print(observation)
