import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pycub import pyCub
from pycub_gym.envs import pyCubEnv
import random

# load the robot with correct world/config
client = pyCub(config="freebase.yaml")

env = pyCubEnv(client)

# Definir el número máximo de pasos antes de realizar un reset
max_steps = 10
steps_counter = 0

# Bucle para ejecutar acciones aleatorias y renderizar
while True:
    # Incrementar el contador de pasos
    steps_counter += 1
    
    # Realizar un reset si se alcanza el número máximo de pasos
    if steps_counter > max_steps:
        env.reset()
        steps_counter = 0
    
    # Lista para almacenar los números aleatorios para las articulaciones
    random_numbers_list = []
    
    # Generar números aleatorios para cada articulación y almacenarlos en la lista
    for joint_name in env.joints_names:
        # Obtener los límites de la articulación desde el diccionario
        lower_limit, upper_limit = env.joint_dict.get(joint_name, (0, 0))

        # Generar un número aleatorio dentro de los límites
        random_number = random.uniform(lower_limit, upper_limit)

        # Agregar el número aleatorio a la lista
        random_numbers_list.append(random_number)        

    # Ejecutar un paso en el entorno con la acción aleatoria
    observation, reward, _, _,_ = env.step(random_numbers_list)
    print(observation)
    
    # Renderizar el entorno
    env.render()