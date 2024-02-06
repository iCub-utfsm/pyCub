from gymnasium.envs.registration import register

register(
    id='pyCub-v0', 
    entry_point='pycub_gym.envs:pyCubEnv'
)