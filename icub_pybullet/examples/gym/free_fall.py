
from pycub import pyCub


# load the robot with correct world/config
client = pyCub(config="freebase.yaml")


# just wait until the gui is closed
while client.is_alive():
    client.update_simulation()