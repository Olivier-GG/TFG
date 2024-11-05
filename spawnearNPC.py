import glob
import os
import sys
import time
import random
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def Spawn (enviroment, blueprints, nCoches, nPeatones):

    listaNPC = []


    for c in range(nCoches):
        transform = random.choice(enviroment.get_map().get_spawn_points())
        vehiculoNPC = enviroment.try_spawn_actor(random.choice(blueprints.filter('vehicle.*.*')), transform)
        listaNPC.append(vehiculoNPC)
        time.sleep(0.5) #Si no se pone no le da tiempo a activarse al autopilot
        vehiculoNPC.set_autopilot(True)

    return listaNPC

        
    


