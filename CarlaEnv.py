import glob
import os
import sys
import gym
from gymnasium import spaces
import numpy as np
import carla 

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



class CarlaEnv(gym.Env):
    def __init__(self, cliente):
        super(CarlaEnv, self).__init__()

        cliente.set_timeout(5.0)
        enviroment = cliente.get_world()
        #enviroment = cliente.load_world('Town01')
        blueprint_library = enviroment.get_blueprint_library()

        self.action_space = spaces.Discrete(4)  # Puede ser aceleración, frenado, dirección, etc.

        # Definir el espacio de observación (puede ser una imagen de cámara, por ejemplo)
        self.observation_space = spaces.Box(low=0, high=255, shape=(240, 320, 3), dtype=np.uint8)

    def reset(self):
        # Restablecer el coche y el entorno a un estado inicial
        self.vehicle.set_location(carla.Location(x=0, y=0, z=1))
        return self.get_observation()

    def step(self, action):
        # Convertir la acción en un comando para el coche (ejemplo, movimiento)
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0)) #Acelerar
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=1.0)) #Girar 
        elif action == 2:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=-1.0)) #Girar
        else:
            self.vehicle.apply_control(carla.VehicleControl(throttle=-1.0, steer=0.0)) #Frenar

        # Obtener la observación actual (por ejemplo, imagen de la cámara)
        observation = self.get_observation()

        # Calcular la recompensa (esto depende de tu objetivo específico)
        reward = -1  # Aquí va la lógica de recompensa, ejemplo simple

        # Verificar si el episodio ha terminado (por ejemplo, si el coche ha chocado)
        done = False  # Lógica para determinar cuándo se acaba el episodio

        return observation, reward, done, {}

    def get_observation(self):
        # Aquí puedes obtener una imagen de la cámara u otros datos
        image = np.zeros((240, 320, 3), dtype=np.uint8)  # Imagen de ejemplo
        return image

    def render(self, mode='human'):
        # Renderizar el entorno para visualizarlo (si es necesario)
        pass

    def close(self):
        # Limpiar y cerrar los recursos al finalizar
        self.vehicle.destroy()
