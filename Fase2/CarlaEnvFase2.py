import glob
import os
import sys
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import carla 
import time
import cv2
import random
import math
from PIL import Image
from gymnasium.envs.registration import register

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


register(
    id='CarlaEnviroment',                                # Call it whatever you want
    entry_point='CarlaEnvFase2:CarlaEnv',                # Module name and class name
)
class CarlaEnv(gym.Env):

    metadata = {"render_modes": ["human"], 'render_fps': 60}

    def __init__(self, render_mode=None):
        
        self.render_mode = render_mode
        self.cliente = carla.Client('localhost', 2000)
        self.world = self.cliente.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.puntosSpawn = self.world.get_map().get_spawn_points()
        self.cache = []
        self.listaActores = []
        self.sensorColision = None
        self.sensorColisionOld = None
        self.sensorColisionb = self.blueprint_library.find('sensor.other.collision')
        self.posicionInicial = None
        self.ultimaPosicion = None
        self.VelocidadVehiculo = 0
        self.FrameActual = None
        self.libre = True

        self.cocheAutonomo = self.spawnearVehiculoAutonomo(self.world, self.blueprint_library)
        
        self.action_space = spaces.Discrete(10)  # Puede ser aceleración, frenado, dirección, etc.

        self.observation_space = spaces.Box(0,255,(3,300,300),np.uint8) # Todas las combinaciones de los  (linea y obstaculos), porque el de colisión es para acabar el episodio


    def reset(self, seed=None, options=None):
        
        super().reset(seed=seed)

        self.moverCochePosicionIncial()
        self.cache = [] #Limpiamos la cache por si acaso
        obs, info = self.get_observation()
        return obs, info #Devuelve informacion al final de cada episodio

    def step(self, action):

        # Convertir la acción en un comando para el coche (ejemplo, movimiento)
        if action == 0:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=1.0, steer=0.0)) #Acelerar
        elif action == 1:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.5, steer=0.0)) #Acelerar 
        elif action == 2:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=0.75)) #Girar 
        elif action == 3:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=0.5)) #Girar 
        elif action == 4:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=0.25)) #Girar 
        elif action == 5:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=-0.25)) #Girar 
        elif action == 6:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=-0.5)) #Girar 
        elif action == 7:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.0, throttle=0.0, steer=-0.75)) #Girar 
        elif action == 8:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=0.5, throttle=0.0, steer=0.0)) #Frenar 
        else:
            self.cocheAutonomo.apply_control(carla.VehicleControl(brake=1.0, throttle=0.0, steer=0.0)) #Frenar

        time.sleep(0.2) #Tiempo entre acciones que toma el coche (0.25 es el tiempo de reaccion de un humano promedio)

        # Obtener la observación actual (por ejemplo, imagen de la cámara)
        obs, info = self.get_observation()

        reward = self.calcularRecompensa()  # Aquí va la lógica de recompensa
        
        # Verificar si el episodio ha terminado (por ejemplo, si el coche ha chocado)
        done = self.terminated()  # Lógica para determinar cuándo se acaba el episodio
        
        
        """"
        # Devolver la información necesaria para el aprendizaje
        
        print("")
        print("Estado: " + info)
        print("Recompensa: " + str(reward))
        print(self.cache) # Para ir viendo como va el entorno
        """

        self.cache = [] #Vaciamos la cache para que no se acumulen los datos
        

        return obs, reward, done, False, info 

    def get_observation(self):

        dic = {"info": "información"}

        while self.libre and self.FrameActual is None:
            time.sleep(0.05)

        obs = self.FrameActual
        self.libre = True

        return obs, dic


    def calcularRecompensa(self):

        acu = self.cocheAutonomo.get_location().distance(self.ultimaPosicion)
        acu += self.VelocidadVehiculo * 3.6 #Le damos los puntos en base a km/h y no m/s

        for elemento in self.cache:
            if elemento == 2: # Linea exterior
                acu -= 5
            elif elemento == 3: # Linea interior
                acu -= 5
            elif elemento == 4: # Linea discontinua
                acu -= 1
            elif elemento == 0: # Colision
                acu -= 40

        self.ultimaPosicion = self.cocheAutonomo.get_location()
        return acu

    def render(self, mode='human'):
        # Renderizar el entorno para visualizarlo (si es necesario)
        info, obs = self.get_observation()
        print(info)

    def close(self):
        print("Cerrando entorno")
        # Limpiar y cerrar los recursos al finalizar
        self.sensorColision.stop()
        self.sensorColision.destroy()
        if self.sensorColisionOld.is_alive:
            self.sensorColisionOld.stop()
            self.sensorColisionOld.destroy()
        
        self.destruirActores()
        print("Cerrando entorno")

    def terminated(self):
        if 0 in self.cache:
            return True
        else:
            return False

    
    #||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    #Funciones para manejar los sensores del coche autonomo
    #||||||||||||||||||||||||||||||||||||||||||||||||||||||||

    def manejarSensorLinea(self, invasion):
        if 2 not in self.cache and 3 not in self.cache and 4 not in self.cache:
            if str(invasion.crossed_lane_markings[0].color) == "Yellow": #Todas las lineas del interior son amarillas
                self.cache.append(3) # Para lineas continuas interiores
                print("Linea interior detectada")
            elif "Solid" == str(invasion.crossed_lane_markings[0].type) or "Grass" == str(invasion.crossed_lane_markings[0].type) or "Curb" == str(invasion.crossed_lane_markings[0].type): #Esto reprersentaria la parte derecha de la  carretera
                self.cache.append(2) # Para todo tipo de linea que no se deberia de poder cruzar, ya sea cualquier tipo de continua o bordillo o hierba
                print("Linea exterior detectada")
            elif "Solid" in str(invasion.crossed_lane_markings[0].type): #Esto representaraia la parte que se encuentra entre los 2 carriles
                self.cache.append(3)
                print("Linea interior detectada")
            else:
                self.cache.append(4) # Para lineas discontinuas
                print("Linea discontinua detectada")
            
            #print("Invasion de linea detectada de tipo: " + str(invasion.crossed_lane_markings[0].type))

    def manejadorColisiones(self, colision):
        if self.sensorColision is not None:
            self.cache.append(0) #Hacemos que el sensor deje de registrar colisiones(Produce ciertos bugs)
            self.sensorColision.stop()
            print("Colision detectada")

    def manejarSensorObstaculos(self, obstaculo):
        if 1 not in self.cache:
            print("Obstaculo detectado")
            self.cache.append(1)    

    def manejarSensorCamara(self, world, vehicle):
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
            carla.Rotation(pitch=-90)))
        
    #Va un poco lagado entonces no lo utilizo, dejas para hacer video futuro
    def procesarImagen(self, imagen): #Para que se vea como circula el coche
        imagen = np.array(imagen.raw_data)
        img = imagen.reshape((600,800,4))
        img2 = img[:,:,:3]

        cv2.imshow("", img2)
        cv2.waitKey(100)

    def manejarSensorLidar(self, lidar):
        data = np.frombuffer(lidar.raw_data, dtype=np.float32)  # Convertir a numpy
        data = np.reshape(data, (-1, 4))  # Cada punto tiene (X, Y, Z, Intensidad)
        print(f"LiDAR recibió {data.shape[0]} puntos")
        print(data)  # Imprimir los primeros 5 puntos para ver el formato

    def manejarSensorSemantico (self, semantico):
        #semantico.save_to_disk("imagenes/" + str(time.time()) + ".png", carla.ColorConverter.CityScapesPalette)
        if self.libre:

            self.libre = False
            # Convertir la imagen del sensor a un array de NumPy
            img_array = np.frombuffer(semantico.raw_data, dtype=np.uint8)
            
            # Redimensionar la imagen según las dimensiones del sensor
            img_array = img_array.reshape((semantico.height, semantico.width, 4))  # Última dimensión: BGRA

            # Convertir a RGB eliminando el canal alfa
            img_array = img_array[:, :, :3]

            # Transponer la imagen para que tenga la forma (N_CHANNELS, HEIGHT, WIDTH)
            img_array = np.transpose(img_array, (2, 0, 1))

            
            self.FrameActual = img_array
            semantico.save_to_disk("imagenes/" + str(time.time()) + ".png", carla.ColorConverter.CityScapesPalette)



    #|||||||||||||||||||||||||||||||||||||||||||||||||||||||
    #Funciones auxiliares
    #|||||||||||||||||||||||||||||||||||||||||||||||||||||||

    def destruirActores(self):
        if len(self.listaActores) > 0:
            for actor in reversed(self.listaActores):
                if actor.is_alive:
                    actor.destroy()
            self.listaActores.clear()
            print("Se ha vaciado toda la lista de actores")
        else:
            print("No hay ningun actor para destruir")


    #Funcion que mueve el coche a la posicion inicial y setea el sensor de colision
    def moverCochePosicionIncial(self):
        print("Moviendo coche a la nueva posicion aleatoria")
        #self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0)) # Frenamos el coche
        
        if self.cocheAutonomo is not None:
            self.cocheAutonomo.set_simulate_physics(False)
            if self.sensorColision is not None: #Si hay un sensor de colision lo destruimos
                self.sensorColisionOld = self.sensorColision
                self.sensorColisionOld.destroy()
            time.sleep(0.5)
            puntoDeSpawn = random.choice(self.puntosSpawn)
            self.cocheAutonomo.set_transform(puntoDeSpawn) # Movemos el coche a una posicion aleatoria
            self.posicionInicial = puntoDeSpawn.location
            self.ultimaPosicion = puntoDeSpawn.location #Para que no de problemas al calcular la recompensa
            time.sleep(1)

            #setear sensor de colision
            self.cocheAutonomo.set_simulate_physics(True)
            self.sensorColision = self.world.try_spawn_actor(self.sensorColisionb, carla.Transform(), attach_to=self.cocheAutonomo) # Añadimos el sensor de colisiones desde aqui porque sino causa problemas
            self.sensorColision.listen(lambda colision: self.manejadorColisiones(colision))
            

        
    def spawnearVehiculoAutonomo(self, world, blueprint_library): #Se le pasa el mundo y la libreria de blueprints para poder spawnear los actores 

        print("Empezamos a spawnear el coche autonomo")

        #Comenzamos spawneado el vehículo, en caso de que no se pueda spawnear en el punto deseado se intentará en otro
        vehiculoAutonomo = None
        for transform in world.get_map().get_spawn_points():
            vehiculoAutonomo = world.try_spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)
            if vehiculoAutonomo is None:
                print("Spawn point ocupado, probando otro...")
            else:
                print("Vehículo spawneado exitosamente")
                break

        self.listaActores.append(vehiculoAutonomo)

        #||||||||||||||||||||||||||||
        #|||||||| Sensores ||||||||||
        #||||||||||||||||||||||||||||

        #Spawneamos camara para ver vehiculo
        camarab = blueprint_library.find('sensor.camera.rgb')
        camarab.set_attribute('image_size_x', '800')
        camarab.set_attribute('image_size_y', '600')
        camarab.set_attribute('fov', '90')
        camara = world.try_spawn_actor(blueprint_library.find('sensor.camera.rgb'),  carla.Transform(carla.Location(x=-7, z=3)), attach_to=vehiculoAutonomo)
        self.listaActores.append(camara)
        if camara is None:
            print("No se ha podido spawnear la camara")
            return None
        else:
            print("Camara spawneada")


        #Spawneamos sensor de colision
        sensorColisionb = blueprint_library.find('sensor.other.collision')
        sensorColision = world.try_spawn_actor(sensorColisionb, carla.Transform(), attach_to=vehiculoAutonomo)
        self.listaActores.append(sensorColision)
        if sensorColision is None:
            print("No se ha podido spawnear el sensor de colision")
            return None
        else:
            print("Sensor de colision spawneado")
            self.sensorColision = sensorColision


        #Spawneamos sensor de invasion de linea
        sensorInvasionb = blueprint_library.find('sensor.other.lane_invasion')
        sensorInvasion = world.try_spawn_actor(sensorInvasionb, carla.Transform(), attach_to=vehiculoAutonomo)
        self.listaActores.append(sensorInvasion)
        if sensorInvasion is None:
            print("No se ha podido spawnear el sensor de invasion de linea")
            return None
        else:
            print("Sensor de invasion de linea spawneado")


        #Spawneamos sensor de obstaculos
        sensorObstaculosb = blueprint_library.find('sensor.other.obstacle')
        sensorObstaculosb.set_attribute('distance', '20')
        sensorObstaculosb.set_attribute('hit_radius', '0')
        sensorObstaculosb.set_attribute('only_dynamics', 'True')
        sensorObstaculosb.set_attribute('sensor_tick', '1.0')
        sensorObstaculos = world.try_spawn_actor(sensorObstaculosb, carla.Transform(), attach_to=vehiculoAutonomo)
        self.listaActores.append(sensorObstaculos)
        if sensorObstaculos is None:
            print("No se ha podido spawnear el sensor de obstaculos")
            return None
        else:
            print("Sensor de obstaculos spawneado")


        #Spawneamos sensor lidar
        sensorLidarb = blueprint_library.find('sensor.lidar.ray_cast')
        sensorLidarb.set_attribute('sensor_tick', '1.0')
        sensorLidar = world.try_spawn_actor(sensorLidarb, carla.Transform(carla.Location(x=0, z=2.5)), attach_to=vehiculoAutonomo)
        self.listaActores.append(sensorLidar)
        if sensorLidar is None:
            print("No se ha podido spawnear el sensor Lidar")
            return None
        else:
            print("Sensor Lidar spawneado")

        #Spawneamos sensor semantico
        sensorSemantico = blueprint_library.find('sensor.camera.semantic_segmentation')
        sensorSemantico.set_attribute('image_size_x', '300')
        sensorSemantico.set_attribute('image_size_y', '300')
        sensorSemantico.set_attribute('fov', '70')
        sensorSemantico = world.try_spawn_actor(sensorSemantico, carla.Transform(carla.Location(x=0, z=2.5)), attach_to=vehiculoAutonomo)
        self.listaActores.append(sensorSemantico)
        if sensorSemantico is None:
            print("No se ha podido spawnear el sensor semantico")
            return None
        else:
            print("Sensor semantico spawneado")


        #||||||||||||||||||||||
        #Activamos los sensores
        #||||||||||||||||||||||

        #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
        camara.listen(lambda image: self.manejarSensorCamara(world, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
        sensorColision.listen(lambda colision: self.manejadorColisiones(colision)) #Para que se imprima por pantalla cuando se detecte una colision
        sensorInvasion.listen(lambda invasion: self.manejarSensorLinea(invasion)) #Para que se imprima por pantalla cuando se detecte una invasion de linea
        sensorObstaculos.listen(lambda obstaculo: self.manejarSensorObstaculos(obstaculo)) #Para que se imprima por pantalla cuando se detecte un obstaculo
        
        #sensorLidar.listen(lambda lidar: env.manejarSensorLidar(lidar)) #Para que se imprima por pantalla cuando se detecte un obstaculo
        sensorSemantico.listen(lambda semantico: self.manejarSensorSemantico(semantico)) #Para que se imprima por pantalla cuando se detecte un obstaculo
    
        return vehiculoAutonomo
        
            
                