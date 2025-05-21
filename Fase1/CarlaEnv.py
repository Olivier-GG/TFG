import gymnasium as gym
from gymnasium import spaces
import numpy as np
import carla 
import time
import cv2
import random
import math

class CarlaEnv(gym.Env):
    def __init__(self, client):
        super(CarlaEnv, self).__init__()

        self.cliente = client
        self.world = self.cliente.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.puntosSpawn = self.world.get_map().get_spawn_points()
        self.cache = []
        self.cocheAutonomo = None
        self.sensorColision = None
        self.sensorColisionOld = None
        self.sensorColisionb = self.blueprint_library.find('sensor.other.collision')
        self.posicionInicial = None
        self.ultimaPosicion = None
        self.VelocidadVehiculo = 0
        
        self.action_space = spaces.Discrete(10)  # Espacio de acción discreto (acelerar, frenar, etc.)
        self.observation_space = spaces.Discrete(20) # Espacio de observación discreto (todo correcto, obstaculo, linea continua interior, etc.)


    def reset(self):
        
        self.moverCochePosicionIncial()
        self.cache = [] #Limpiamos la cache
        return self.get_observation()

    def step(self, action):

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

        time.sleep(0.2) #Tiempo entre que se toma la accion y se obtiene la observacion

        info, state = self.get_observation()

        if state == 16: #Si es este estado no puede haber recompensa negativa por chocar, es un error a la hora de respawnear
            reward = 0
        else:
            reward = self.calcularRecompensa() 
        
        done = self.terminated()  ## Comprobar si el episodio ha terminado

        self.cache = [] #Limpiamos la cache
        
        return info, state, reward, done


    def get_observation(self):

        self.VelocidadVehiculo = math.sqrt(self.cocheAutonomo.get_velocity().x**2 + self.cocheAutonomo.get_velocity().y**2) # Obtenemos la velocidad del coche autonomo en m/s
        
        if 1 in self.cache:

            if self.VelocidadVehiculo <= 0.5:
                return "obstaculo detectado, Parado, S0", 0
            elif self.VelocidadVehiculo > 0.5 and self.VelocidadVehiculo < 3:
                return "obstaculo detectado, < 10kmh, S1", 1
            elif self.VelocidadVehiculo >= 3 and self.VelocidadVehiculo < 9:
                return "obstaculo detectado, 10 < 30, S2", 2
            else:
                return "obstaculo detectado, >30 , S3", 3
        
        elif 2 in self.cache:

            if self.VelocidadVehiculo <= 0.5:
                return "linea continua exterior detectada, Parado, S4", 4
            elif self.VelocidadVehiculo > 0.5 and self.VelocidadVehiculo < 3:
                return "linea continua exterior detectada, < 10kmh, S5", 5
            elif self.VelocidadVehiculo >= 3 and self.VelocidadVehiculo < 9:
                return "linea continua exterior detectada, 10 < 30, S6", 6
            else:
                return "linea continua exterior detectada, >30 , S7", 7
        
        elif 3 in self.cache:

            if self.VelocidadVehiculo <= 0.5:
                return "linea continua interior detectada, Parado, S8", 8
            elif self.VelocidadVehiculo > 0.5 and self.VelocidadVehiculo < 3:
                return "linea continua interior, < 10kmh, S9", 9
            elif self.VelocidadVehiculo >= 3 and self.VelocidadVehiculo < 9:
                return "linea continua interior, 10 < 30, S10", 10
            else:
                return "linea continua interior, >30 , S11", 11
            
        elif 4 in self.cache:

            if self.VelocidadVehiculo <= 0.5:
                return "linea discontinua detectada, Parado, S12", 12
            elif self.VelocidadVehiculo > 0.5 and self.VelocidadVehiculo < 3:
                return "linea discontinua detectada, < 10kmh, S13", 13
            elif self.VelocidadVehiculo >= 3 and self.VelocidadVehiculo < 9:
                return "linea discontinua detectada, 10 < 30, S14", 14
            else:
                return "linea discontinua detectada, >30 , S15", 15
        
        else:

            if self.VelocidadVehiculo <= 0.5:
                return "Todo correcto, Parado, S16", 16
            elif self.VelocidadVehiculo > 0.5 and self.VelocidadVehiculo < 3:
                return "Todo correcto, < 10kmh, S17", 17
            elif self.VelocidadVehiculo >= 3 and self.VelocidadVehiculo < 9:
                return "Todo correcto, 10 < 30, S18", 18
            else:
                return "Todo correcto, >30 , S19", 19


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

    def terminated(self):
        #Si se ha detectado una colision devolvemos True
        if 0 in self.cache:
            return True
        else:
            return False
        

    def render(self, mode='human'):
        pass


    def close(self):
        #Destruimos los sensores que hemos creado antes de cerrar el entorno
        self.sensorColision.stop()
        self.sensorColision.destroy()
        if self.sensorColisionOld.is_alive:
            self.sensorColisionOld.stop()
            self.sensorColisionOld.destroy()
        print("Cerrando entorno")


    #||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    #|||||||| Funciones para manejar los sensores del coche autonomo ||||||||||
    #||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

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
        

    #No se utiliza, pero se deja por si se quiere ver en 3 persona como circula el vehiculo
    def procesarImagen(self, imagen):
        imagen = np.array(imagen.raw_data)
        img = imagen.reshape((600,800,4))
        img2 = img[:,:,:3]

        cv2.imshow("", img2)
        cv2.waitKey(100)



    #||||||||||||||||||||||||||||||||||||||||
    #|||||||| Funciones auxiliares ||||||||||
    #||||||||||||||||||||||||||||||||||||||||

    #Setter coche autonomo
    def setCocheAutonomo(self, vehiculo):
        self.cocheAutonomo = vehiculo
        self.ultimaPosicion = self.cocheAutonomo.get_location()
        self.posicionInicial = self.cocheAutonomo.get_location()


    #Funcion que mueve el coche a la posicion inicial y setea el sensor de colision
    def moverCochePosicionIncial(self):
        print("Moviendo coche a la nueva posicion aleatoria")
        self.cocheAutonomo.set_simulate_physics(False)
        if self.cocheAutonomo is not None:

            if self.sensorColision is not None: #Si hay un sensor de colision lo destruimos
                self.sensorColisionOld = self.sensorColision
                self.sensorColisionOld.destroy()
            
            time.sleep(0.5)

            puntoDeSpawn = random.choice(self.puntosSpawn)
            self.cocheAutonomo.set_transform(puntoDeSpawn) # Movemos el coche a una posicion aleatoria
            self.posicionInicial = puntoDeSpawn.location
            self.ultimaPosicion = puntoDeSpawn.location
            
            time.sleep(1)

            self.cocheAutonomo.set_simulate_physics(True)
            self.sensorColision = self.world.try_spawn_actor(self.sensorColisionb, carla.Transform(), attach_to=self.cocheAutonomo) # Añadimos el sensor de colisiones desde aqui porque sino causa problemas
            self.sensorColision.listen(lambda colision: self.manejadorColisiones(colision))
            


    
        
            