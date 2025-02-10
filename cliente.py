import glob
import os
import sys
import random
from spawn_npc import spawnearCoches
from spawnearNPC import Spawn
import numpy as np
import carla
import glob
import os
import sys
import time
import tqdm
from typing_extensions import Literal
import pdb
import json
import gymnasium as gym
from gymnasium import spaces

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



# |||||| Listas con todos los actores ||||||||

listaActores = [] #Se añaden todos los actores
listaCocheAutonomo = [] #Se añade todos los sensores del coche autonomo y el coche autonomo para poder ser borrados
listaNPC = [] #Se añaden todos los sensores de los NPC y los NPC para poder ser borrados



# ||||||| Funcion para spawnear el coche autonomo y añadirle los sensores necesarios ||||||||||

def spawnearVehiculoAutonomo (world, blueprint_library, cache, env): #Se le pasa el mundo y la libreria de blueprints para poder spawnear los actores 

    print("Empezamos a spawnear el coche autonomo")
    #Donde vamos a respawnear el coche
    #transform = world.get_map().get_spawn_points()[0]
    #transform = random.choice(world.get_map().get_spawn_points())# para que el sitio de respawn sea random
    #Spawneamos el vehiculo
    #vehiculoAutonomo = world.spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)

    vehiculoAutonomo = None
    for transform in world.get_map().get_spawn_points():
        vehiculoAutonomo = world.try_spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)
        if vehiculoAutonomo is None:
            print("Spawn point ocupado, probando otro...")
        else:
            print("Vehículo spawneado exitosamente")
            break

    listaActores.append(vehiculoAutonomo)
    listaCocheAutonomo.append(vehiculoAutonomo)
    if vehiculoAutonomo is None:
        print("No se ha podido spawnear el vehiculo autonomo")
        return None
    else:
        print("coche autonomo spawneado")
    #|||||||| Sensores ||||||||||

    #Spawneamos camara para ver vehiculo
    camarab = blueprint_library.find('sensor.camera.rgb')
    camarab.set_attribute('image_size_x', '800')
    camarab.set_attribute('image_size_y', '600')
    camarab.set_attribute('fov', '90')
    camara = world.try_spawn_actor(blueprint_library.find('sensor.camera.rgb'),  carla.Transform(carla.Location(x=-7, z=3)), attach_to=vehiculoAutonomo)
    listaActores.append(camara)
    listaCocheAutonomo.append(camara)
    if camara is None:
        print("No se ha podido spawnear la camara")
        return None
    else:
        print("Camara spawneada")
    #Spawneamos sensor de colision
    sensorColisionb = blueprint_library.find('sensor.other.collision')
    sensorColision = world.try_spawn_actor(sensorColisionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorColision)
    listaCocheAutonomo.append(sensorColision)
    if sensorColision is None:
        print("No se ha podido spawnear el sensor de colision")
        return None
    else:
        print("Sensor de colision spawneado")
    #Spawneamos sensor de invasion de linea
    sensorInvasionb = blueprint_library.find('sensor.other.lane_invasion')
    sensorInvasion = world.try_spawn_actor(sensorInvasionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorInvasion)
    listaCocheAutonomo.append(sensorInvasion)
    if sensorInvasion is None:
        print("No se ha podido spawnear el sensor de invasion de linea")
        return None
    else:
        print("Sensor de invasion de linea spawneado")
    #Spawneamos sensor de obstaculos
    sensorObstaculosb = blueprint_library.find('sensor.other.obstacle')
    sensorObstaculosb.set_attribute('distance', '15')
    sensorObstaculosb.set_attribute('hit_radius', '4')
    sensorObstaculosb.set_attribute('only_dynamics', 'True')
    sensorObstaculosb.set_attribute('sensor_tick', '1.0')
    sensorObstaculos = world.try_spawn_actor(sensorObstaculosb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorObstaculos)
    listaCocheAutonomo.append(sensorObstaculos)
    if sensorObstaculos is None:
        print("No se ha podido spawnear el sensor de obstaculos")
        return None
    else:
        print("Sensor de obstaculos spawneado")
    #Activamos los sensores

    #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
    camara.listen(lambda image: env.manejarSensorCamara(world, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
    #sensorColision.listen(lambda colision: env.manejadorColisiones(colision)) #Para que se imprima por pantalla cuando se detecte una colision
    sensorInvasion.listen(lambda invasion: env.manejarSensorLinea(invasion)) #Para que se imprima por pantalla cuando se detecte una invasion de linea
    sensorObstaculos.listen(lambda obstaculo: env.manejarSensorObstaculos(obstaculo)) #Para que se imprima por pantalla cuando se detecte un obstaculo
    if sensorColision is None or sensorInvasion is None or sensorObstaculos is None:
        print("No se han podido activar los sensores")
        return None
    else:
        print("Vehiculo con sensores spawneado")

    return vehiculoAutonomo





# |||||| Todas funciones para procesar las información de los sensores ||||||||

"""""""""
#Va un poco lagado entonces no lo utilizo, dejas para hacer video futuro

def procesarImagen(imagen): #Para que se vea como circula el coche
    imagen = np.array(imagen.raw_data)
    img = imagen.reshape((600,800,4))
    img2 = img[:,:,:3]

    cv2.imshow("", img2)
    cv2.waitKey(100)

"""""""""


# |||||||| Funciones auxiliares ||||||||||

# Guardar en JSON
def guardar_qtable(q_table, filename):
    with open("TablasFase1/" + filename + ".json", "w") as f:
        json.dump(q_table.tolist(), f)

# Cargar desde JSON
def cargar_qtable(filename="qtable.json"):
    try:
        with open(filename, "r") as f:
            return np.array(json.load(f))
    except FileNotFoundError:
        print("No se encontró el archivo de la tabla Q, creando una nueva.")
        return np.zeros((6, 4))  # Ajusta el tamaño


def initialize_q_table(state_space, action_space):
  Qtable = np.zeros((state_space, action_space))
  return Qtable

def greedy_policy(Qtable, state):
  # Exploitation: take the action with the highest state, action value
  action = np.argmax(Qtable[state][:])

  return action

def epsilon_greedy_policy(Qtable, state, epsilon, env):
  # Randomly generate a number between 0 and 1
  random_num = random.uniform(0,1)
  # if random_num > greater than epsilon --> exploitation
  if random_num > epsilon:
    # Take the action with the highest value given a state
    # np.argmax can be useful here
    action = greedy_policy(Qtable, state)
  # else --> exploration
  else:
    action = env.action_space.sample()

  return action




# |||||||||||||| MAIN |||||||||||||||||
def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor
        
        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(5.0)
        #cliente.load_world('Town01')
        world = cliente.get_world()
        blueprint_library = world.get_blueprint_library()

        #Incializamos el entorno de gym
        env = CarlaEnv(cliente)

        state_space = env.observation_space.n
        action_space = env.action_space.n

        #Inicializamos Qtable
        Qtable = initialize_q_table(state_space, action_space)

        print("Conexion con el servidor establecida y todas las variables principales inicializadas")

        #|||||||||||||||||| Parametros para el entrenamiento |||||||||||||||||

        # Training parameters
        n_training_episodes = 1000  # Total training episodes
        learning_rate = 0.1          # Learning rate

        # Evaluation parameters
        n_eval_episodes = 100        # Total number of test episodes

        # Environment parameters
        max_steps = 150            # Max steps per episode
        gamma = 0.95                 # Discounting rate
        eval_seed = []               # The evaluation seed of the environment

        # Exploration parameters
        max_epsilon = 1.0             # Exploration probability at start
        min_epsilon = 0.05            # Minimum exploration probability
        decay_rate = 0.0005            # Exponential decay rate for exploration prob

        #|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        #print("\nProcedo a spawnear 30 coches y 10 peatones")
        #listaActores.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla 
        #listaNPC.extend(spawnearCoches(30,10)) 
        #listaActores.extend(Spawn(enviroment,blueprint_library,10,10)) #Codigo hecho por mi

        #|||| Paso 3, comienza el entrenamiento ||||||||||
        
        print("               Comenzando el entrenamiento        ")

        for episode in range(n_training_episodes):
            # Reduce epsilon (because we need less and less exploration)
            epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay_rate*episode)

            #Printemaos al final de cada episodio para ver como va la Qtable
            print("\n\n |||||||||||||||||||||||||||||")
            print("Episodio: " + str(episode))
            print ("Epsilon: " + str(epsilon))
            print(Qtable)
            print("||||||||||||||||||||||||||||||| \n\n")

            # Reset the environment
            info, state = env.reset()
            step = 0
            terminated = False

            #Cada 200 episodios guardamos la Qtable para ver la evolución
            if episode % 200 == 0:
                guardar_qtable(Qtable, "V1-" + str(episode))

            # repeat
            for step in range(max_steps):
                # Choose the action At using epsilon greedy policy
                time.sleep(0.25) #Tiempo entre acciones que toma el coche (0.25 es el tiempo de reaccion de un humano promedio)
                action = epsilon_greedy_policy(Qtable, state, epsilon, env)
                #print( "Action: " + str(action))
                # Take action At and observe Rt+1 and St+1
                # Take the action (a) and observe the outcome state(s') and reward (r)
                info, new_state, reward, terminated = env.step(action)

                # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
                Qtable[state][action] = Qtable[state][action] + learning_rate * (reward + gamma * np.max(Qtable[new_state]) - Qtable[state][action])

                # If terminated or truncated finish the episode
                if terminated:
                    break

                # Our next state is the new state
                state = new_state


    except KeyboardInterrupt:
        destruirActores()

    except Exception as e:
        print("Error: " + str(e))
        destruirActores()



# |||||||||| Funciones para destruir los actores ||||||||||

def destruirNPC():
    if len(listaNPC) > 0:
        for npc in listaNPC:
            npc.destroy()
            if npc in listaActores:
                listaActores.remove(npc)
        listaNPC.clear()
        print("Se ha vaciado toda la lista de NPC")
    else:
        print("No hay ningun NPC para destruir")

def destruirCocheAutonomo():
    if len(listaCocheAutonomo) > 0:
        print(len(listaCocheAutonomo))
        #Se elimina la lista al reves para eliminar primero los sensores y despues el coche autonomo
        listaCocheAutonomo[0].apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(0.1)
        listaCocheAutonomo[1].stop()
        listaCocheAutonomo[2].stop()
        listaCocheAutonomo[3].stop()
        listaCocheAutonomo[4].stop()
        for elemento in listaCocheAutonomo:
            correcto = elemento.destroy()
            if correcto:
                print("Elemento destruido")
            if elemento in listaActores:
                listaActores.remove(elemento)

        listaCocheAutonomo.clear()
        time.sleep(3)#Tiempo de precaución antes de respawnear otro coche
        
        print("Se ha vaciado la lista de todos los sensores y el coche autonomo")
    else:
        print("No hay ningun coche autonomo para destruir")

def destruirActores():
    if len(listaActores) > 0:
        for actor in reversed(listaActores):
            actor.destroy()
            if actor in listaCocheAutonomo:
                listaCocheAutonomo.remove(actor)
        listaActores.clear()
        print("Se ha vaciado toda la lista de actores")
    else:
        print("No hay ningun actor para destruir")



# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

# |||||| Clase para crear el entorno de gym ||||||||||

class CarlaEnv(gym.Env):
    def __init__(self, client):
        super(CarlaEnv, self).__init__()

        self.cliente = client
        self.world = self.cliente.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.cache = []
        self.cocheAutonomo = spawnearVehiculoAutonomo(self.world, self.blueprint_library, self.cache, self)
        self.ultColx = 0
        self.ultColy = 0
        self.sensorColision = None
        
        
        self.action_space = spaces.Discrete(10)  # Puede ser aceleración, frenado, dirección, etc.
        self.observation_space = spaces.Discrete(6) # Todas las combinaciones de los  (linea y obstaculos), porque el de colisión es para acabar el episodio


    def reset(self):
        
        self.moverCochePosicionIncial()
        if self.cocheAutonomo is None:
            raise ValueError("Error al spawnear el vehículo autónomo")
        return self.get_observation() #Devuelve informacion al final de cada episodio

    def step(self, action):
        # Convertir la acción en un comando para el coche (ejemplo, movimiento)
        if action == 0:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0)) #Acelerar
        elif action == 1:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0)) #Acelerar 
        elif action == 2:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=0.75)) #Girar 
        elif action == 3:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=0.5)) #Girar 
        elif action == 4:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=0.25)) #Girar 
        elif action == 5:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=-1.0)) #Girar 
        elif action == 6:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=-0.75)) #Girar 
        elif action == 7:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=0.0, steer=-0.5)) #Girar 
        elif action == 8:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=-0.5, steer=0.0)) #Frenar 
        else:
            self.cocheAutonomo.apply_control(carla.VehicleControl(throttle=-1.0, steer=0.0)) #Frenar

        # Obtener la observación actual (por ejemplo, imagen de la cámara)
        info, state = self.get_observation()

        # Calcular la recompensa (esto depende de tu objetivo específico)
        reward = self.calcularRecompensa()  # Aquí va la lógica de recompensa, ejemplo simple
        #print("Recompensa: " + str(reward))
        # Verificar si el episodio ha terminado (por ejemplo, si el coche ha chocado)
        done = self.terminated()  # Lógica para determinar cuándo se acaba el episodio

        #print(self.cache) # Para ir viendo como va el entorno
        self.cache = [] #Vaciamos la cache para que no se acumulen los datos

        return info, state, reward, done

    def get_observation(self):
        
        if 1 in self.cache and 2 not in self.cache and 3 not in self.cache:
            return "obstaculo detectado, S5", 5
        elif 1 not in self.cache and 2 in self.cache and 3 not in self.cache:
            return "linea continua detectada, S4", 4
        elif 1 not in self.cache and 2 not in self.cache and 3 in self.cache:
            return "linea discontinua detectada, S3", 3
        elif 1 in self.cache and 2 in self.cache and 3 not in self.cache:
            return "obstaculo y linea continua detectada, S2", 2
        elif 1 in self.cache and 2 not in self.cache and 3 in self.cache:
            return "obstaculo y linea discontinua detectada, S1", 1
        else:
            return "Todo correcto S0", 0


    def render(self, mode='human'):
        # Renderizar el entorno para visualizarlo (si es necesario)
        pass

    def close(self):
        # Limpiar y cerrar los recursos al finalizar
        destruirActores()

    def terminated(self):
        if 0 in self.cache:
            return True
        else:
            return False

    def calcularRecompensa(self):
        acu = 0
        for elemento in self.cache:
            if elemento == 2:
                acu -= 1
            elif elemento == 3:
                acu -= 1
        return acu

    #Funciones para manejar los sensores del coche autonomo

    def manejarSensorLinea(self, invasion):
        if 2 not in self.cache and 3 not in self.cache:
            if "Solid" in str(invasion.crossed_lane_markings[0].type):
                self.cache.append(2) # Para lineas continuas y continuas con discontinuas
            else:
                self.cache.append(3) # Para lineas discontinuas
            print("Invasion de linea detectada de tipo: " + str(invasion.crossed_lane_markings[0].type))

    def manejadorColisiones(self, colision):
        if self.sensorColision is not None:
            self.sensorColision.stop()
            self.sensorColision.destroy()
            listaActores.remove(self.sensorColision)
            self.sensorColision = None
            self.cache.append(0)
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
        
    def moverCochePosicionIncial(self):
        if self.cocheAutonomo is not None:
            self.cocheAutonomo.set_transform(self.world.get_map().get_spawn_points()[0])
            time.sleep(1)
            #setear sensor de colision
            self.sensorColision = self.blueprint_library.find('sensor.other.collision')
            self.sensorColision = self.world.try_spawn_actor(self.sensorColision, carla.Transform(), attach_to=self.cocheAutonomo)
            listaActores.append(self.sensorColision)
            self.sensorColision.listen(lambda colision: self.manejadorColisiones(colision))
            print("Coche autonomo movido a la posicion inicial")
        
            

# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||




#Para que se ejecute el main cuando se inicia el programa

if __name__ == '__main__':

    try:
        main()  
    finally:
        destruirActores()
        print("Done")





    

