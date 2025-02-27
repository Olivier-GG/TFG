import glob
import os
import sys
import random
from spawn_npc import spawnearCoches
from spawnearNPC import Spawn
import numpy as np
import glob
import os
import sys
import time
import tqdm
from typing_extensions import Literal
import pdb
import json
import cv2
import gymnasium as gym
from gymnasium import spaces
from CarlaEnv import CarlaEnv

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla

# |||||| Listas con todos los actores ||||||||

listaActores = [] #Se añaden todos los actores
listaCocheAutonomo = [] #Se añade todos los sensores del coche autonomo y el coche autonomo para poder ser borrados
listaNPC = [] #Se añaden todos los sensores de los NPC y los NPC para poder ser borrados



# ||||||| Funcion para spawnear el coche autonomo y añadirle los sensores necesarios ||||||||||

def spawnearVehiculoAutonomo (world, blueprint_library, env): #Se le pasa el mundo y la libreria de blueprints para poder spawnear los actores 

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

    listaActores.append(vehiculoAutonomo)
    listaCocheAutonomo.append(vehiculoAutonomo)

    #||||||||||||||||||||||||||||
    #|||||||| Sensores ||||||||||
    #||||||||||||||||||||||||||||

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
    sensorObstaculosb.set_attribute('distance', '20')
    sensorObstaculosb.set_attribute('hit_radius', '0')
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



    #||||||||||||||||||||||
    #Activamos los sensores

    #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
    camara.listen(lambda image: env.manejarSensorCamara(world, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
    #sensorColision.listen(lambda colision: env.manejadorColisiones(colision)) #Para que se imprima por pantalla cuando se detecte una colision
    sensorInvasion.listen(lambda invasion: env.manejarSensorLinea(invasion)) #Para que se imprima por pantalla cuando se detecte una invasion de linea
    sensorObstaculos.listen(lambda obstaculo: env.manejarSensorObstaculos(obstaculo)) #Para que se imprima por pantalla cuando se detecte un obstaculo
   


    return vehiculoAutonomo





# |||||||| Funciones auxiliares ||||||||||

# Guardar en JSON
def guardar_qtable(q_table, filename):
    with open("TablasFase1/" + filename + ".json", "w") as f:
        json.dump(q_table.tolist(), f)

# Cargar desde JSON
def cargar_qtable(filename):
    with open("TablasFase1/" + filename + ".json", "r") as f:
        q_table = json.load(f)  # Cargar la lista desde JSON
    return np.array(q_table) 


def initialize_q_table(state_space, action_space):
  Qtable = np.zeros((state_space, action_space))
  return Qtable



#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#Funciones para la political de exploracion y explotacion

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




#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||| MAIN |||||||||||||||||
def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor
        
        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(15.0)
        cliente.load_world('Town03') #Cargamos la cuidad que deseemos
        world = cliente.get_world()
        blueprint_library = world.get_blueprint_library()

        #Incializamos el entorno de gym
        env = CarlaEnv(cliente)
        vehiculo = spawnearVehiculoAutonomo(world, blueprint_library, env) # Se le pasa el enviromental para poder manejar los sensores
        env.setCocheAutonomo(vehiculo)

        state_space = env.observation_space.n
        action_space = env.action_space.n

        #Inicializamos Qtable
        Qtable = initialize_q_table(state_space, action_space)

        print("Conexion con el servidor establecida y todas las variables principales inicializadas")



        #|||||||||||||||||| Parametros para el entrenamiento |||||||||||||||||

        # Training parameters
        n_training_episodes = 3005 # Total training episodes
        learning_rate = 0.1         # Learning rate

        # Evaluation parameters
        n_eval_episodes = 100        # Total number of test episodes

        # Environment parameters
        max_steps = 200              # Max steps per episode
        gamma = 0.10                 # Discounting rate 
        eval_seed = []               # The evaluation seed of the environment

        # Exploration parameters
        max_epsilon = 1.0             # Exploration probability at start
        min_epsilon = 0.05            # Minimum exploration probability
        decay_rate = 0.0015           # Exponential decay rate for exploration prob

        #|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||



        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        print("\nProcedo a spawnear 30 coches y 10 peatones")
        
        listaNPC.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla 



        #Paso 3, elegir si queremos evaluar o entrenar agente

        print("Introduce una 't' si quieres entrenar el agente o una 'e' si quieres evaluarlo: ")
        eleccion = input()

        if eleccion == "e":

            print("Evaluando agente...")
            Qtable = cargar_qtable("V2-3000")
            print(Qtable)
            
            mean_reward, std_reward = evaluate_agent(env, max_steps, n_eval_episodes, Qtable)

            print("Resultados de la evaluación:")
            print("Mean reward: ", mean_reward)
            print("Std reward: ", std_reward)

            

        elif eleccion == "t":

            print("\n\n\n")
            print("               Comenzando el entrenamiento        \n")

            train_agent(env, Qtable, n_training_episodes, max_steps, gamma, learning_rate, max_epsilon, min_epsilon, decay_rate, Qtable, max_epsilon)


        env.close()
        print("Enviroment cerrado")


    except KeyboardInterrupt:
        destruirActores()

    except Exception as e:
        print("Error: " + str(e))
        destruirActores()



#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#||||| Funciones para el agente ||||| |||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

def train_agent(env, Q, n_training_episodes, max_steps, gamma, learning_rate, epsilon, min_epsilon, decay_rate, Qtable, max_epsilon):

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
            guardar_qtable(Qtable, "V2-" + str(episode))

        # repeat
        for step in range(max_steps):
            # Elegimos la accion segun nuestra politica
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
        
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



# Evaluate the agent
def evaluate_agent(env, max_steps, n_eval_episodes, Q):

    episode_rewards = []

    for episode in range(n_eval_episodes):

        info, state = env.reset()
        step = 0
        terminated = False
        total_rewards_ep = 0

        for step in range(max_steps):
            
            action = greedy_policy(Q, state)
            info, new_state, reward, terminated = env.step(action)
            total_rewards_ep += reward
            
            if terminated:
                break
            
            state = new_state

            episode_rewards.append(total_rewards_ep)

        print("Reward: " + str(total_rewards_ep))

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)

    return mean_reward, std_reward







# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||| Funciones para destruir los actores ||||||||||

def destruirNPC():
    if len(listaNPC) > 0:
        for npc in listaNPC:
            if npc.is_alive:
                npc.destroy()
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
        for elemento in listaCocheAutonomo:
            elemento.destroy()
            if elemento in listaActores:
                listaActores.remove(elemento)

        listaCocheAutonomo.clear()
        time.sleep(2)#Tiempo de precaución
        
        print("Se ha vaciado la lista de todos los sensores y el coche autonomo")
    else:
        print("No hay ningun coche autonomo para destruir")

def destruirActores():
    destruirNPC()
    if len(listaActores) > 0:
        for actor in reversed(listaActores):
            if actor.is_alive:
                actor.destroy()
            if actor in listaCocheAutonomo:
                listaCocheAutonomo.remove(actor)
        listaActores.clear()
        print("Se ha vaciado toda la lista de actores")
    else:
        print("No hay ningun actor para destruir")




#Para que se ejecute el main cuando se inicia el programa

if __name__ == '__main__':

    try:
        main()  
    finally:
        destruirActores()
        print("Done")





    

