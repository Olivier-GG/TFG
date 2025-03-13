import glob
import os
import sys
import random
from spawn_npc import spawnearCoches
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
from FuncionesEntrenamientoFase1 import train_agent, evaluate_agent

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
    #||||||||||||||||||||||

    #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
    camara.listen(lambda image: env.manejarSensorCamara(world, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
    #sensorColision.listen(lambda colision: env.manejadorColisiones(colision)) #Para que se imprima por pantalla cuando se detecte una colision
    sensorInvasion.listen(lambda invasion: env.manejarSensorLinea(invasion)) #Para que se imprima por pantalla cuando se detecte una invasion de linea
    sensorObstaculos.listen(lambda obstaculo: env.manejarSensorObstaculos(obstaculo)) #Para que se imprima por pantalla cuando se detecte un obstaculo
   


    return vehiculoAutonomo



# |||||||| Funciones auxiliares ||||||||||

# Cargar desde JSON
def cargar_qtable(filename):
    with open("TablasFase1/" + filename + ".json", "r") as f:
        q_table = json.load(f)  # Cargar la lista desde JSON
        print("Qtable cargada: " + filename)
    return np.array(q_table) 


def initialize_q_table(state_space, action_space):
  Qtable = np.zeros((state_space, action_space))
  return Qtable



#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||| MAIN |||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||
def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor e inicializar enviroment ||||||||
        
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

        # Parametros de entrenamiento
        n_training_episodes = 4005  # Total training episodes
        learning_rate = 0.1         # Learning rate

        # Parametros de evaluación
        n_eval_episodes = 100        # Total number of test episodes

        # Parametros de episodio
        max_steps = 200              # Max steps per episode
        gamma = 0.10                 # Discounting rate 

        # Parametros de exploración
        max_epsilon = 1.0             # Exploration probability at start
        min_epsilon = 0.05            # Minimum exploration probability
        decay_rate = 0.0020           # Exponential decay rate for exploration prob

        #|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||



        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        print("\nProcedo a spawnear 30 coches y 10 peatones")
        
        listaNPC.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla 


        # ||||||||||||| Paso 3, elegir si queremos evaluar o entrenar agente ||||||||||||||

        print("\nIntroduce una 't' si quieres entrenar el agente o una 'e' si quieres evaluarlo(Seleccionar en el codigo que Qtable desea cargar -línea 221-): ")
        eleccion = input()

        if eleccion == "e" or eleccion == "E":

            print("Evaluando agente...")
            Qtable = cargar_qtable("V2-4000")
            print(Qtable)
            
            mean_reward, std_reward = evaluate_agent(env, max_steps, n_eval_episodes, Qtable)

            print("Resultados de la evaluación:")
            print("Mean reward: ", mean_reward)
            print("Std reward: ", std_reward)

            
        elif eleccion == "t" or eleccion == "T":

            
            print("               Comenzando el entrenamiento        \n")

            train_agent(env, Qtable, n_training_episodes, max_steps, gamma, learning_rate, max_epsilon, min_epsilon, decay_rate, Qtable, max_epsilon)

            print("\n\n\n")
            print("               Entrenamiento completado         \n")

        else:
            print("Opción no válida")

        #Cerramos el enviroment
        env.close()
        print("Enviroment cerrado")


    except KeyboardInterrupt:
        destruirActores()

    except Exception as e:
        print("Error: " + str(e))
        destruirActores()




# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||| Funciones para destruir los actores ||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
# |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

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





    

