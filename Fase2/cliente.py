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
from CarlaEnvFase2 import CarlaEnv
from FuncionesEntrenamientoFase2 import train_agent, evaluate_agent
from stable_baselines3.common.env_checker import check_env

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla


listaNPC = []

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
        cliente.set_timeout(20.0)
        cliente.load_world('Town03') #Cargamos la cuidad que deseemos

        #Incializamos el entorno de gym
        env = gym.make('CarlaEnviroment')
        
        #Comprobamos que el enviroment es correcto
        check_env(env)

        print("Conexion con el servidor establecida y todas las variables principales inicializadas")


        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        print("\nProcedo a spawnear 30 coches y 10 peatones")
        
        listaNPC.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla 


        # ||||||||||||| Paso 3, elegir si queremos evaluar o entrenar agente ||||||||||||||

        print("\nIntroduce una 't' si quieres entrenar el agente o una 'e' si quieres evaluarlo(Seleccionar en el codigo que Qtable desea cargar -línea 221-): ")
        eleccion = input()

        if eleccion == "e" or eleccion == "E":

            print("Evaluando agente...")
            
            evaluate_agent(env)

            print("Evaluar agente completado")

            
        elif eleccion == "t" or eleccion == "T":

            
            print("               Comenzando el entrenamiento        \n")

            train_agent(env)

            print("\n\n\n")
            print("               Entrenamiento completado         \n")

        else:
            print("Opción no válida")

        #Cerramos el enviroment
        env.close()
        print("Enviroment cerrado")


    except KeyboardInterrupt:
        destruirNPC()
        env.close()

    except Exception as e:
        print("Error: " + str(e))
        destruirNPC()
        env.close() 

    


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


""""

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


"""

#Para que se ejecute el main cuando se inicia el programa

if __name__ == '__main__':

    try:
        main()  
    finally:
        destruirNPC()
        print("Done")





    

