import os
import sys
import os
import sys
import gymnasium as gym
from FuncionesEntrenamientoFase2 import train_agent, evaluate_agent
from stable_baselines3.common.env_checker import check_env

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ScriptsAuxiliares/')))
from spawn_npc import spawnearCoches

import carla


listaNPC = []

#||||||||||||||||||||||||||||||||||||||||||||||||
#||||||||||||||| MAIN |||||||||||||||||||||||||||
#||||||||||||||||||||||||||||||||||||||||||||||||

def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor e inicializar enviroment ||||||||
        
        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(20.0) #Establecemos el tiempo para establecer la conexion a 20 segundos, porque al cargar el mapa puede tardar unos pocdos segundos
        cliente.load_world('Town03') #Cargamos la cuidad que deseemos

        #Incializamos el entorno de gym
        env = gym.make('CarlaEnviroment')

        #Comprobamos que el enviroment es correcto
        print ("Comprobando el enviroment...")
        check_env(env)
        print ("✅El enviroment es correcto")
        

        print("Conexion con el servidor establecida y todas las variables principales inicializadas")

        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        print("\nProcedo a spawnear 30 coches y 10 peatones")
        
        listaNPC.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla (posiblemente spawneen menos peatones, porque pueden ser que spawneen en un sitio que ya hay uno)


        # ||||||||||||| Paso 3, elegir si queremos evaluar o entrenar agente ||||||||||||||

        print("\nIntroduce una 't' si quieres entrenar el agente o una 'e' si quieres evaluarlo (seleccionar modelo en línea 78 de cliente.py): ")
        eleccion = input()

        if eleccion == "e" or eleccion == "E":

            print("Evaluando agente...")

            evaluate_agent(env, 'modelos/models(Lidar)/dqn_semantico_550000_V1') 

            print("✅Evaluar agente completado")
            
        elif eleccion == "t" or eleccion == "T":

            print("               Comenzando el entrenamiento        \n")

            train_agent(env)

            print("\n\n\n               ✅Entrenamiento completado         \n")

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

    

#Funcion para destruir los actores
def destruirNPC():
    if len(listaNPC) > 0:
        for npc in listaNPC:
            if npc.is_alive:
                npc.destroy()
        listaNPC.clear()
        print("Se ha vaciado toda la lista de NPC")
    else:
        print("No hay ningun NPC para destruir")


#Para que se ejecute el main cuando se inicia el programa
if __name__ == '__main__':

    try:
        main()  
    finally:
        destruirNPC()
        print("Done")





    

