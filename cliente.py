import glob
import os
import sys
import random
from spawn_npc import spawnearCoches
from spawnearNPC import Spawn
import cv2
import numpy as np
import carla
import glob
import os
import sys
import time
from time import sleep

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

def spawnearVehiculoAutonomo (enviroment, blueprint_library):

    #Donde vamos a respawnear el coche
    transform = enviroment.get_map().get_spawn_points()[0]
    #transform = random.choice(enviroment.get_map().get_spawn_points()[0] para que el sitio de respawn sea random
    
    #Spawneamos el vehiculo
    vehiculoAutonomo = enviroment.try_spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)
    listaActores.append(vehiculoAutonomo)
    listaCocheAutonomo.append(vehiculoAutonomo)

    #|||||||| Sensores ||||||||||

    #Spawneamos camara para ver vehiculo
    camarab = blueprint_library.find('sensor.camera.rgb')
    camarab.set_attribute('image_size_x', '800')
    camarab.set_attribute('image_size_y', '600')
    camarab.set_attribute('fov', '90')
    camara = enviroment.try_spawn_actor(blueprint_library.find('sensor.camera.rgb'),  carla.Transform(carla.Location(x=-7, z=3)), attach_to=vehiculoAutonomo)
    listaActores.append(camara)
    listaCocheAutonomo.append(camara)

    #Spawneamos sensor de colision
    sensorColisionb = blueprint_library.find('sensor.other.collision')
    sensorColision = enviroment.try_spawn_actor(sensorColisionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorColision)
    listaCocheAutonomo.append(sensorColision)

    #Spawneamos sensor de invasion de linea
    sensorInvasionb = blueprint_library.find('sensor.other.lane_invasion')
    sensorInvasion = enviroment.try_spawn_actor(sensorInvasionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorInvasion)
    listaCocheAutonomo.append(sensorInvasion)

    #Spawneamos sensor de obstaculos
    sensorObstaculosb = blueprint_library.find('sensor.other.obstacle')
    sensorObstaculosb.set_attribute('distance', '15')
    sensorObstaculosb.set_attribute('hit_radius', '4')
    sensorObstaculosb.set_attribute('only_dynamics', 'True')
    sensorObstaculosb.set_attribute('sensor_tick', '1.0')
    sensorObstaculos = enviroment.try_spawn_actor(sensorObstaculosb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorObstaculos)
    listaCocheAutonomo.append(sensorObstaculos)

    #Activamos los sensores

    #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
    camara.listen(lambda image: moverEspectador(enviroment, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
    sensorColision.listen(lambda colision: destruirCoche(colision)) #Para que se imprima por pantalla cuando se detecte una colision
    sensorInvasion.listen(lambda invasion: print("Invasion de linea detectada, linea de tipo: " + invasion.type)) #Para que se imprima por pantalla cuando se detecte una invasion de linea
    sensorObstaculos.listen(lambda obstaculo: print("Obstaculo " + obstaculo.other_actor.type_id + " detectado a " + str(obstaculo.distance) + " metros")) #Para que se imprima por pantalla cuando se detecte un obstaculo


    print("Vehiculo con sensores spawneado")


    #||||||||| Control del vehiculo |||||||||||

    #vehiculoAutonomo.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
    #vehiculoAutonomo.set_autopilot(True)
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

#Apano para poder ver el coche moverse gracias al espectador
def moverEspectador(world, vehicle):
    if vehicle.is_alive:
        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
        carla.Rotation(pitch=-90)))
    

def destruirCoche(colision):
    print("Colision detectada con " + colision.other_actor + ", eliminando vehiculo")
    destruirCocheAutonomo()





# |||||||| Funciones auxiliares ||||||||||

#Mueve el coche con inputs totalmente aleatorios
def MoverCocheAutonomoAleatorio(vehiculo):
    
    giroAleatorio = random.uniform(-1, 1)
    #print("girio aleatorio: " + str(giroAleatorio))
    acelerarAleatorio = random.uniform(-1, 1)
    #print("aceleracion aleatoria" + str(acelerarAleatorio))

    vehiculo.apply_control(carla.VehicleControl(throttle=acelerarAleatorio, steer=giroAleatorio))




# |||||||||||||| MAIN |||||||||||||||||
def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor

        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(5.0)
        enviroment = cliente.get_world()
        #enviroment = cliente.load_world('Town01')
        blueprint_library = enviroment.get_blueprint_library()


        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    

        #print("\nProcedo a spawnear 30 coches y 10 peatones")
        #listaActores.extend(spawnearCoches(30,10)) #Codigo de ejemplo carla 
        #listaNPC.extend(spawnearCoches(30,10)) 
        #listaActores.extend(Spawn(enviroment,blueprint_library,10,10)) #Codigo hecho por mi

        while True:
            #|||||| Paso 2, Spawnear vehiculo, y anadirle todos los sensores necesarios |||||

            cocheAutonomo = spawnearVehiculoAutonomo(enviroment, blueprint_library)

            #|||||| Paso 4, Ejecutar entrenamiento |||||


            #Mientras no haya codigo de Ia se deja el bucle para que el coche se siga moviendo hasta que se pulse ctrC
            while cocheAutonomo.is_alive:
                MoverCocheAutonomoAleatorio(cocheAutonomo)
                time.sleep(2)


    except KeyboardInterrupt:
        destruirActores()

    except Exception as e:
        print("Error: " + str(e))
        destruirActores()





# |||||||||| Funciones para destruir los actores ||||||||||

def destruirNPC():
    for npc in listaNPC:
        npc.destroy()
        listaNPC.remove(npc)
        listaActores.remove(npc)
    print("Se ha vaciado toda la lista de NPC")

def destruirCocheAutonomo():
    for elemento in listaCocheAutonomo:
        elemento.destroy()
        listaCocheAutonomo.remove(elemento)
        listaActores.remove(elemento)
    print("Se ha vaciado la lista de todos los sensores y el coche autonomo")

def destruirActores():
    for actor in listaActores:
        actor.destroy()
        listaActores.remove(actor)
    print("Se ha vaciado toda la lista de actores")



#Para que se ejecute el main cuando se inicia el programa

if __name__ == '__main__':

    try:
        main()  
    finally:
       print("Done")





    

