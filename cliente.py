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

# Encontrar modulo de carla
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



listaActores = [] #Se añaden todos los actores para poder ser borrados al final de la ejecucion

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
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
    carla.Rotation(pitch=-90)))


def spawnearVehiculoAutonomo (enviroment, blueprint_library):
     #Donde vamos a respawnear el coche
    transform = enviroment.get_map().get_spawn_points()[0]
    #transform = random.choice(enviroment.get_map().get_spawn_points()[0] para que el sitio de respawn sea random
    
    #Spawneamos el vehiculo
    vehiculoAutonomo = enviroment.try_spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)
    listaActores.append(vehiculoAutonomo)

    #|||||||| Sensores ||||||||||

    #Spawneamos camara para ver vehiculo
    camarab = blueprint_library.find('sensor.camera.rgb')
    camarab.set_attribute('image_size_x', '800')
    camarab.set_attribute('image_size_y', '600')
    camarab.set_attribute('fov', '90')
    camara = enviroment.try_spawn_actor(blueprint_library.find('sensor.camera.rgb'),  carla.Transform(carla.Location(x=-7, z=3)), attach_to=vehiculoAutonomo)
    listaActores.append(camara)

    #Spawneamos sensor de colision
    sensorColisionb = blueprint_library.find('sensor.other.collision')
    sensorColision = enviroment.try_spawn_actor(sensorColisionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorColision)

    #Spawneamos sensor de invasion de linea
    sensorInvasionb = blueprint_library.find('sensor.other.lane_invasion')
    sensorInvasion = enviroment.try_spawn_actor(sensorInvasionb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorInvasion)

    #Spawneamos sensor de obstaculos
    sensorObstaculosb = blueprint_library.find('sensor.other.obstacle')
    sensorObstaculosb.set_attribute('distance', '20')

    sensorObstaculos = enviroment.try_spawn_actor(sensorObstaculosb, carla.Transform(), attach_to=vehiculoAutonomo)
    listaActores.append(sensorObstaculos)

    #Activamos los sensores

    #camara.listen(lambda image: procesarImagen(image)) # Para activar la vista en primera persona
    camara.listen(lambda image: moverEspectador(enviroment, vehiculoAutonomo)) #En vez de procesar lo recibido por el sensor, se mueve al espectador para que siga al coche
    #sensorColision.listen(lambda colision: print("Colision detectada")) #Para que se imprima por pantalla cuando se detecte una colision
    sensorInvasion.listen(lambda invasion: print("Invasion de linea detectada")) #Para que se imprima por pantalla cuando se detecte una invasion de linea
    sensorObstaculos.listen(lambda obstaculo: print("Obstaculo" + obstaculo.other_actor.type_id + "detectado a " + str(obstaculo.distance) + " metros")) #Para que se imprima por pantalla cuando se detecte un obstaculo


    print("Vehiculo con sensores spawneado")




    #||||||||| Control del vehiculo |||||||||||

    #vehiculoAutonomo.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
    #vehiculoAutonomo.set_autopilot(True)
    return vehiculoAutonomo


#Mueve el coche con inputs totalmente aleatorios
def MoverCocheAutonomoAleatorio(vehiculo):
    
    giroAleatorio = random.uniform(-1, 1)
    #print("girio aleatorio: " + str(giroAleatorio))
    acelerarAleatorio = random.uniform(-1, 1)
    #print("aceleracion aleatoria" + str(acelerarAleatorio))

    vehiculo.apply_control(carla.VehicleControl(throttle=acelerarAleatorio, steer=giroAleatorio))




def main () :

    try:

        #|||| Paso 1, conectar el cliente con el servidor

        cliente = carla.Client('localhost', 2000)
        cliente.set_timeout(5.0)
        enviroment = cliente.get_world()
        #enviroment = cliente.load_world('Town01')
        blueprint_library = enviroment.get_blueprint_library()


        #||||| Paso 2, Spawneo de trafico para poder realizar la simulacion ||||||||
    
        print("\nProcedo a spawnear 10 coches")
        #spawnearCoches(5,10)
        
        #listaActores.extend(Spawn(enviroment,blueprint_library,10,10))

        print("Han spawneado muchos 5 coche")
        #|||||| Paso 2, Spawnear vehicul, y anadirle todos los sensores necesarios |||||

        cocheAutonomo = spawnearVehiculoAutonomo(enviroment, blueprint_library)

        #|||||| Paso 4, Ejecutar entrenamiento |||||



        #Mientras no haya codigo de Ia se deja el bucle para que el coche se siga moviendo hasta que se pulse ctrC
    

        while True:
            MoverCocheAutonomoAleatorio(cocheAutonomo)
            time.sleep(2)

    except KeyboardInterrupt:
        cliente.apply_batch([carla.command.DestroyActor(x) for x in listaActores])
        print("Se ha vaciado toda la lista de actores")



#Para que se ejecute el main cuando se inicia el programa

if __name__ == '__main__':

    try:
        main()  
    finally:
       print("Done")





    

