import glob
import os
import sys
import random
from spawn_npc import spawnearCoches
import cv2
import numpy as np
import carla
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



listaActores = []
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
enviroment = client.get_world()
#enviroment = client.load_world('Town07')


blueprint_library = enviroment.get_blueprint_library()


def procesarImagen(imagen): #Para que se vea como circula el coche
    imagen = np.array(imagen.raw_data)
    img = imagen.reshape((600,800,4))
    img = img[:,:,:3]

    cv2.imshow("img", img)
    cv2.waitKey(100)




def spawnearVehiculoAutonomo (enviroment):
    transform = enviroment.get_map().get_spawn_points()[0]
    #transform = random.choice(enviroment.get_map().get_spawn_points()[0] para que el sitio de respawn sea random
    
    vehiculoAutonomo = enviroment.try_spawn_actor(blueprint_library.filter('vehicle.*.*')[0], transform)
    listaActores.append(vehiculoAutonomo)

    camarab = blueprint_library.find('sensor.camera.rgb')
    camarab.set_attribute('image_size_x', '800')
    camarab.set_attribute('image_size_y', '600')
    camarab.set_attribute('fov', '90')
    camara = enviroment.try_spawn_actor(blueprint_library.find('sensor.camera.rgb'),  carla.Transform(carla.Location(x=-7, z=3)), attach_to=vehiculoAutonomo)
    listaActores.append(camara)

    #vehiculoAutonomo.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
    vehiculoAutonomo.set_autopilot(True)

    print("Vehiculo con sensores spawneado")


    camara.listen(lambda image: procesarImagen(image))






def main () :

 
   
    #print("Procedo a spawnear 5 coches y 10' peatones")
    #spawnearCoches(5,10)

    spawnearVehiculoAutonomo(enviroment)



    while True :
        i = 0







#Esto solo se ejecuta si esta funcion ha sido ejecutada directamente y no desde un import de otra funcion, siempre que acaba el programa vacia la lista de actores.

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in listaActores])
        print("Se ha vaciado toda la lista de actores")



    

