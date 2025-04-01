import numpy as np
import random
import json
from stable_baselines3 import DQN
import os
import torch


def train_agent(env):

    #Creamos los directorios para almacenar la informacion
    model_dir = "models"
    log_dir = "logs"
    os.makedirs(model_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    
    #Seleccionamos el tipo de modelo que queremos(segun el tipo de sensor que usemos)
    model = DQN('CnnPolicy', env, verbose=1, device='cuda', tensorboard_log=log_dir, buffer_size=30000) #Utilizado para la camara semantica
    #model = DQN('MlpPolicy', env, verbose=1, device='cuda', tensorboard_log=log_dir, buffer_size=30000) #Utilizado para el LIDAR

    # Puedes ver los resulatdos que va dando el entrenamiento con el comando -> tensorboard --logdir logs

    TIMESTEPS = 300000 #Equivaldria a unos 3000 episodios de los anteirores

    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, log_interval=1, progress_bar=True) # train
    model.save(f"{model_dir}/dqnV1_300000") # Save a trained model every TIMESTEPS


    print("Entrenamiento finalizado, puede proceder a evaluarlo")



# Evaluate the agent
def evaluate_agent(env, directory):

    # Load model
    model = DQN.load(directory, env=env)

    # Run a test
    obs = env.reset()[0]
    terminated = False
    while True:
        action, _ = model.predict(observation=obs, deterministic=True) # Turn on deterministic, so predict always returns the same behavior
        obs, _, terminated, _, _ = env.step(action)

        if terminated:
            break
        
