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

    #No encuentra la grafica porque las ultimas versiones de CUDA no funcionan
    print("CUDA disponible:", torch.cuda.is_available())
   
    model = DQN('CnnPolicy', env, verbose=1, tensorboard_log=log_dir, target_update_interval=5000, buffer_size=30000, device='cuda') # si no pones cuda automaticamente se pone en gpu
   
    TIMESTEPS = 500000 # 200 * numero de episodios para compensar 
    iters = 0

    model.learn(total_timesteps=TIMESTEPS) # train
    model.save(f"{model_dir}/dqn_{TIMESTEPS*iters}") # Save a trained model every TIMESTEPS
        
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



# Evaluate the agent
def evaluate_agent(env):

    # Load model
    model = DQN.load('models/dqn_2000', env=env)

    # Run a test
    obs = env.reset()[0]
    terminated = False
    while True:
        action, _ = model.predict(observation=obs, deterministic=True) # Turn on deterministic, so predict always returns the same behavior
        obs, _, terminated, _, _ = env.step(action)

        if terminated:
            break
        
