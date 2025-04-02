import numpy as np
import random
import json
from stable_baselines3 import DQN
import os
import torch
from tqdm import tqdm
from stable_baselines3.common.logger import configure

def train_agent(env):

    #Creamos los directorios para almacenar la informacion
    model_dir = "models"
    log_dir = "logsTensorboard"
    os.makedirs(model_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    
    #Seleccionamos el tipo de modelo que queremos(segun el tipo de sensor que usemos)
    model = DQN('CnnPolicy', env, verbose=1, device='cuda', tensorboard_log=log_dir, buffer_size=30000) 

    # Puedes ver los resulatdos que va dando el entrenamiento con el comando -> tensorboard --logdir logs

    TIMESTEPS = 50000 #Equivaldria a unos 3000 episodios de los anteirores
    iteraciones = 0

    while iteraciones < 20:
        iteraciones += 1

        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, log_interval=1, progress_bar=True)
        model.save(f"{model_dir}/dqn_Semantic_{TIMESTEPS * (iteraciones)}_noDelay") 
        
    
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



# Evaluate the agent
def evaluate_agent(env, directory):

    model = DQN.load(directory, env=env)

    rewards = []
    timestep = 0
    terminated = False

    total_timesteps = 2000  # Total de timesteps para la evaluación
    
    progress_bar = tqdm(total=total_timesteps, desc="Evaluando agente", unit="timestep")


    obs, _ = env.reset()
    

    while timestep < total_timesteps:
        
        action, _ = model.predict(observation=obs, deterministic=False)
        obs, reward, terminated, _ , _ = env.step(action)
        
        rewards.append(reward)
        timestep += 1
        progress_bar.update(1)

        
        if terminated:
            obs, _ = env.reset()
            terminated = False
           

    progress_bar.close()

    mean_reward = np.mean(rewards)
    var_reward = np.var(rewards)
    print(f"\n\n🔹 Timesteps: {timestep} | Recompensa Promedia: {mean_reward:.4f} | Varianza: {var_reward:.4f}")
    
    print("\n✅ Evaluación completa.")

    