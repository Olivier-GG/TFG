import numpy as np
from stable_baselines3 import DQN
import os
import json
from tqdm import tqdm

#Función para entrenar el agente
def train_agent(env):

    with open('../configuracion.json', 'r') as file:
        configuracion = json.load(file)

    #Creamos los directorios para almacenar la informacion
    model_dir = configuracion['Fase2']['Directorio_Nuevo_Modelo']
    log_dir = configuracion['Fase2']['Directorio_Nuevos_Logs']
    os.makedirs(model_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    
    #Seleccionamos el tipo de modelo que queremos(segun el tipo de sensor que usemos)
    if configuracion['Fase2']['Sensor_Activo'] == 'Semantico':
        model = DQN('CnnPolicy', env, verbose=1, device='cuda', tensorboard_log=log_dir, buffer_size=50000, exploration_fraction=0.80, gamma=0.7, learning_starts=1000)  #Para usar el Sensor Semantico
    elif configuracion['Fase2']['Sensor_Activo'] == 'Lidar':
        model = DQN('MlpPolicy', env, verbose=1, device='cuda', tensorboard_log=log_dir, buffer_size=50000, exploration_fraction=0.80, gamma=0.7, learning_starts=1000) # Para el Lidar

    # Puedes ver los resulatdos que va dando el entrenamiento con el comando -> tensorboard --logdir logs

    TIMESTEPS = 25000 # Total de timesteps por iteración
    iteraciones = 0

    while iteraciones < 40:
        iteraciones += 1

        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, log_interval=1, progress_bar=True)
        model.save(f"{model_dir}/dqn_{configuracion['Fase2']['Sensor_Activo']}_{TIMESTEPS * (iteraciones)}")
        
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



#Función para evaluar el agente entrenado
def evaluate_agent(env, directory):

    with open('../configuracion.json', 'r') as file:
        configuracion = json.load(file)

    model = DQN.load(directory, env=env)

    rewards = []
    timestep = 0
    terminated = False

    total_timesteps = 2000  # Total de timesteps para la evaluación
    
    progress_bar = tqdm(total=total_timesteps, desc="Evaluando agente " + configuracion['Fase2']['Sensor_Activo'], unit="timestep")

    obs, _ = env.reset()

    while timestep < total_timesteps:
        
        action, _ = model.predict(observation=obs, deterministic=True)
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

    