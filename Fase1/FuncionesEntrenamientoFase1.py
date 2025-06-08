import numpy as np
import random
import json
import os

#Funcion par el entrenamiento del agente
def train_agent(env, Q, n_training_episodes, max_steps, gamma, learning_rate, epsilon, min_epsilon, decay_rate, Qtable, max_epsilon):

    with open('../configuracion.json', 'r') as file:
            configuracion = json.load(file)

    os.makedirs("Tablas/" + configuracion['Fase1']['Version_Nuevo_Entrenamiento'], exist_ok=True)

    for episode in range(n_training_episodes):
        # Reduce epsilon, para aumentar la explotación y disminuir la exploración
        epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay_rate*episode)

        #Mostramos por consola la Qtable cada 100 episodios
        print("\n\n |||||||||||||||||||||||||||||")
        print("Episodio: " + str(episode))
        print ("Epsilon: " + str(epsilon))
        print(Qtable)
        print("||||||||||||||||||||||||||||||| \n\n")

        # Reseteamos el enviroment
        info, state = env.reset()
        step = 0
        terminated = False

        #Cada 200 episodios guardamos la Qtable para ver la evolución
        if episode % 200 == 0:
            nombreTabla = configuracion['Fase1']['Version_Nuevo_Entrenamiento'] + "/" + configuracion['Fase1']['Version_Nuevo_Entrenamiento'] + "-" + str(episode)
            guardar_qtable(Qtable, nombreTabla)

        
        for step in range(max_steps):

            # Elegimos la accion segun nuestra politica
            action = epsilon_greedy_policy(Qtable, state, epsilon, env)
            
            info, new_state, reward, terminated = env.step(action)

            #Actualizamos la Qtable            
            Qtable[state][action] = Qtable[state][action] + learning_rate * (reward + gamma * np.max(Qtable[new_state]) - Qtable[state][action])

            #Si terminated es True, significa que hemos llegado al final del episodio
            if terminated:
                break

            state = new_state
        
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



#Funcion para evaluar el agente
def evaluate_agent(env, max_steps, n_eval_episodes, Q):

    episode_rewards = []

    for episode in range(n_eval_episodes):

        info, state = env.reset()
        step = 0
        terminated = False
        total_rewards_ep = 0

        for step in range(max_steps):
            
            action = greedy_policy(Q, state)
            info, new_state, reward, terminated = env.step(action)
            total_rewards_ep += reward
            
            if terminated:
                break
            
            state = new_state

        episode_rewards.append(total_rewards_ep)

        print("Reward: " + str(total_rewards_ep))

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)

    return mean_reward, std_reward


#||||||||||||||||||||||||||||||||||||||||
#|||||||| Funciones auxiliares ||||||||||
#||||||||||||||||||||||||||||||||||||||||

# Toma la acción con el valor más alto dado un estado
def greedy_policy(Qtable, state):
  
  action = np.argmax(Qtable[state][:])

  return action


#Genera un número aleatorio entre 0 y 1, si es mayor que epsilon, elige la acción greedy, si no, elige una acción aleatoria
def epsilon_greedy_policy(Qtable, state, epsilon, env):

  random_num = random.uniform(0,1)
 
  if random_num > epsilon:
    action = greedy_policy(Qtable, state)
  else:
    action = env.action_space.sample()

  return action


#Funcion para guardar la Qtable en un fichero json
def guardar_qtable(q_table, filename):
    with open("Tablas/" + filename + ".json", "w") as f:
        json.dump(q_table.tolist(), f)