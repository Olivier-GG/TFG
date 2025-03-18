import numpy as np
import random
import json


def train_agent(env, Q, n_training_episodes, max_steps, gamma, learning_rate, epsilon, min_epsilon, decay_rate, Qtable, max_epsilon):

    for episode in range(n_training_episodes):
        # Reduce epsilon, para aumentar la explotación y disminuir la exploración
        epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay_rate*episode)

        #Printemaos al final de cada episodio para ver como va la Qtable
        print("\n\n |||||||||||||||||||||||||||||")
        print("Episodio: " + str(episode))
        print ("Epsilon: " + str(epsilon))
        print(Qtable)
        print("||||||||||||||||||||||||||||||| \n\n")

        # Reseteamos el enviroment
        state, info = env.reset()
        step = 0
        terminated = False

        
        for step in range(max_steps):
            # Elegimos la accion segun nuestra politica
            action = epsilon_greedy_policy(Qtable, state, epsilon, env)
            #print( "Action: " + str(action))

            # Take action At and observe Rt+1 and St+1
            # Take the action (a) and observe the outcome state(s') and reward (r)

            new_state, reward, terminated, truncated, info  = env.step(action)

            # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
            
            Qtable[state][action] = Qtable[state][action] + learning_rate * (reward + gamma * np.max(Qtable[new_state]) - Qtable[state][action])

            # If terminated or truncated finish the episode
            if terminated:
                break

            # Our next state is the new state
            state = new_state
        
    print("Entrenamiento finalizado, puede proceder a evaluarlo")



# Evaluate the agent
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



#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#Funciones para la political de exploracion y explotacion

def greedy_policy(Qtable, state):
  # Exploitation: take the action with the highest state, action value
  action = np.argmax(Qtable[state][:])

  return action

def epsilon_greedy_policy(Qtable, state, epsilon, env):
  # Randomly generate a number between 0 and 1
  random_num = random.uniform(0,1)
  # if random_num > greater than epsilon --> exploitation
  if random_num > epsilon:
    # Take the action with the highest value given a state
    # np.argmax can be useful here
    action = greedy_policy(Qtable, state)
  # else --> exploration
  else:
    action = env.action_space.sample()

  return action

