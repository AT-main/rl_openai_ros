#!/usr/bin/env python

from datetime import datetime
import gym
import json
import numpy
import os
import time

import qlearn
from gym import wrappers
# ROS packages required
import rospy
import rospkg
# import our training environment
from openai_ros.task_envs.turtlebot2 import turtlebot2_maze


EVERY_EPISODE = 100


if __name__ == '__main__':

    rospy.init_node('turtlebot2_maze_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('TurtleBot2Maze-v0')
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('turtle2_openai_ros_example')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/turtlebot2/alpha")
    Epsilon = rospy.get_param("/turtlebot2/epsilon")
    Gamma = rospy.get_param("/turtlebot2/gamma")
    epsilon_discount = rospy.get_param("/turtlebot2/epsilon_discount")
    nepisodes = rospy.get_param("/turtlebot2/nepisodes")
    nsteps = rospy.get_param("/turtlebot2/nsteps")

    running_step = rospy.get_param("/turtlebot2/running_step")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0
    batch_reward = 0
    batch_reward_dic = {}

    # Load Q_table from file
    file_name = "saved_Q_2020-07-17T07:36:28.json"
    file_name = os.path.join('Q_tables', file_name)

    if os.path.isfile(file_name):
        with open(file_name, 'r') as q_file:
            saved_q = json.load(q_file)['q_table']
            qlearn.q = {eval(key): saved_q[key] for key in saved_q}
            rospy.logwarn('Q table successfully loaded. Size: {0}'.format(len(qlearn.q)))
            time.sleep(5)
            qlearn.epsilon = 0.05 # 0.9 * (0.999 ** 1500)
    else:
        rospy.logerr('File not found > Q table not loaded.')

    # Starts the main training loop: the one about the episodes to do
    for x in range(1, nepisodes+1):
        rospy.logdebug("############### START EPISODE=>" + str(x))

        cumulated_reward = 0
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))

        # Show on screen the actual situation of the robot
        # env.render()
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            rospy.logwarn("############### Start Step=>" + str(i))
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.logwarn("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)
            print '\n\nDONE? >>', 'Yes' if done else 'No', '\n\n'

            rospy.logwarn(str(observation) + " " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
            rospy.logwarn("# state we were=>" + str(state))
            rospy.logwarn("# action that we took=>" + str(action))
            rospy.logwarn("# reward that action gave=>" + str(reward))
            rospy.logwarn("# episode cumulated_reward=>" + str(cumulated_reward))
            rospy.logwarn("# State in which we will start next step=>" + str(nextState))
            qlearn.learn(state, action, reward, nextState)
            if not (done):
                rospy.logwarn("NOT DONE")
                state = nextState
            else:
                rospy.logwarn("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.logwarn("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            # rospy.sleep(2.0)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logerr(("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
            round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

        batch_reward += cumulated_reward
        if not x % EVERY_EPISODE:
            batch_reward_dic[x] = batch_reward / EVERY_EPISODE
            batch_reward = 0

    env.close()

    right_now = datetime.now().isoformat()
    right_now = right_now[:right_now.rfind('.')]
    file_name = os.path.join('Q_tables', 'saved_Q_' + right_now + '.json')

    with open(file_name, 'w') as q_file:
        new_q = {str(key): qlearn.q[key] for key in qlearn.q}
        collective_dict = {
            'q_table': new_q,
            'nepisodes': nepisodes,
            'Alpha': qlearn.alpha,
            'Gamma': qlearn.gamma,
            'initial_epsilon': initial_epsilon,
            'epsilon_discount': epsilon_discount,
            'highest_reward': highest_reward,
            'batch_rewards': batch_reward_dic,
        }
        json.dump(collective_dict, q_file)

    rospy.loginfo(("\n|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" + str(
        initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    # print("Parameters: a="+str)
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    # os.system('systemctl poweroff')