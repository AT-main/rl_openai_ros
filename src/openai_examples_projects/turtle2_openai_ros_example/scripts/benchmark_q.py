#!/usr/bin/env python

import gym
import json
import numpy
import time
import os
import random
import qlearn
from gym import wrappers
# ROS packages required
import rospy
import rospkg
# import our training environment
from openai_ros.task_envs.turtlebot2 import turtlebot2_maze

def choose_action(Q_table, state, actions):
    # q = []
    # modified = False
    # for action in actions:
    #     if Q_table.get((state, action)):
    #         q.append(Q_table[(state, action)])
    #     elif state.startswith('0.5') and (action==0 or action==2) and (not state.endswith('0.5')):
    #         q.append(-100.0)
    #         modified = True
    #     elif state.endswith('0.5') and (action==0 or action==1) and (not state.endswith('0.5')):
    #         q.append(-100.0)
    #         modified = True
    #     else:
    #         q.append(0.0)

    q = [Q_table.get((state, action), 0.0) for action in actions]
    maxQ = max(q)

    # print '\n\nstate:', state
    # print '\nq:', q

    count = q.count(maxQ)
    # In case there're several state-action max values
    # we select a random one among them
    if count > 1:
        best = [i for i in range(len(actions)) if q[i] == maxQ]
        i = random.choice(best)
    else:
        i = q.index(maxQ)

    action = actions[i]
    # print 'selected action: ', action, '\n\n'

    # time.sleep(5) if modified else time.sleep(0.5)

    return action


if __name__ == '__main__':

    rospy.init_node('turtlebot2_maze_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('TurtleBot2Maze-v0')
    rospy.loginfo("Gym environment done")

    actions = range(env.action_space.n)
    start_time = time.time()
    highest_reward = 0

    n_episodes = 20
    n_steps = 500

    # Load Q_table from file
    Q_table_dir = '/home/amir/.ros/Q_tables/'
    file_name   = "saved_Q_2020-09-15T09.json"
    # file_name = 'saved_Q_2020-07-17T07:36:28.json'
    # file_name = 'saved_Q_2020-07-16T22:43:29.json'
    if os.path.isfile(Q_table_dir+file_name):
        with open(Q_table_dir+file_name, 'r') as q_file:
            read_dict = json.loads(q_file.read())
            raw_q = read_dict['q_table']
            Q_table = {eval(key): raw_q[key] for key in raw_q}
    else:
        raise Exception('File not found')

    cumulated_reward = 0

    for x in range(n_episodes):
        done = False

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))

        # Show on screen the actual situation of the robot
        # env.render()
        # for each episode, we test the robot for nsteps
        for i in range(n_steps):
            rospy.logwarn("############### Start Step=>" + str(i))
            # Pick an action based on the current state
            action = choose_action(Q_table, state, actions)
            rospy.logwarn("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)

            rospy.logwarn(str(observation) + " " + str(reward))
            cumulated_reward += reward

            nextState = ''.join(map(str, observation))

            if not (done):
                rospy.logwarn("NOT DONE")
                state = nextState
            else:
                rospy.logwarn("DONE")
                break
            rospy.logwarn("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            # rospy.sleep(2.0)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logerr(("EP: " + str(x + 1) + " - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

    Benchmarks_dir = '/home/amir/.ros/Benchmarks/'
    with open(Benchmarks_dir+file_name, 'w') as b_file:
        json.dump({
            'rewards': cumulated_reward,
            'episodes': n_episodes,
            'average_reward': cumulated_reward/n_episodes}, b_file)
