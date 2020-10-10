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

def choose_action(obs):

    print(obs)
    if obs[0] > obs[-1]:
        action = 2
    elif obs[0] < obs[-1]:
        action = 1
    else:
        action = 0

    return action
    # return random.choice([0,1,2])


if __name__ == '__main__':

    rospy.init_node('turtlebot2_maze_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('TurtleBot2Maze-v0')
    rospy.loginfo("Gym environment done")

    start_time = time.time()
    highest_reward = 0

    n_episodes = 20
    n_steps = 500

    cumulated_reward = 0

    for x in range(n_episodes):
        done = False
        
        observation = env.reset()
        # state = ''.join(map(str, observation))

        for i in range(n_steps):
            rospy.logwarn("############### Start Step=>" + str(i))
            action = choose_action(observation)
            rospy.logwarn("Next action is:%d", action)
            observation, reward, done, info = env.step(action)

            rospy.logwarn(str(observation) + " " + str(reward))
            cumulated_reward += reward

            # nextState = ''.join(map(str, observation))

            if not (done):
                rospy.logwarn("NOT DONE")
                # state = nextState
            else:
                rospy.logwarn("DONE")
                break
            rospy.logwarn("############### END Step=>" + str(i))

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logerr(("EP: " + str(x + 1) + " - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))
