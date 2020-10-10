#! /usr/bin/env python
import math
import numpy
import rospy
from sensor_msgs.msg import LaserScan
import time

def slice_list_symmetric(laser_ranges, n_obs):
    ranges_size = len(laser_ranges)
    step = int(math.ceil(float(ranges_size) / float(n_obs-1)))
    items = list(laser_ranges[0:ranges_size/2:step])
    items.extend(laser_ranges[-1:ranges_size/2:-step])
    return sorted(items)

def discretize_observation(data,new_ranges):
    """
    Discards all the laser readings that are not multiple in index of new_ranges
    value.
    """
    discretized_ranges = []
    filtered_range = []
    #mod = len(data.ranges)/new_ranges
    mod = len(data.ranges) / new_ranges
    
    print '\ntype data.ranges is', type(data.ranges)

    max_laser_value = data.range_max
    min_laser_value = data.range_min
    
    selected_values = slice_list_symmetric(data.ranges, new_ranges)

    for i, item in enumerate(data.ranges):
        if item in selected_values:
            print 'i is:', i
            if item == float ('Inf') or numpy.isinf(item):
                #discretized_ranges.append(self.max_laser_value)
                discretized_ranges.append(round(max_laser_value,1))
            elif numpy.isnan(item):
                #discretized_ranges.append(self.min_laser_value)
                discretized_ranges.append(round(min_laser_value,1))
            else:
                #discretized_ranges.append(int(item))
                discretized_ranges.append(round(item,1))
                
            if (0.5 > item > 0):
                print "done Validation >>> item=" + str(item)+"< "+str(0.5)
            else:
                print "NOT done Validation >>> item=" + str(item)+"> "+str(0.5)
            # We add last value appended
            filtered_range.append(discretized_ranges[-1])
        else:
            # We add value zero
            filtered_range.append(0.1)
                
    print "Size of observations, discretized_ranges==>"+str(len(discretized_ranges))
    
    return discretized_ranges, filtered_range

def callback(msg):
    print 'callback is called....'
    result = discretize_observation(msg, 6)
    print '\n\ndiscretize_observation', len(result[0]), '\n\n'
    print len(msg.ranges)
    time.sleep(2)
    # print 'filtered_range', result[1], '\n\n'

rospy.init_node('scan_values')
rate = rospy.Rate(1)
sub = rospy.Subscriber('/laserscan', LaserScan, callback)
rate.sleep()
# rospy.spin()