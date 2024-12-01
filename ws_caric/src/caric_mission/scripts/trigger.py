#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2023, Cao Muqing
#  * Nanyang Technological University
#  * All Rights Reserved
#  * Authors: Cao Muqing, et al.
#  * -------------------------------------------------------------------------- */

import rospy
from rotors_comm.msg import BoolStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import time
import math
import random
import numpy as np
from numpy import linalg as LA

odom_topic_name = 'unity/odom'

number_of_robots = 3

class auto_commands:


    def __init__(self):

        self.initialized=False
        self.pubTrigger = [None for i in range(0,number_of_robots)]        
        for i in range(0,number_of_robots):
            self.pubTrigger[i]=rospy.Publisher('/firefly'+str(i+1)+'/command/trigger',BoolStamped,queue_size=1,latch=True)

    #In rospy, the callbacks are all of them in separate threads
    def odomCB(self, data, args):
        defg=1

    def logCB(self, data):
        abc=1

    def triggerCB(self, timer):
        # idx = args 
        msg = BoolStamped()
        msg.header.frame_id="world"
        msg.header.stamp = rospy.get_rostime()
        msg.data = True        
        self.pubTrigger[0].publish(msg)
        self.pubTrigger[1].publish(msg)
        self.pubTrigger[2].publish(msg)

                  
def startNode():
    c = auto_commands()
    #s = rospy.Service("/change_mode",MissionModeChange,c.srvCB)
    timerlist = [None for i in range(0,number_of_robots)]  
    for i in range(0,number_of_robots):
        rospy.Subscriber('/firefly'+str(i+1)+"/"+odom_topic_name, Odometry, c.odomCB, i)
        timerlist[i] = rospy.Timer(rospy.Duration(5.0), c.triggerCB)
    rospy.Subscriber('/firefly/log_for_plot', Odometry, c.logCB)
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('auto_commands')  
    startNode()
    rospy.loginfo("auto_commands started")