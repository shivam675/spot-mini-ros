#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
import threading



FLFJ_PUB = rospy.Publisher('/FLFJ_controller/command', Float64, queue_size=1)
FLLLJ_PUB = rospy.Publisher('/FLLLJ_controller/command', Float64, queue_size=1)
FRFJ_PUB = rospy.Publisher('/FRFJ_controller/command', Float64, queue_size=1)
FRLLJ_PUB = rospy.Publisher('/FRLLJ_controller/command', Float64, queue_size=1)
RLFJ_PUB = rospy.Publisher('/RLFJ_controller/command', Float64, queue_size=1)
RLLLJ_PUB = rospy.Publisher('/RLLLJ_controller/command', Float64, queue_size=1)
RRFJ_PUB = rospy.Publisher('/RRFJ_controller/command', Float64, queue_size=1)
RRLLJ_PUB = rospy.Publisher('/RRLLJ_controller/command', Float64, queue_size=1)


class Publisher(threading.Thread):
    def __init__(self, FLFJ_PUB, FLLLJ_PUB, FRFJ_PUB, FRLLJ_PUB, 
    RLFJ_PUB, RLLLJ_PUB, RRFJ_PUB, RRLLJ_PUB, rate):
        threading.Thread.__init__(self)
        self.counter = 0
        self.FLFJ_PUB = FLFJ_PUB 
        self.FLLLJ_PUB = FLLLJ_PUB
        self.FRFJ_PUB = FRFJ_PUB
        self.FRLLJ_PUB = FRLLJ_PUB
        self.RLFJ_PUB = RLFJ_PUB
        self.RLLLJ_PUB = RLLLJ_PUB
        self.RRFJ_PUB = RRFJ_PUB
        self.RRLLJ_PUB = RRLLJ_PUB
        self.rate = rate
    
    def run(self):
        (self.FLFJ_PUB,self.FLLLJ_PUB,self.FRFJ_PUB,self.FRLLJ_PUB,
            self.RLFJ_PUB,self.RLLLJ_PUB,self.RRFJ_PUB,self.RRLLJ_PUB,
            self.rate, self.counter)


class JointPub(object):
    def __init__(self):
        self.rate = rospy.Rate(10)

        thread = Publisher(FLFJ_PUB, FLLLJ_PUB, FRFJ_PUB, FRLLJ_PUB, 
        RLFJ_PUB, RLLLJ_PUB, RRFJ_PUB, RRLLJ_PUB, rate=self.rate)

        thread.start()

        self.init_pos = {
           "FLFJ": 0.0,
           "FLLLJ": 0.0,
           "FRFJ": 0.0,
           "FRLLJ": 0.0,
           "RLFJ": 0.0,
           "RLLLJ": 0.0,
           "RRFJ" : 0,
           "RRLLJ":0 }

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)


    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """

        rate = rospy.Rate(50)  # 10hz
        
        
        while (FLFJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (FLLLJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (FRFJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (FRLLJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (RLFJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (RLLLJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (RRFJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
        while (RRLLJ_PUB.get_num_connections() == 0):
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def move_joints(self, action_dict):
        
        FLFJ_msg = Float64()
        FLFJ_msg.data = action_dict['FLFJ']
        FLLLJ_msg = Float64()
        FLLLJ_msg.data = action_dict['FLLLJ']
        FRFJ_msg = Float64()
        FRFJ_msg.data = action_dict['FRFJ']
        FRLLJ_msg = Float64()
        FRLLJ_msg.data = action_dict['FRLLJ']
        RLFJ_msg = Float64()
        RLFJ_msg.data = action_dict['RLFJ']
        RLLLJ_msg = Float64()
        RLLLJ_msg.data = action_dict['RLLLJ']
        RRFJ_msg = Float64()
        RRFJ_msg.data = action_dict['RRFJ']
        RRLLJ_msg = Float64()
        RRLLJ_msg.data = action_dict['RRLLJ']

        FLFJ_PUB.publish(FLFJ_msg) 
        FLLLJ_PUB.publish(FLLLJ_msg)
        FRFJ_PUB.publish(FRFJ_msg)
        FRLLJ_PUB.publish(FRLLJ_msg)
        RLFJ_PUB.publish(RLFJ_msg)
        RLLLJ_PUB.publish(RLLLJ_msg)
        RRFJ_PUB.publish(RRFJ_msg)
        RRLLJ_PUB.publish(RRLLJ_msg)

        self.rate.sleep()