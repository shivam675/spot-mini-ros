#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
# import numpy
import time
import math
import numpy as np
from std_msgs.msg import Float32MultiArray



class WalkerState(object):

    def __init__(self, desired_x = 6, done_reward = -1000.0):
        rospy.loginfo("Starting spot State Class object...")
        
        self._abs_max_roll = 1.571
        self._done_reward = done_reward
        self.y_pos_reward = 50
        self.forward_reward_weight = 4
        self.ctrl_cost_weight = 0.2
        self.z_pos_reward = 10
        self.goal_pose_weight = 50
        ###########################################
        self.current_step_reward = 0
        # self.step_time = time.time()
        self.get_it_right = []
        self.step_number = 0
        self.desired_x = desired_x #in mtrs
        self.good_done_reward = 1000
        ###########################################
        self.front_left_contact_force = 0
        self.front_right_contact_force = 0
        self.rear_left_contact_force = 0
        self.rear_right_contact_force = 0
        ############ OBS VALS ####################
        self.base_position = None 
        self.base_orientation = None
        self.base_linear_vel = None
        self.base_angular_vel = None
        self.base_linear_acceleration = None

        self.current_action_to_be_done = {
            'FLFJ': 0, 'FLLLJ': 0,
            'FRFJ': 0, 'FRLLJ': 0,
            'RLFJ': 0, 'RLLLJ': 0,
            'RRFJ': 0, 'RRLLJ': 0,
        }

        self._list_of_observations = [
        "base_link_pos_x","base_link_pos_y","base_link_pos_z",
        "FLFJ_joint_pos", "FLFJ_joint_vel", "FLLLJ_joint_pos", "FLLLJ_joint_vel",
        "FRFJ_joint_pos", "FRFJ_joint_vel", "FRLLJ_joint_pos", "FRLLJ_joint_vel",
        "RLFJ_joint_pos", "RLFJ_joint_vel", "RLLLJ_joint_pos", "RLLLJ_joint_vel",
        "RRFJ_joint_pos", "RRFJ_joint_vel", "RRLLJ_joint_pos", "RRLLJ_joint_vel",
        "base_link_orientation_x", "base_link_orientation_y", "base_link_orientation_z",
        "base_linear_vel_x", "base_linear_vel_y", "base_linear_vel_z",
        "base_angular_vel_x", "base_angular_vel_y", "base_angular_vel_z",
        "footfr_contact", "footfl_contact", "footrr_contact", "footrl_contact",]

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_linear_acceleration = Vector3()
        self.left_contact_force = Vector3()
        self.right_contact_force = Vector3()
        self.joint_states = JointState()

        # Odom we only use it for the height detection and planar position ,
        #  because in real robots this data is not trivial.

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/spot/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/front_left_foot", ContactsState, self.front_left_contact_sensor)
        rospy.Subscriber("/front_right_foot", ContactsState, self.front_right_contact_sensor)
        rospy.Subscriber("/back_left_foot", ContactsState, self.rear_left_contact_sensor)
        rospy.Subscriber("/back_right_foot", ContactsState, self.rear_right_contact_sensor)
        rospy.Subscriber("/joint_states", JointState, self.joints_state_callback)

        

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """

        self.current_step_reward = 0

        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                self.base_position = data_pose.pose.pose.position
                rospy.loginfo("Current odom READY")
            except:
                rospy.loginfo("Current odom pose not ready yet, retrying for getting robot base_position")

        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message("/spot/imu/data", Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_linear_acceleration = imu_data.linear_acceleration
                rospy.loginfo("Current imu_data READY")
            except:
                rospy.loginfo("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")


        front_left_contacts_data = None
        while front_left_contacts_data is None and not rospy.is_shutdown():
            try:
                front_left_contacts_data = rospy.wait_for_message("/front_left_foot", ContactsState, timeout=0.1)
                for state in front_left_contacts_data.states:
                    self.front_left_contact_force = state.total_wrench.force.z
                rospy.loginfo("Current LEFT contacts_data READY")
            except:
                rospy.loginfo("Current LEFT contacts_data not ready yet, retrying")

        front_right_contacts_data = None
        while front_right_contacts_data is None and not rospy.is_shutdown():
            try:
                front_right_contacts_data = rospy.wait_for_message("/front_right_foot", ContactsState, timeout=0.1)
                for state in front_right_contacts_data.states:
                    self.front_right_contact_force = state.total_wrench.force.z
                rospy.loginfo("Current RIGHT contacts_data READY")
            except:
                rospy.loginfo("Current RIGHT contacts_data not ready yet, retrying")

        rear_left_contacts_data = None
        while rear_left_contacts_data is None and not rospy.is_shutdown():
            try:
                rear_left_contacts_data = rospy.wait_for_message("/back_left_foot", ContactsState, timeout=0.1)
                for state in rear_left_contacts_data.states:
                    self.right_contact_force = state.total_wrench.force
                rospy.loginfo("Current RIGHT contacts_data READY")
            except:
                rospy.loginfo("Current RIGHT contacts_data not ready yet, retrying")
        
        rear_right_contacts_data = None
        while rear_right_contacts_data is None and not rospy.is_shutdown():
            try:
                rear_right_contacts_data = rospy.wait_for_message("/back_right_foot", ContactsState, timeout=0.1)
                for state in rear_right_contacts_data.states:
                    self.right_contact_force = state.total_wrench.force
                rospy.loginfo("Current RIGHT contacts_data READY")
            except:
                rospy.loginfo("Current RIGHT contacts_data not ready yet, retrying")


        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.loginfo("Current joint_states READY")
            except Exception as e:
                rospy.loginfo("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.loginfo("ALL SYSTEMS READY")


    def get_base_height(self):
        return abs(self.base_position.z)

    def get_base_rpy(self):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])
        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = np.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = np.array((p_end.x, p_end.y, p_end.z))
        distance = np.linalg.norm(a - b)
        return distance

    # def get_joint_states(self):
    #     return self.joints_state

    ###################################################################
    ###################################################################

    def front_left_contact_sensor(self,msg):
        if len(msg.states)==0:
            self.front_left_contact_force = 0
        else:
            for state in msg.states:
                self.front_left_contact_force = state.total_wrench.force.z
                
    def front_right_contact_sensor(self,msg):
        if len(msg.states)==0:
            self.front_right_contact_force = 0
        else:
            for state in msg.states:
                self.front_right_contact_force = state.total_wrench.force.z
                
    def rear_left_contact_sensor(self,msg):
        if len(msg.states)==0:
            self.rear_left_contact_force = 0
        else:
            for state in msg.states:
                self.rear_left_contact_force = state.total_wrench.force.z
                
    def rear_right_contact_sensor(self,msg):
        if len(msg.states)==0:
            self.rear_right_contact_force = 0
        else:
            for state in msg.states:
                self.rear_right_contact_force = state.total_wrench.force.z
                
    ###################################################################
    ###################################################################


    def odom_callback(self,msg):
        self.base_position = msg.pose.pose.position
        self.base_linear_vel = msg.twist.twist.linear

    def imu_callback(self,msg):
        self.base_orientation = msg.orientation
        self.base_linear_acceleration = msg.linear_acceleration
        self.base_angular_vel = msg.angular_velocity
    
    def bot_orientation_ok(self):
        orientation_rpy = self.get_base_rpy()
        roll_ok = self._abs_max_roll > abs(orientation_rpy.x)
        return roll_ok

    def joints_state_callback(self,msg):
        self.joint_states = msg

################################ REWARD CODE ##########################################################
#######################################################################################################


    def calculate_x_linear_velocity_reward(self, weight=10):
        a = self.base_linear_vel.x
        return (a*weight)**2

    # def calculate_x_displacement_reward(self, weight=3):
    #     a = self.base_position.x
    #     return (a**2)*weight

    def calculate_z_lateral_displacement_reward(self, weight=50):
        """
        Ideal value must be between 0.264
        """
        if self.base_position.z > 0.27 or self.base_position.z < 0.18:
            return (abs(0.264 - self.base_position.z))*weight
        else:
            return 0
    
    def calculate_y_lateral_displacement_reward(self, weight=50):
        return (abs(self.base_position.y)**2)*weight
        


    def calculate_reward_joint_effort(self, weight=0.02):
        """
        We calculate reward base on the joints effort readings. 
        The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = sum([abs(x) for x in list(self.get_joint_state_positions().values())])
        return weight*acumulated_joint_effort


    def calculate_total_reward(self):
        """
        We consider VERY BAD REWARD -100 or less
        """

        r1 = self.calculate_x_linear_velocity_reward(self.forward_reward_weight)
        r2 = self.calculate_y_lateral_displacement_reward(self.y_pos_reward)
        r3 = self.calculate_z_lateral_displacement_reward(self.z_pos_reward)
        r4 = 1 # alive reward
        r5 = self.calculate_reward_joint_effort(self.ctrl_cost_weight)
        total_reward = r1 -r2 -r3 + r4 - r5

        return total_reward

#######################################################################################################
#######################################################################################################

#######################################################################################################
########################################## Obsservation code ##########################################

    def get_observations(self):

        base_orientation = self.get_base_rpy()
        joint_states_dict = self.get_joint_state_positions()
        dict_of_angular_vels = self.get_angular_vels()

        base_r = base_orientation.x
        base_p = base_orientation.y
        base_y = base_orientation.z

        observation = np.array([
        self.base_position.x,
        self.base_position.y,
        self.base_position.z,
        joint_states_dict['FLFJ'],
        dict_of_angular_vels['FLFJ'],
        joint_states_dict['FLLLJ'],
        dict_of_angular_vels['FLLLJ'],
        joint_states_dict['FRFJ'],
        dict_of_angular_vels['FRFJ'],
        joint_states_dict['FRLLJ'],
        dict_of_angular_vels['FRLLJ'],
        joint_states_dict['RLFJ'],
        dict_of_angular_vels['RLFJ'],
        joint_states_dict['RLLLJ'],
        dict_of_angular_vels['RLLLJ'],
        joint_states_dict['RRFJ'],
        dict_of_angular_vels['RRFJ'],
        joint_states_dict['RRLLJ'],
        dict_of_angular_vels['RRLLJ'],
        base_r,
        base_p,
        base_y,
        self.base_linear_vel.x,
        self.base_linear_vel.y,
        self.base_linear_vel.z,
        self.base_angular_vel.x,
        self.base_angular_vel.y,
        self.base_angular_vel.z,
        self.front_left_contact_force,
        self.front_right_contact_force,
        self.rear_left_contact_force,
        self.rear_right_contact_force,
        self.current_action_to_be_done['FLFJ'],
        self.current_action_to_be_done['FLLLJ'],
        self.current_action_to_be_done['FRFJ'],
        self.current_action_to_be_done['FRLLJ'],
        self.current_action_to_be_done['RLFJ'],
        self.current_action_to_be_done['RLLLJ'],
        self.current_action_to_be_done['RRFJ'],
        self.current_action_to_be_done['RRLLJ'],
        ])

        return observation


    def dump_previous_actions(self, action,step_number):
        self.current_action_to_be_done = {
            'FLFJ': action[0], 
            'FLLLJ': action[1],
            'FRFJ': action[2],
            'FRLLJ': action[3],
            'RLFJ': action[4],
            'RLLLJ': action[5],
            'RRFJ': action[6],
            'RRLLJ': action[7],
            }
        self.step_number = step_number 
        return self.current_action_to_be_done

    def process_data(self):
        done = (self.step_number >= 150) or (not self.bot_orientation_ok())
        if done:
            total_reward = self._done_reward
        elif self.base_position.x >= self.desired_x:
            total_reward = self.good_done_reward + self.calculate_total_reward()
            done = True
        else:
            total_reward = self.calculate_total_reward()

        return total_reward, done

    
    def get_joint_state_positions(self):
        jsp = self.joint_states.position
        jsn = self.joint_states.name
        if len(jsp) == 0:
            jsp = [0, 0, 0, 0, 0, 0, 0, 0]
        dict_of_joint_positions = {}
        for idx, joint in enumerate(jsn):
            dict_of_joint_positions[joint] = jsp[idx]
        return dict_of_joint_positions

    def get_angular_vels(self):
        names = self.joint_states.name
        angular_vels = self.joint_states.velocity
        if len(angular_vels) == 0:
            angular_vels = [0, 0, 0, 0, 0, 0, 0, 0]
        main_dict = {}
        for idx, name in enumerate(names):
            main_dict[name] = angular_vels[idx]
        return main_dict
            