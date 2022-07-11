#!/usr/bin/env python3


import rospy
from gazebo_msgs.msg import ContactsState
from spot_mini_ros.msg import ContactData


def main():
    contact_msg = ContactData()
    try:
        fr = rospy.wait_for_message('/front_right_foot', ContactsState, timeout=None)
        fl = rospy.wait_for_message('/front_left_foot', ContactsState, timeout=None)
        br = rospy.wait_for_message('/back_right_foot', ContactsState, timeout=None)
        bl = rospy.wait_for_message('/back_left_foot', ContactsState, timeout=None)
    except:
        pass
    
    # print(len([x for x in fr.states]))

    # contact_msg.FR = 0 if len(fr.states)==0 else round(sum([state.total_wrench.force.z for state in fr.states])/len(fr.states), 3)
    # contact_msg.FL = 0 if len(fl.states)==0 else round(sum([state.total_wrench.force.z for state in fl.states])/len(fl.states), 3)
    # contact_msg.BR = 0 if len(br.states)==0 else round(sum([state.total_wrench.force.z for state in br.states])/len(br.states), 3)
    # contact_msg.BL = 0 if len(bl.states)==0 else round(sum([state.total_wrench.force.z for state in bl.states])/len(bl.states), 3)


    contact_msg.FR = False if len(fr.states)==0 else True
    contact_msg.FL = False if len(fl.states)==0 else True
    contact_msg.BR = False if len(br.states)==0 else True
    contact_msg.BL = False if len(bl.states)==0 else True

    try:
        pub.publish(contact_msg)
    except:
        print(contact_msg)



if __name__ == '__main__':
    rospy.init_node('contact_data_node', anonymous=True)
    pub = rospy.Publisher('/contact_data', ContactData , queue_size=1)
    while not rospy.is_shutdown() or KeyboardInterrupt:
        try:
            main()
        except AttributeError:
            pass