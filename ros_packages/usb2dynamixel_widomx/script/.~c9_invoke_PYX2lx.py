#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx')
import rospy
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import argparse
import time

INNER_LOOP_HZ = 15

current_torque = 0
target_torque = 0
steps = 0

timeout = 0
zero_torque = 1

set_torque_cb(data):
    zero_torque = 0 #torque not zero env start torque controller
    current_torque = target_torque
    target_torque = data
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("joint_name", type = str, \
                        help = "name of this joint")
    parser.add_argument("service_name", type = str,\ #joint*_controller/set_torque_limit
                        help = "name of the service control the torque limit")
    parser.add_argumemt("command_topic_name", type = str, \ #joint*_controller/command
                        help = "name of the topic control the position of this servo")
    
    args = parser.parse_args()
    
    rospy.init_node(args.joint_name, anonymous = True)
    rospy.Subscriber(args.joint_name + "/set_torque", Float64, set_torque_cb)