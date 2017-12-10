#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx')
import rospy
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import time

'''
should use topic to do this instead of service. However, limited by the current API
I am using service call to change torque now

Lower level controller, like the code running in arduino
'''
rospy.init_node('linear_interpolation_controller', anonymous = True)
joint1pub = rospy.Publisher('joint1_controller/command', Float64, queue_size = 1) #0-6.28
joint2pub = rospy.Publisher('joint2_controller/command', Float64, queue_size = 1)
joint3pub = rospy.Publisher('joint3_controller/command', Float64, queue_size = 1)
joint4pub = rospy.Publisher('joint4_controller/command', Float64, queue_size = 1)

current_torque = np.zeros(4)
target_torque = np.zeros(4)
steps = np.zeros(4)
#accept a 4 dim Float64MultiArray
def arm_command_cb(data):
    target_torque = data.data
    print (target_torque)
    
rospy.Subscriber("arm_command", Float64MultiArray, arm_command_cb)

#torque range -1 ~ 1 two directions
def joint1_torque_controller(torque):
    rospy.wait_for_service('joint1_controller/set_torque_limit')
    try:
        set_torque = rospy.ServiceProxy('joint1_controller/set_torque_limit', SetTorqueLimit)
        return1 = set_torque(np.abs(torque))
        #then set the direction
        if torque >= 0:
            joint1pub.publish(6.28)
        else:
            joint1pub.publish(0)
        return return1
    except rospy.ServiceException, e:
        print("service joint1 call failed: %s"%e)

def joint2_torque_controller(torque):
    rospy.wait_for_service('joint2_controller/set_torque_limit')
    try:
        set_torque = rospy.ServiceProxy('joint2_controller/set_torque_limit', SetTorqueLimit)
        return1 = set_torque(np.abs(torque))
        #then set the direction
        if torque >= 0:
            joint2pub.publish(6.28)
        else:
            joint2pub.publish(0)
        return return1
    except rospy.ServiceException, e:
        print("service joint2 call failed: %s"%e)

def joint3_torque_controller(torque):
    rospy.wait_for_service('joint3_controller/set_torque_limit')
    try:
        set_torque = rospy.ServiceProxy('joint3_controller/set_torque_limit', SetTorqueLimit)
        return1 = set_torque(np.abs(torque))
        #then set the direction
        if torque >= 0:
            joint3pub.publish(6.28)
        else:
            joint3pub.publish(0)
        return return1
    except rospy.ServiceException, e:
        print("service joint3 call failed: %s"%e)

def joint4_torque_controller(torque):
    rospy.wait_for_service('joint4_controller/set_torque_limit')
    try:
        set_torque = rospy.ServiceProxy('joint4_controller/set_torque_limit', SetTorqueLimit)
        return1 = set_torque(np.abs(torque))
        #then set the direction
        if torque >= 0:
            joint4pub.publish(6.28)
        else:
            joint4pub.publish(0)
        return return1
    except rospy.ServiceException, e:
        print("service joint4 call failed: %s"%e)

function_list = [joint1_torque_controller, joint2_torque_controller, joint3_torque_controller,\
                                            joint4_torque_controller]
                                            
"""
For now, 4 servo share the same linear_interpolation_controller
If we need faster controller in the furture we can use 4 controllers 
instead of just one controller to control 4 servos
"""
if __name__ == "__main__":
    #safty feature: if no new torque command come in for a while, set all torque to zero
    while 1:
        
        now = time.time()
        #calculate torque need to apply
        for i in range(4):
            function_list[i](current_torque[i])
        
        elapsed = time.time() - now
        if elapsed < 0.02:
            time.sleep(0.02 - elapsed)
        else:
            print("timeout, error this loop takes {}".format(elapsed))
            