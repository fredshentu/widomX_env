#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx')
import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
import numpy as np
import time

pub = None
pub_msg = Float64MultiArray()
# pub_msg.layout.dim = 3*4

#3d array for each joint (current pos, velocity, load)
joint1_latest_data = np.zeros(3)
joint2_latest_data = np.zeros(3)
joint3_latest_data = np.zeros(3)
joint4_latest_data = np.zeros(3)

joint1_timeout = 100
joint2_timeout = 100
joint3_timeout = 100
joint4_timeout = 100

def joint1_cb(data):
    global joint1_timeout
    joint1_timeout = 100
    joint1_latest_data[0] = data.velocity
    joint1_latest_data[1] = data.current_pos
    joint1_latest_data[2] = data.load
    # print("updated!")
    
def joint2_cb(data):
    global joint2_timeout
    joint2_timeout = 100
    joint2_latest_data[0] = data.velocity
    joint2_latest_data[1] = data.current_pos
    joint2_latest_data[2] = data.load
    
def joint3_cb(data):
    global joint3_timeout
    joint3_timeout = 100
    joint3_latest_data[0] = data.velocity
    joint3_latest_data[1] = data.current_pos
    joint3_latest_data[2] = data.load

def joint4_cb(data):
    global joint4_timeout
    joint4_timeout = 100
    joint4_latest_data[0] = data.velocity
    joint4_latest_data[1] = data.current_pos
    joint4_latest_data[2] = data.load
    

def send_latest_data(data):
    data_array = np.concatenate([joint1_latest_data, joint2_latest_data, \
                                joint3_latest_data, joint4_latest_data])
    pub_msg.data = data_array
    pub.publish(pub_msg)
    
def joint_state_buffer():
    global pub
    global joint1_timeout
    global joint2_timeout
    global joint3_timeout
    global joint4_timeout
    
    rospy.init_node('joint_state_buffer', anonymous = True)
    pub = rospy.Publisher('arm_state', Float64MultiArray, queue_size = 1)
    sentinel_pub = rospy.Publisher('servo_sentinel', Empty, queue_size = 10)
    rospy.Subscriber("camera_trigger", Empty, send_latest_data)
    rospy.Subscriber("joint1_controller/state", JointState, joint1_cb)
    rospy.Subscriber("joint2_controller/state", JointState, joint2_cb)
    rospy.Subscriber("joint3_controller/state", JointState, joint3_cb)
    rospy.Subscriber("joint4_controller/state", JointState, joint4_cb)
    while 1:
        if (joint1_timeout <=0 or joint2_timeout <= 0 or joint3_timeout <= 0 or joint4_timeout <= 0):
            sentinel_pub.publish(Empty())
            #not sure what to do next, for now, just shut down the whole system
            print("fatal error latest_state can not update")
            time.sleep(1)
            # exit()
        else:
            joint1_timeout -= 1
            joint2_timeout -= 1
            joint3_timeout -= 1
            joint4_timeout -= 1
        time.sleep(0.05)
if __name__ == '__main__':
    joint_state_buffer()