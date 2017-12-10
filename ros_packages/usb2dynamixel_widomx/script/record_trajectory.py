#!/usr/bin/env python
#import roslib; roslib.load_manifest('usb2dynamixel_widomx') #this step is necessary in order to use the customized messag
import roslib; roslib.load_manifest('widomX')
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
import time
import pdb
from dynamixel_controllers.srv import *
import pickle

state_flag = 0
state = None
def state_cb(data):
	global state
	global state_flag
	state = [data.data[1],data.data[4],data.data[7],data.data[10]] 
	state_flag = 1
	print("data back, set flag")

FILE_NAME = "reset_traj_recorded.pkl"
pos_list = []
if __name__ == "__main__":
	rospy.init_node('pos_recorder', anonymous=True)
	obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 10)
	rospy.Subscriber('arm_state', Float64MultiArray, state_cb)

	print("start recording in 3 sec")
	time.sleep(1)
	print("start recording in 2 sec")
	time.sleep(1)
	print("start recording in 1 sec")
	time.sleep(1)
	print("start recording")
	
	for i in range(60):
		obs_trigger.publish(Empty())
		while not state_flag:
			#print("waitiing")
			#time.sleep(0.5)
			pass
		state_flag = 0
		pos_list.append(state)
		print("recording")
		time.sleep(0.06)
	
	print("stop recording, save traj and exit...")
	save_dict = {"pos_list":pos_list}
	import pdb; pdb.set_trace()
	print pos_list
	with open(FILE_NAME, 'wb') as handle:
		pickle.dump(save_dict, handle, protocol = pickle.HIGHEST_PROTOCOL)
	print("traj saved")