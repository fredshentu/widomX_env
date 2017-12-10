#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx')
import rospy
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Float64MultiArray, Empty
import numpy as np
import argparse
import time

INNER_LOOP_FEQ = 4
TIME_INTERVAL = 1.0/INNER_LOOP_FEQ/10
SERVICE_NAME = None

current_torque = 0.0
target_torque = 0.0
steps = 0.0

timeout = 0
zero_torque = 1
time_penality = 0

loop_counter = 0

def set_torque_cb(data):
	global loop_counter
	print(SERVICE_NAME + str(loop_counter))
	loop_counter = 0
	global time_penality
	global zero_torque
	global current_torque
	global target_torque
	global steps
	global timeout
	timeout = INNER_LOOP_FEQ * 2
	time_penality = 0
	zero_torque = 0 #torque not zero env start torque controller
	current_torque = target_torque
	target_torque = data.data
	steps = (target_torque - current_torque)/INNER_LOOP_FEQ
	
	# print("command received\ncurrent:{}, target:{}, step:{}".format(current_torque, target_torque, steps))
	
def stop_interpolation(data):
	print(SERVICE_NAME + "set torque, step to zero")
	global zero_torque
	global current_torque
	global target_torque
	global steps
	global time_penality
	zero_torque = 1
	current_torque = 0.0
	target_torque = 0.0
	steps = 0.0
	rospy.wait_for_service(SERVICE_NAME + '/set_torque_limit')
	try:
		_set_torque = rospy.ServiceProxy(SERVICE_NAME + '/set_torque_limit', SetTorqueLimit)
		return1 = _set_torque(0)
		return None
	except rospy.ServiceException, e:
		print("service %s call failed: %s"%SERVICE_NAME%e)
	
	
def set_torque(torque, service_name, pos_pub):
	#set the direction
	if torque > 0.0:
		pos_pub.publish(6.28)
	else:
		pos_pub.publish(0.0)
		
	time.sleep(0.01)
	rospy.wait_for_service(service_name + '/set_torque_limit')
	try:
		_set_torque = rospy.ServiceProxy(service_name + '/set_torque_limit', SetTorqueLimit)
		return1 = _set_torque(np.abs(torque))
		return None
	except rospy.ServiceException, e:
		print("service %s call failed: %s"%service_name%e)
	
if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("joint_name", type = str, \
						help = "name of this joint")
	parser.add_argument("service_name", type = str,\
						help = "name of the service control the torque limit")
	parser.add_argument("command_topic_name", type = str, \
						help = "name of the topic control the position of this servo")
	#hard code, remove in the future 
	parser.add_argument("_", type = str, \
						help = "no")
	parser.add_argument("_1", type = str, \
						help = "no")
	args = parser.parse_args()
	SERVICE_NAME = args.service_name
	rospy.init_node(args.joint_name, anonymous = True)
	rospy.Subscriber(args.joint_name + "/set_torque", Float64, set_torque_cb)
	rospy.Subscriber("stop_linear_interpolation", Empty, stop_interpolation)
	joint_pos_pub = rospy.Publisher(args.command_topic_name + "/command", Float64, queue_size = 1)
	print (SERVICE_NAME + "finished initializing the node")
	while (1):
		now = time.time()
		if zero_torque:
			pass
		else:
			loop_counter += 1
			current_torque = np.clip(current_torque + steps, -np.abs(target_torque), np.abs(target_torque))
			#print steps
			set_torque(current_torque, args.service_name, joint_pos_pub)
			timeout -=1
			if timeout < 0:
				print (SERVICE_NAME + "Fatal error, env crashed, set torque to zero")
				timeout = INNER_LOOP_FEQ * 2
				zero_torque = 1
				current_torque = 0.0
				target_torque = 0.0
				steps = 0.0
				set_torque(0.0,args.service_name, joint_pos_pub)
		elapsed = time.time() - now + time_penality
		#print(elapsed)
		if elapsed <= TIME_INTERVAL:
			time.sleep(TIME_INTERVAL - elapsed) #80hz inner loop
			time_penality = 0
		else:
			time_penality = elapsed - TIME_INTERVAL
			print(SERVICE_NAME + "timeout, penality is:{}".format(time_penality))
		