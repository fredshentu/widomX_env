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
from std_msgs.msg import Float64MultiArray, Float64, UInt8MultiArray
import time
import pdb
from dynamixel_controllers.srv import *
import pickle

'''
mock gym env for the real robot, the observation is 
raw pixel from two cameras and joint pos&speed of 4 servos

use with usb2dynamixel_widomx package and usb2dynamixel_widomx

current support API:
step: publish torque to robot
reset: set all torque to zero
set_joint_position: position control

'''

class MockGymEnv():
	def __init__(self, trajectory_file = None):
		self.recorded_trajectory = None
		if trajectory_file != None:
			with open(trajectory_file, "rb") as handler:
				data_dict = pickle.load(handler)#,encoding='latin1')
			self.recorded_trajectory = data_dict['pos_list']
		
		self.observation = None
		self.state = None
		self.audio = None
		self.cam_flag = 0
		self.data_flag = 0
		self.audio_flag = 0
		self.sentinel_flag = 0
		self.trigger_msg = Empty()
		
		def audio_cb(data):
			self.audio_flag = 1
			self.audio = data.data
		#return a 6 channel rgb array or 6 channel gray_scale(3 continuous images)
		def multi_cam_cb(data):
			height = data.cam1.height
			width = data.cam1.width
			# print("received image width:{}, height:{}".format(width, height))
			cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
			cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
			# print(len(cam2_data))
			cam1_data = np.reshape(cam1_data, [height ,width, 3])
			cam2_data = np.reshape(cam2_data, [height ,width, 3])
			self.observation = np.concatenate((cam1_data, cam2_data), axis = 2)
			self.cam_flag = 1
			# print("cam back")
		def state_cb(data):
			self.state = data.data
			self.data_flag = 1
			# print("data back")
		def servo_sentinel_cb(data):
			"""
			set all torque to Zero
			IF possible(with proper hardware) shut down the arm
			"""
			self.step([0,0,0,0])
			self.sentinel_flag = 1
			print("fatal ERROR, env halted")
			
			
		rospy.init_node('mock_gym_env', anonymous=True)
		rospy.Subscriber('env_camera', multi_cam, multi_cam_cb)
		rospy.Subscriber('arm_state', Float64MultiArray, state_cb)
		rospy.Subscriber('servo_sentinel', Empty, servo_sentinel_cb)
		rospy.Subscriber('arm_audio', UInt8MultiArray, audio_cb)
		self.obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 10)
		self.stop_linear_interpolation = rospy.Publisher('/stop_linear_interpolation', Empty, queue_size = 1)
		self.joint1_controller = rospy.Publisher("/joint1/set_torque", Float64, queue_size = 1)
		self.joint2_controller = rospy.Publisher("/joint2/set_torque", Float64, queue_size = 1)
		self.joint3_controller = rospy.Publisher("/joint3/set_torque", Float64, queue_size = 1)
		self.joint4_controller = rospy.Publisher("/joint4/set_torque", Float64, queue_size = 1)
		
		self.joint1_pos = rospy.Publisher("joint1_controller/command", Float64, queue_size = 1)
		self.joint2_pos = rospy.Publisher("joint2_controller/command", Float64, queue_size = 1)
		self.joint3_pos = rospy.Publisher("joint3_controller/command", Float64, queue_size = 1)
		self.joint4_pos = rospy.Publisher("joint4_controller/command", Float64, queue_size = 1)
		#test
		self.send_command([0,0,0,0])
		self.obs_trigger.publish(self.trigger_msg)
		time.sleep(1)
		self.send_command([0,0,0,0])
		self.obs_trigger.publish(self.trigger_msg)
		time.sleep(1)
		self.cam_flag = 0
		self.data_flag = 0
		self.audio_flag = 0
	#help function only return the current joint position
	def get_im(self):
		self.obs_trigger.publish(self.trigger_msg)
		while not (self.cam_flag ):
			pass
		self.cam_flag = 0
		self.data_flag = 0
		self.audio_flag = 0
		return self.observation
	def get_joint_pos(self):
		while self.sentinel_flag:
			time.sleep(1)
			
		self.obs_trigger.publish(self.trigger_msg)
		while not (self.cam_flag and self.data_flag):
			pass
		self.cam_flag = 0
		self.data_flag = 0
		self.audio_flag = 0
		joint_angle =  np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		return joint_angle
	def reset(self):
		while self.sentinel_flag:
			time.sleep(1)
			
		self.obs_trigger.publish(self.trigger_msg)
		self.send_command([0,0,0,0])
		# self.stop_linear_interpolation.publish(self.trigger_msg)
		time.sleep(1)
		while not (self.cam_flag and self.data_flag and self.audio_flag):
			pass
		self.cam_flag = 0
		self.data_flag = 0
		self.audio_flag = 0
		return self.observation, self.state, self.audio
	"""
	Step function which publishs torque to servo. Range in [-1,1]
	The observation is the observation before the action was applied
	"""
	def step_state_only(self, torque_input):
		while self.sentinel_flag:
			time.sleep(1)
			
		assert(-1<=torque_input[0] and torque_input[0]<=1)
		assert(-1<=torque_input[1] and torque_input[1]<=1)
		assert(-1<=torque_input[2] and torque_input[2]<=1)
		assert(-1<=torque_input[3] and torque_input[3]<=1)
		
		if (self.state[4] > 4.53 and torque_input[1] > 0.2):
			torque_input[1] = 0.2
		if ( self.state[4] < 1.53 and torque_input[1] < -0.2):
			torque_input[1] = -0.2
			
		if (self.state[7] > 5.8 and torque_input[2] > 0.2):
			torque_input[2] = 0.2
		if ( self.state[7] < 1.54 and torque_input[2] < -0.2):
			torque_input[2] = -0.2
			
		if (self.state[10] > 4.96 and torque_input[3] > 0.4):
			torque_input[3] = 0.4
		if ( self.state[10] < 1.31 and torque_input[3] < -0.4):
			torque_input[3] = -0.4
		self.send_command(torque_input)
		self.obs_trigger.publish(self.trigger_msg)
		time.sleep(0.04)
		while not  (self.data_flag):
			#sprint("waiting")
			pass
		self.data_flag = 0
		return self.state
		
		
	def step(self, torque_input):
		self.obs_trigger.publish(self.trigger_msg)
		while self.sentinel_flag:
			time.sleep(1)
			
		assert(-1<=torque_input[0] and torque_input[0]<=1)
		assert(-1<=torque_input[1] and torque_input[1]<=1)
		assert(-1<=torque_input[2] and torque_input[2]<=1)
		assert(-1<=torque_input[3] and torque_input[3]<=1)
		
		if (self.state[4] > 4.53 and torque_input[1] > 0.2):
			torque_input[1] = 0.2
		if ( self.state[4] < 1.53 and torque_input[1] < -0.2):
			torque_input[1] = -0.2
			
		if (self.state[7] > 5.8 and torque_input[2] > 0.2):
			torque_input[2] = 0.2
		if ( self.state[7] < 1.54 and torque_input[2] < -0.2):
			torque_input[2] = -0.2
			
		if (self.state[10] > 4.96 and torque_input[3] > 0.4):
			torque_input[3] = 0.4
		if ( self.state[10] < 1.31 and torque_input[3] < -0.4):
			torque_input[3] = -0.4
		self.send_command(torque_input)
		while not  (self.cam_flag and self.data_flag and self.audio_flag):
			#sprint("waiting")
			pass
		self.cam_flag = 0
		self.audio_flag = 0
		self.data_flag = 0
		return self.observation, self.state, self.audio
		
	#input 4d array, publish to linear_interpolation_controller
	def send_command(self, torque_command):
		while self.sentinel_flag:
			time.sleep(1)
			
		self.joint1_controller.publish(torque_command[0])
		self.joint2_controller.publish(torque_command[1])
		self.joint3_controller.publish(torque_command[2])
		self.joint4_controller.publish(torque_command[3])
	"""
	Direct torque control without linear interpolation
	"""
	def torque_direct_controll(torque):
		pass
	
	#assume in position control model...
	def step_pos(self, delta_joint_pos):
		
		#get current pos, current pos + delta_joint_pos = final pos, assume delta is small
		self.obs_trigger.publish(self.trigger_msg)
		while not self.data_flag:
			pass
		self.data_flag = 0
		self.cam_flag = 0
		self.audio_flag = 0
		goal_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]]) + \
					delta_joint_pos
		#try to go to the goal_pos, then return the final position
		self.set_pos(goal_pos, current = True)
		time.sleep(0.)
		self.obs_trigger.publish(self.trigger_msg)
		while not (self.data_flag and self.cam_flag):
			pass
		self.data_flag = 0
		self.cam_flag = 0
		self.audio_flag = 0
		current_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		return self.observation, current_pos
	#Do not use this function while running arm with arena
	def set_joint_position(self, joint_angle):
		while self.sentinel_flag:
			time.sleep(1)
			
		self.obs_trigger.publish(self.trigger_msg)
		while not  (self.cam_flag and self.data_flag and self.audio_flag):
			pass
		self.data_flag = 0
		self.cam_flag = 0
		self.audio_flag = 0
		current_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		# print(current_pos)
		target_pos = np.array(joint_angle)
		step  = (joint_angle - current_pos)/200.0
		self.set_pos(current_pos, current = True)
		time.sleep(1)
		#set torque limit (Non_zero torque limit)
		torque_limit_list= [0.6,0.4,0.2,0.2]
		for i in range(1,5):
			service_name = "/joint{}_controller/set_torque_limit".format(i)
			rospy.wait_for_service(service_name)
			try:
				_set_torque = rospy.ServiceProxy(service_name, SetTorqueLimit)
				return1 = _set_torque(torque_limit_list[i-1])
			except rospy.ServiceException as e:
				print("service call failed: %s"%e)
		print("finished setting torque limit")
		#get the current position
		
	
		for i in range(200):
			# print("moving")
			current_pos = current_pos + step
			self.set_pos(current_pos)
			time.sleep(0.02)
		self.set_pos(target_pos)
		time.sleep(1)
		self.obs_trigger.publish(self.trigger_msg)
		while not  (self.cam_flag and self.data_flag and self.audio_flag):
			pass
		self.data_flag = 0
		self.cam_flag = 0
		self.audio_flag = 0
		current_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		return self.observation, self.state, self.audio
		
	#Check if torque_limit has been set to zero before using this function    
	def set_pos(self,position, current = False):
		while self.sentinel_flag:
			time.sleep(1)
			
		#ensure the position is in arm's range
		if not current:
			assert(1.53<position[1] and position[1] < 4.53)
			assert(1.54<position[2] and position[2] < 5.8)
			assert(1.31<position[3] and position[3] < 4.96)
			
		self.joint1_pos.publish(position[0])
		self.joint2_pos.publish(position[1])
		self.joint3_pos.publish(position[2])
		self.joint4_pos.publish(position[3])
		
	#trajectory is a n*4 array, 4d position echo row
	def play_trajectory(self, angle = 0):
		while self.sentinel_flag:
			time.sleep(1)
			
		if self.recorded_trajectory == None:
			print("no traj found")
			return
		self.obs_trigger.publish(self.trigger_msg)
		while not (self.data_flag and self.cam_flag):
			pass
		self.data_flag = 0
		self.cam_flag = 0
		current_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		self.set_pos(current_pos, current = True)
		"""
		set torque limit (Non_zero torque limit)
		
		reset process:
		Lift arm to recorded_traj[0](call left for the first time.)
		rotate
		replay (only the distal 2 moving)
		
		"""
		for i in range(1,3):
			service_name = "/joint{}_controller/set_torque_limit".format(i)
			rospy.wait_for_service(service_name)
			try:
				_set_torque = rospy.ServiceProxy(service_name, SetTorqueLimit)
				return1 = _set_torque(0.8)
			except rospy.ServiceException as e:
				print("service call failed: %s"%e)
		
		print("finished setting torque limit for first two servos")
		target_pos = self.recorded_trajectory[0][:]
		target_pos[0] = current_pos[0]
		step = (target_pos - current_pos)/20
		step[2] = 0
		step[3] = 0
		for _ in range(20):
			current_pos += step
			self.set_pos(current_pos)
			time.sleep(0.07) #finished initializing 2
		for i in range(3,5):
			service_name = "/joint{}_controller/set_torque_limit".format(i)
			rospy.wait_for_service(service_name)
			try:
				_set_torque = rospy.ServiceProxy(service_name, SetTorqueLimit)
				return1 = _set_torque(0.5)
			except rospy.ServiceException as e:
				print("service call failed: %s"%e)
		self.recorded_trajectory[0][0] = angle
		step = (self.recorded_trajectory[0] - current_pos)/40.0
		joint1_step = step[0]*4
		step[0] = 0
		for i in range(40):
			current_pos += step
			self.set_pos(current_pos, current = True)
			time.sleep(0.05) #finished initalizing 3,4
		for i in range(10):
			current_pos[0] += joint1_step
			self.set_pos(current_pos, current = True)
			time.sleep(0.1)
		for pos in self.recorded_trajectory[1:]:
			pos[0] = angle
			self.set_pos(pos, current = True)
			time.sleep(0.04) 
			#positions were recorded @20hz, update @ the same frequency
	"""
	lift the arm while there is a arena below
	Before calling this function, call reset first
	
	"""
	def lift_arm(self, joint1_pos = None):
		while self.sentinel_flag:
			time.sleep(1)
		self.obs_trigger.publish(self.trigger_msg)
		while not (self.data_flag and self.cam_flag):
			pass
		self.data_flag = 0
		self.cam_flag = 0
		current_pos = np.array([self.state[1], self.state[4], self.state[7], self.state[10]])
		self.set_pos(current_pos, current = True)
		time.sleep(0.2)
		#set torque limit (Non_zero torque limit), but only for the first two servos
		for i in range(1,3):
			service_name = "/joint{}_controller/set_torque_limit".format(i)
			rospy.wait_for_service(service_name)
			try:
				_set_torque = rospy.ServiceProxy(service_name, SetTorqueLimit)
				return1 = _set_torque(0.8)
			except rospy.ServiceException as e:
				print("service call failed: %s"%e)
		
		#lift the second joint
		joint2_current = current_pos[1]
		step = (1.7 - current_pos[1])/50
		for i in range(50):
			self.joint2_pos.publish(joint2_current)
			joint2_current += step
			time.sleep(0.06)
		#move the first joint if necessary
		if joint1_pos != None:
			joint1_current = current_pos[0]
			# print("1 current{}".format(joint1_current))
			#joint1 diff is very different from case to case, thus traj length 
			#should also change
			traj_length = int(np.abs(joint1_pos - joint1_current)*10)
			print("traj_length{}".format(traj_length))
			if (joint1_pos - joint1_current)> 0:
				step = 0.1
			else:
				step = -0.1
			for i in range(traj_length-1):
				# print("1 current{}".format(joint1_current))
				self.joint1_pos.publish(joint1_current)
				joint1_current += step
				time.sleep(0.05)
			self.joint1_pos.publish(joint1_pos)
			time.sleep(1)