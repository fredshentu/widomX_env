#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import numpy as np
from std_msgs.msg import Int16MultiArray
import time
import pdb

class MockGym_pos():
    def __init__(self):
        self.observation = None
        self.state = None
        self.cam_flag = 0
        self.data_flag = 0
        #return a 6 channel rgb array or 6 channel gray_scale(3 continuous images)
        def multi_cam_cb(data):
            height = data.cam1.height
            width = data.cam1.width
            cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
            cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
            cam1_data = np.reshape(cam1_data, [height ,width, 3])
            cam2_data = np.reshape(cam2_data, [height ,width, 3])
            self.observation = np.concatenate((cam1_data, cam2_data), axis = 2)
            self.cam_flag = 1
            # print("cam back")
        def state_cb(data):
            self.state = data.data #arm position data
            self.data_flag = 1
            # print("data back")
        
        rospy.init_node('mock_gym_env', anonymous = True)
        rospy.Subscriber('env_obs', multi_cam, multi_cam_cb)
        rospy.Subscriber('armstate', Int16MultiArray, state_cb)
        
        self.set_torque = rospy.Publisher('set_servo_torque', Int16MultiArray, queue_size = 1)
        self.obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 1)
        self.send_command = rospy.Publisher('arm_command', Int16MultiArray, queue_size = 1)
        self.msg_to_send = Int16MultiArray()
        self.msg_to_send.data = [600,300,300,400]
        self.trigger_msg = Empty()
        #test
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(0.5)
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(0.5)
        self.set_torque.publish(self.msg_to_send)
        time.sleep(0.5)
        self.msg_to_send.data = [20,20,20,20]
        self.send_command.publish(self.msg_to_send)
        # import pdb; pdb.set_trace()
        while (not self.data_flag or not self.cam_flag):
            pass
        self.cam_flag = 0
        self.data_flag = 0
    def step(self, command):
        #self.state is the current state
        if (self.state[0] + command[0] > 4095):
            command[0] = 2047 - self.state[0]
        elif (self.state[0] + command[0] < 0):
            command[0] = -self.state[0]
        
        if (self.state[1] + command[1] > 2833):
            command[1] = 2833 - self.state[1]
        elif (self.state[1] + command[1] < 1000):
            command[1] = 1000-self.state[0]
            
        if (self.state[2] + command[2] > 3820):
            command[2] = 3820 - self.state[2]
        elif (self.state[2] + command[2] < 1000):
            command[2] = 1000-self.state[2]
        
        if (self.state[3] + command[3] > 3200):
            command[3] = 3200 - self.state[3]
        elif (self.state[3] + command[3] < 809):
            command[3] = 809-self.state[3]
        assert(len(command) == 4)
        self.msg_to_send.data = command
        self.send_command.publish(self.msg_to_send)
        # print("wait for state")
        while (not self.data_flag):
            pass
        self.data_flag = 0
        self.obs_trigger.publish(self.trigger_msg)
        while(not self.cam_flag):
            pass
        self.cam_flag = 0
        return self.state, self.observation
        
'''
mock gym env for the real robot, the observation is 
raw pixel from two cameras and joint pos&speed of 4 servos

While using this code, arduino should run direct_control.ino
and ros node camera2_node or camera2_node_3_gray should be activated
Robot excutes torque send by this environment directly
Current supported methods:
step:pubish torque to robot #WARNING: robot excutes torque without and range limit
reset:set all torque to zero
set_joint_position:position control
'''
class MockGymEnvWithStates():
    def __init__(self):
        self.observation = None
        self.state = Int16MultiArray()
        self.cam_flag = 0
        self.data_flag = 0
        #return a 6 channel rgb array or 6 channel gray_scale(3 continuous images)
        def multi_cam_cb(data):
            height = data.cam1.height
            width = data.cam1.width
            
            cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
            cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
            cam1_data = np.reshape(cam1_data, [height ,width, 3])
            cam2_data = np.reshape(cam2_data, [height ,width, 3])
            self.observation = np.concatenate((cam1_data, cam2_data), axis = 2)
            self.cam_flag = 1
            # print("cam back")
        def state_cb(data):
            self.state = data
            self.data_flag = 1
            # print("data back")
        rospy.init_node('mock_gym_env', anonymous = True)
        rospy.Subscriber('env_obs', multi_cam, multi_cam_cb)
        self.torque_controller = rospy.Publisher('servo_torque', Int16MultiArray, queue_size = 10)
        self.obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 10)
        self.position_controller = rospy.Publisher('servo_position', Int16MultiArray, queue_size = 10)
        
        
        rospy.Subscriber('armstate', Int16MultiArray, state_cb)
        self.msg_to_send = Int16MultiArray()
        self.msg_to_send.data = [0,0,0,0]
        self.trigger_msg = Empty()
        #test
        self.obs_trigger.publish(self.trigger_msg)
        self.torque_controller.publish(self.msg_to_send)
        time.sleep(1)
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(1)
        self.cam_flag = 0
        self.data_flag = 0

    #reset function under torque control model. Set torque to Zero and wait
    #until camera and arm node return the observation
    def reset(self):
        #reset strateg
        self.msg_to_send.data = [0,0,0,0]
        self.torque_controller.publish(self.msg_to_send)
        self.obs_trigger.publish(self.trigger_msg)
        while not (self.cam_flag and self.data_flag):
            pass
        #import pdb; pdb.set_trace()
        self.cam_flag = 0
        self.data_flag = 0
        return self.observation, self.state.data
        # return {"images":self.observation, "states": self.state}
        
    #The step function for torque controller, 4 joints, range from -1013
    #to +1023, sign corresponding to different direction and magnitude 
    #corresponding to the mangnitude of the torque
    #The obs and state returned are those before action has been applied
    def step(self, torque_input):
        assert(-1024<torque_input[0] and torque_input[0]<1024)
        assert(-1024<torque_input[1] and torque_input[1]<1024)
        assert(-1024<torque_input[2] and torque_input[2]<1024)
        assert(-1024<torque_input[3] and torque_input[3]<1024)

        #When arm hits it limit, only allow torque which drive arm to the opposite direction
        #clip torque to the same direction
        
        #upper arm lift one side:
        #HARDCODE PART
        if (self.state.data[5] > 3000 and torque_input[1] < -100):
            torque_input[1] = -100
        if ( self.state.data[5] < 1010 and torque_input[1] > 100):
            torque_input[1] = 100
            
        if (self.state.data[6] > 3860 and torque_input[2] < -100):
            torque_input[2] = -100
        if ( self.state.data[6] < 1038 and torque_input[2] > 100):
            torque_input[2] = 100
            
        if (self.state.data[7] > 3210 and torque_input[3] > 300):
            torque_input[3] = 300
        if ( self.state.data[7] < 868 and torque_input[3] < -300):
            torque_input[3] = -300
        
        
        self.msg_to_send.data = torque_input
        self.obs_trigger.publish(self.trigger_msg)
        #arduino send back state then apply new torque. Thus the state is the state
        #before action has been applied
        self.torque_controller.publish(self.msg_to_send)
        
        #waiting, will not return unitil get new data
        while not (self.cam_flag and self.data_flag):
            pass
        
        self.cam_flag = 0
        self.data_flag = 0
        return self.observation, self.state.data

    #set the joint position use self.posotion_controller to publish a 4d array as position
    #this function returns arm state after the action has been fully executted
    #
    #Under Arena arduino controller, the controller only control the position of the first
    #servo. The should lift servo will lift the arm. The last 2 will be set as free to move model
    #THIS FUNCTION also does not call camera_trigger, since we won't save obs returned by this
    #Function
    
    def set_joint_position(self, joint_angle = [0,0,0,0]):
        assert(0<=joint_angle[0] and joint_angle[0]<=4095)
        assert(1070<=joint_angle[1] and joint_angle[1]<=3000)
        assert(1060<=joint_angle[2] and joint_angle[2]<=3600)
        assert(825<=joint_angle[3] and joint_angle[3]<=3100)
        
        self.msg_to_send.data = joint_angle
        
        self.position_controller.publish(self.msg_to_send)
        while not (self.data_flag):
            pass  
        
        self.obs_trigger.publish(self.trigger_msg)
        
        while not (self.cam_flag):
            pass
        
        self.cam_flag = 0
        self.data_flag = 0
        return self.observation, self.state.data
        # return {"images":self.observation, "states": self.state}
    
class MockGymEnvWithStates_linear_interpolation(MockGymEnvWithStates):
    def __init__(self):
        
        MockGymEnvWithStates.__init__(self)
        self.set_torque_to_zero = rospy.Publisher('servo_reset', Empty, queue_size = 10)
    
    #override reset model with re-enable torque control model
    def reset(self):
        #reset strateg
        self.set_torque_to_zero.publish(self.trigger_msg)
        self.obs_trigger.publish(self.trigger_msg)
        while not (self.cam_flag and self.data_flag):
            pass
        #import pdb; pdb.set_trace()
        self.cam_flag = 0
        self.data_flag = 0
        return self.observation, self.state.data
