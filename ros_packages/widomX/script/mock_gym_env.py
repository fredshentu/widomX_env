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
#mock gym env for the real robot, the observation is 
#raw pixel from two cameras or joint states
class MockGymEnv():
    def __init__(self):
        self.observation = None
        #return a 6 channel rgb array
        def multi_cam_cb(data):
            #pdb.set_trace()
            height = data.cam1.height
            width = data.cam1.width
            cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
            cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
            cam1_data = np.reshape(cam1_data, [height ,width, 3])
            cam2_data = np.reshape(cam2_data, [height ,width, 3])
            self.observation = np.concatenate((cam1_data, cam2_data), axis = 2)
            
        rospy.init_node('mock_gym_env', anonymous = True)
        rospy.Subscriber('env_obs', multi_cam, multi_cam_cb)
        self.torque_controller = rospy.Publisher('servo', Int16MultiArray, queue_size = 10)
        self.obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 10)
        self.msg_to_send = Int16MultiArray()
        self.trigger_msg = Empty()
        #test
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(2)
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(2)
        # import pdb; pdb.set_trace()
    def step(self, torque_input):
        assert(-1024<torque_input[0] and torque_input[0]<1024)
        assert(-1024<torque_input[1] and torque_input[1]<1024)
        assert(-1024<torque_input[2] and torque_input[2]<1024)
        assert(-1024<torque_input[3] and torque_input[3]<1024)
        # print ("self.obs before call back")
        # print self.observation
        self.msg_to_send.data = torque_input
        self.torque_controller.publish(self.msg_to_send)
        # pdb.set_trace()
        self.obs_trigger.publish(self.trigger_msg)
        #  time.sleep(2)
        # print ("self.obs after call back")
        # print (self.observation)
        return self.observation, 0, False, {"info" : "this environment can not \
                                            provide intrinsic reward"}
    #random initlize for each trajectory
    def set_joint_position(self, joint_angle = [0,0,0,0]):
        pass
    
class MockGymEnvWithStates():
    def __init__(self):
        self.observation = None
        self.state = Int16MultiArray()
        #return a 6 channel rgb array
        def multi_cam_cb(data):
            #pdb.set_trace()
            height = data.cam1.height
            width = data.cam1.width
            cam1_data = np.fromstring(data.cam1.data, dtype = np.uint8)
            cam2_data = np.fromstring(data.cam2.data, dtype = np.uint8)
            cam1_data = np.reshape(cam1_data, [height ,width, 3])
            cam2_data = np.reshape(cam2_data, [height ,width, 3])
            self.observation = np.concatenate((cam1_data, cam2_data), axis = 2)
        def state_cb(data):
            self.state = data
        rospy.init_node('mock_gym_env', anonymous = True)
        rospy.Subscriber('env_obs', multi_cam, multi_cam_cb)
        self.torque_controller = rospy.Publisher('servo', Int16MultiArray, queue_size = 10)
        self.obs_trigger = rospy.Publisher('camera_trigger', Empty, queue_size = 10)
        rospy.Subscriber('armstate', Int16MultiArray, state_cb)
        self.msg_to_send = Int16MultiArray()
        self.msg_to_send.data = [0,0,0,0]
        self.trigger_msg = Empty()
        #test
        self.obs_trigger.publish(self.trigger_msg)
        self.torque_controller.publish(self.msg_to_send)
        time.sleep(2)
        self.obs_trigger.publish(self.trigger_msg)
        time.sleep(2)
        # import pdb; pdb.set_trace()
    def step(self, torque_input):
        assert(-1024<torque_input[0] and torque_input[0]<1024)
        assert(-1024<torque_input[1] and torque_input[1]<1024)
        assert(-1024<torque_input[2] and torque_input[2]<1024)
        assert(-1024<torque_input[3] and torque_input[3]<1024)
        # print ("self.obs before call back")
        # print self.observation
        self.msg_to_send.data = torque_input
        self.torque_controller.publish(self.msg_to_send)
        self.obs_trigger.publish(self.trigger_msg)
        #import pdb; pdb.set_trace()
        return {"images":self.observation, "states": self.state}, 0, False, {"info" : "this environment can not \
                                            provide intrinsic reward"}
    #random initlize for each trajectory
    def set_joint_position(self, joint_angle = [0,0,0,0]):
        pass
    