#!/usr/bin/env python
import roslib; roslib.load_manifest('widomX') #this step is necessary in order to use the customized message
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
#import the customized message
from widomX.msg import multi_cam
import scipy.misc
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16MultiArray
import time
from mock_gym_env import MockGymEnvWithStates, MockGymEnvWithStates_linear_interpolation
import random
import csv
import vis_utils
import time
import numpy
import pickle


action_list = np.zeros([400,4])

# action_list = [[800,80,60,400],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [40,10,10,100],\
#                 [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [650,20,30,-200],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [650,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
#                 [800,80,60,400],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [40,10,10,100],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
#                 [800,80,60,400],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [40,10,10,100],\
#                 [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [450,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
                
#                  [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [450,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
#                 [800,80,60,400],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [40,10,10,100],\
#                 [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [650,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
#                 [800,80,60,400],\
#                 [800,80,60,400],\
#                 [600,80,80,400],\
#                 [150,-30,30,300],\
#                 [40,10,10,100],\
#                 [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [450,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
                
#                  [0,-10,-10,0],\
#                 [-400,-30,-30,-100],\
#                 [-450,-50,-50,-300],\
#                 [-400,-80,-70,-400],\
#                 [-150,-80,-70,-500],\
#                 [-200,-70,-70,-100],\
#                 [-150,-10,-10,200],\
#                 [-40,-30,-30,-100],\
#                 [60,10,10,-500],\
#                 [-150,30,30,-800],\
#                 [150,30,30,-800],\
#                 [150,80,30,-500],\
#                 [450,20,30,-200],\
#                 [450,30,30,200],\
#                 [350,30,30,800],\
#                 ]

#INIT_POS = [1614, 1440, 3305, 2209]
# INIT_POS = [2600,1920,3060,1997] (in the middle without arena)
INIT_POS = [1400, 2700, 3226, 1421]

env = MockGymEnvWithStates_linear_interpolation()

def test(index):
    state_trajectory = []
    image_trakectory = []
    action_trajectory = []
    print("set env to the initial position")
    image, state = env.set_joint_position(INIT_POS)
    print("arm in initial position")
    # time.sleep(3)
    
    #debug perpose
    # image = return_dict["images"]
    # state = return_dict["states"]
    # plt.imshow(image[:,:,:3])
    # plt.show()
    # plt.imshow(image[:,:,3:])
    # plt.show()
    # print("state after setting init pos")
    # print(state)
    
    start_test = time.time()
    for action in action_list:
        now = time.time()
        image, state = env.step(action)
        #save image and state
        image_trakectory.append(image)
        state_trajectory.append(state)
        action_trajectory.append(action)
        elapsed = time.time()-now
        print("{}sec elapsed".format(elapsed))
        if elapsed < 0.1:
            time.sleep(0.1-elapsed)
        else:
            print("timeout")
    print("takes {} secs to excute this action_list with {} actions".format(time.time()-start_test,\
                                                        len(action_list)))
    image, state = env.reset()
    image_trakectory.append(image)
    state_trajectory.append(state)
     
    save_dict={"images":image_trakectory, "states":state_trajectory, 'init_pos': INIT_POS,\
                'action_list':action_trajectory
    }
                    
    FILE_NAME = "test{}.pickle".format(index)
    with open(FILE_NAME, 'wb') as handle:
    	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    print("traj{} saved".format(index))
for i in range(20):
    test(i)
# return_dict = env.set_joint_position(INIT_POS)