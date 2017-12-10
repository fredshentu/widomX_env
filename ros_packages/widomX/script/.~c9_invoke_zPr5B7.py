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


env = MockGymEnvWithStates_linear_interpolation()
EPOCH_LENGTH = 600
def sample(index):
    if (index %30 == 0):
        print("cooling down the system....")
        time.sleep(200)
    INIT_POS = np.zeros(4)
    INIT_POS[0] = np.random.randint(0,4095)
    INIT_POS[1] = np.random.randint(1070, 3000)
    INIT_POS[2] = np.random.randint(2600, 3130)
    INIT_POS[3] = np.random.randint(1400, 2500)
    state_trajectory = []
    image_trajectory = []
    action_trajectory = []
    print("set env to the initial position")
    image, state = env.set_joint_position(INIT_POS)
    print("arm in initial position")
    
    start_test = time.time()
    for i in range(EPOCH_LENGTH):
        action = [np.random.randint(-1023, 1023), np.random.randint(-250, 250),\
                    np.random.randint(-200, 200), np.random.randint(-1023, 1023)]
        now = time.time()
        image, state = env.step(action)
        #save image and state
        image_trajectory.append(image)
        state_trajectory.append(state)
        action_trajectory.append(action)
        elapsed = time.time()-now
        print("{}sec elapsed".format(elapsed))
        time.sleep(0.1-elapsed)
        
    print("takes {} secs to excute this action_list with {} actions".format(time.time()-start_test,\
                                                        EPOCH_LENGTH))
    image, state = env.reset()
    image_trajectory.append(image)
    state_trajectory.append(state)
     
    save_dict={"images":image_trajectory, "states":state_trajectory, 'init_pos': INIT_POS,\
                'action_list':action_trajectory}
                    
    FILE_NAME = "sample{}.pickle".format(index)
    with open(FILE_NAME, 'wb') as handle:
    	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    print("traj{} saved".format(index))
for i in range(22,100):
    sample(i)
# return_dict = env.set_joint_position(INIT_POS)