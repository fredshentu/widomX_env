#!/usr/bin/env python
import roslib; roslib.load_manifest('usb2dynamixel_widomx') #this step is necessary in order to use the customized message
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
from mock_gym import MockGymEnv
import random
import csv
import time
import pickle
import sys


WITH_OBJ = False
env = MockGymEnv("/home/fredshentu/kinetic_workspace/sandbox/usb2dynamixel_widomx/script/reset_traj_recorded.pkl")
def sample(index, length = 600):
    
    action_list = np.random.rand(length,4)*1.6 - 0.8
    
    """
    smooth the action
    """
    action_list[:,1] = np.convolve(action_list[:,1], [0.1,0.2,0.2,0.2,0.2,0.1], "same")
    action_list[:,2] = np.convolve(action_list[:,2], [0.1,0.2,0.2,0.2,0.2,0.1], "same")

    
    state_trajectory = []
    image_trajectory = []
    action_trajectory = []
    audio_trajectory = []
    
    #randomly choose an angle to start the trajectory
    env.lift_arm(np.random.rand()*6.28)
    start_test = time.time()
    for action in action_list:
        now = time.time()
        image, state, audio = env.step(action)
        # print("step end")
        #save image and state
        image_trajectory.append(image)
        state_trajectory.append(state)
        action_trajectory.append(action)
        audio_trajectory.append(audio)
        
        elapsed = time.time()-now
        # print("{}sec elapsed".format(elapsed))
        if elapsed < 0.1:
            time.sleep(0.1-elapsed)
        else:
            print("timeout")
    image, state, audio= env.reset()
    image_trajectory.append(image)
    state_trajectory.append(state)
    audio_trajectory.append(audio)
    print("takes {} secs to excute this action_list with {} actions".format(time.time()-start_test,\
                                                        len(action_list)))
    
    save_dict={"image_list":image_trajectory, "state_list":state_trajectory,\
                'action_list':action_trajectory, "audio_list": audio_trajectory
    }
                    
    FILE_NAME = "sample{}.pickle".format(index)
    assert(len(state_trajectory)==(length+1) and len(image_trajectory)==(length+1) and \
            len(action_trajectory)==length and len(audio_trajectory)==(length + 1))
    with open(FILE_NAME, 'wb') as handle:
    	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    print("traj{} saved".format(index))
    time.sleep(1)
#after traj 285 no audio data......
for i in range(498,800):
    sample(i)
    if (i % 10 == 0) and WITH_OBJ:
        print("reset the environment...")
        env.lift_arm(0)
        for i in range(20):
            env.play_trajectory(i*6.28/20.0)
    #rotate arm to a random angle
    env.reset()
    print ("cooling down")
    time.sleep(20)