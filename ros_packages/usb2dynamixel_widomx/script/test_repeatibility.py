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



action_list = np.zeros([80,4])

action_list = np.array([[ 0.12575425,  0.2869075 , -0.44743692,  0.14973334],
       [-0.61164276, -0.32495567,  0.25871056,  0.60520842],
       [-0.02495361,  0.37398016,  0.45166722, -0.29958219],
       [-0.51745736,  0.06281329, -0.08626201,  0.01660094],
       [ 0.54676645, -0.51370773, -0.7159517 , -0.62668822],
       [ 0.21064179,  0.76192012, -0.58994354,  0.28568261],
       [-0.13393848, -0.19301237, -0.4473784 ,  0.61127246],
       [-0.36644432, -0.13544063,  0.50867033,  0.74517553],
       [-0.74096266,  0.77633744, -0.46370654, -0.5292459 ],
       [-0.32696052,  0.10268592,  0.40963141,  0.29022339],
       [ 0.59210678, -0.66557696, -0.43565858,  0.3108482 ],
       [ 0.6307105 , -0.19831695,  0.55758019, -0.09791381],
       [-0.65302351,  0.44665855,  0.67052096,  0.31041033],
       [ 0.03604363,  0.41802738,  0.66910943, -0.2960899 ],
       [ 0.76635463, -0.54500316, -0.36504307, -0.78442063],
       [ 0.17974079,  0.73252115,  0.61013754,  0.3110887 ],
       [ 0.66357476,  0.74983223,  0.43024288,  0.32429174],
       [ 0.3058871 , -0.24516299,  0.56182668, -0.79780146],
       [ 0.64782014,  0.34042064,  0.5282182 ,  0.10430877],
       [-0.60489745,  0.08250669,  0.06714925, -0.79251088],
       [-0.19839498,  0.68733472, -0.76812754, -0.30642676],
       [-0.73301624,  0.55804727, -0.23395888,  0.72773531],
       [-0.04132066, -0.08658881,  0.72088622,  0.37972649],
       [ 0.22630979, -0.36899259,  0.08860106, -0.38601333],
       [ 0.71048963,  0.75947884, -0.56091705, -0.05066643],
       [ 0.52140424, -0.6379251 , -0.21127659,  0.7729955 ],
       [-0.06116278, -0.6641366 ,  0.74851609, -0.44843021],
       [-0.06644368, -0.76619369,  0.6080953 , -0.4736453 ],
       [ 0.30967613, -0.34921229,  0.60445024, -0.25109894],
       [ 0.13726523,  0.33745799, -0.53345196,  0.2198852 ],
       [-0.16906205, -0.68768637,  0.57331904, -0.62216169],
       [ 0.10199642, -0.47659603,  0.60670533,  0.52494203],
       [-0.54190992,  0.39690681,  0.34493759,  0.23387179],
       [-0.02357645,  0.42919137, -0.36774343, -0.41385816],
       [ 0.00393775, -0.28513501, -0.17261762, -0.02624651],
       [-0.70586438,  0.3525013 ,  0.36434101,  0.08033256],
       [ 0.7087352 ,  0.4001578 , -0.70565624,  0.063144  ],
       [ 0.51090496,  0.21172139, -0.6628677 , -0.23535476],
       [ 0.09360219,  0.44660809,  0.51493597,  0.7309342 ],
       [ 0.2643058 ,  0.53416732, -0.63604383,  0.58675402],
       [ 0.13463354, -0.004383  ,  0.22946852, -0.60568039],
       [-0.11968427, -0.12677002, -0.17699125,  0.13669366],
       [ 0.51929367,  0.14765235,  0.66247641, -0.59217855],
       [ 0.18927826,  0.46848142,  0.06061333, -0.36859004],
       [ 0.51997582, -0.34258638,  0.3329287 ,  0.46117899],
       [-0.28721902,  0.50584814, -0.15327795, -0.09075156],
       [ 0.06281772, -0.17324515,  0.4483074 , -0.53266277],
       [ 0.41337787, -0.21419735,  0.50533892, -0.02686847],
       [ 0.33406757, -0.38516583,  0.05418969,  0.28225606]])
#INIT_POS = [1614, 1440, 3305, 2209]
# INIT_POS = [2600,1920,3060,1997] (in the middle without arena)
#INIT_POS = [2.75, 3.45, 3.70, 3.46] #without arena

INIT_POS = [4, 1.7, 4.65,3] #with arena 

# """
# smooth the action
# """
# action_list[:,1] = np.convolve(action_list[:,1], [0.1,0.2,0.2,0.2,0.2,0.1], "same")
# action_list[:,2] = np.convolve(action_list[:,2], [0.1,0.2,0.2,0.2,0.2,0.1], "same")



env = MockGymEnv()
def test(index):
    state_trajectory = []
    image_trakectory = []
    action_trajectory = []
    print("set env to the initial position")
    env.lift_arm(2.75)
    # time.sleep(10)
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
        # action[0] = 0
        # action[1] = 0
        # action[2] = 0 #np.random.rand() - 0.5
        # action[3] = np.random.rand() - 0.5
        print("start step")
        now = time.time()
        image, state, sound = env.step(action)
        print("step end")
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
    image, state, sound = env.step([0,0,0,0])
    image_trakectory.append(image)
    state_trajectory.append(state)
     
    save_dict={"images":image_trakectory, "states":state_trajectory, 'init_pos': INIT_POS,\
                'action_list':action_trajectory
    }
                    
    FILE_NAME = "test{}.pickle".format(index)
    with open(FILE_NAME, 'wb') as handle:
    	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    print("traj{} saved".format(index))
    time.sleep(5)
for i in range(10):
    test(i)
# return_dict = env.set_joint_position(INIT_POS)