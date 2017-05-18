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
from mock_gym_env import MockGymEnvWithStates
import random
import csv

count = 0
def save_obs(obs):
    global count 
    count += 1
    cam1_data = obs[:,:,:3]
    cam2_data = obs[:,:,3:]
    cam1_file_name = "cam1_" + str(count)+".jpg"
    cam2_file_name = "cam2_" + str(count)+".jpg"
    scipy.misc.imsave(cam1_file_name, cam1_data)
    scipy.misc.imsave(cam2_file_name, cam2_data)
    print("saved")
env = MockGymEnvWithStates()
f = open("state_log.csv", 'wb')
f1 = open("command_log.csv", 'wb')
writer = csv.writer(f, dialect = 'excel')
writer1 = csv.writer(f1, dialect = 'excel')
writer.writerows([['joint1_v','joint2_v','joint3_v','joint4_v',\
                    'joint1_p','joint2_p','joint3_p','joint4_p' ]])
writer1.writerows([['joint1_torque', 'joint2_torque',\
                    'joint3_torque', 'joint4_torque']])
i=0
while i < 100:
    a = random.randint(-400,400)
    b = random.randint(-70,70)
    c = random.randint(-70,70)
    d = random.randint(-400,400)
    #a = 0;b=0;c=0;d=0
    writer1.writerows([[a,b,c,d]])
    obs, reward, done, _ = env.step([a,b,c,d])
    i+=1
    #print obs["states"].data
    #import pdb; pdb.set_trace()
    #save_obs(obs["images"])
    lst =obs["states"].data
   #f.write("%s\n" % lst)
    writer.writerows([list(lst)])
    time.sleep(0.1) #6Hz
env.step([0,0,0,0])
f.close()
