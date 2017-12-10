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
import vis_utils
import time
import numpy
import pickle
# from rllab.misc import logger

# from railrl.data_management.simple_replay_pool import SimpleReplayPool
SLEEP_TIME = 0.4
print("time interval is {}".format(SLEEP_TIME))
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
    
def plot_obs(obs, anim):
    cam1_data =scipy.misc.imresize(obs[:,:,:3],(256,256))
    cam2_data = scipy.misc.imresize(obs[:,:,3:], (256,256))
    anim._display([cam1_data, cam2_data])
    
env = MockGymEnvWithStates()
time.sleep(5)
anim = vis_utils.MyAnimationMulti(None, numPlots=2)


def difference_clip_random_action(old_action):
    a = random.randint(-800,800)
    b = random.randint(-220,220)
    c = random.randint(-220,220)
    d = random.randint(-800,800)
    action = np.array([a,b,c,d])
    clipped_action = np.clip(action,[old_action[0]-100, \
                    old_action[1] - 20, old_action[2] - 10, old_action[3]-100],\
                    [old_action[0]+100, \
                    old_action[1] + 10, old_action[2] + 10, old_action[3]+100])
    clipped_action = np.clip(clipped_action, [-800,-220,-220,-800], [800,220,220,800])

    return clipped_action

def linear_interpolation_random_action(old_action, bounded_state, tau = 0.2, mx28_tau = 0.4):
    a = random.randint(-1023,1023)
    b = random.randint(-350,350)
    c = random.randint(-350,350)
    d = random.randint(-1023,1023)
    action = np.array([old_action[0]*(1-mx28_tau) + mx28_tau*a,\
            old_action[1]*(1-tau) + tau*b,\
            old_action[2]*(1-tau) + tau*c,\
            old_action[3]*(1-mx28_tau) + mx28_tau*d], np.int)
    clipped_action = np.clip(action, [-1023,-350,-350,-800], [1023,350,350,800])
    last_state_position = bounded_state[1]
    #if arm close to its limit, output torque set to zero
    m2_pos = last_state_position[5]
    m3_pos = last_state_position[6]
    if m2_pos > 2945 or m2_pos < 1130:
        clipped_action[1] = 0
    if m3_pos > 3770 or m3_pos < 1044:
        clipped_action[2] = 0
        
    
    return clipped_action

def step_twice(action):
    now = time.time()
    obs1, reward, done, _ = env.step(action)
    image1 = obs1["images"]
    state1 = obs1["states"].data
    elapsed = time.time() - now
    time.sleep(SLEEP_TIME-elapsed) #20Hz
    
    
    now = time.time()
    obs2, reward, done, _ = env.step(action)
    image2 = obs2["images"]
    state2 = obs2["states"].data
    bounded_image = np.concatenate([image1, image2], axis = 2)
    bounded_state = np.array([state1,state2])
        
    return bounded_image, bounded_state, now
    
old_action = [0,0,0,0]
TRAJ_LENGTH = 30
NUM_EPOCH = 1

with open("10hz_2obs/epoch_counter.pickle") as handle:
    data = pickle.load(handle)
epoch_counter = data["index"]


for epoch in range(epoch_counter + 1, epoch_counter + NUM_EPOCH + 1):

    image_list = []
    state_list = []
    action_list = []
    #reset (zero torque)
    bounded_image, bounded_state, now = step_twice([0,0,0,0])
    
    for i in range(TRAJ_LENGTH):
        # a = random.randint(-800,800)
        # b = random.randint(-220,220)
        # c = random.randint(-220,220)
        # d = random.randint(-800,800)
        # a = 0;b=0;c=0;d=0
        image_list.append(bounded_image)
        state_list.append(bounded_state)
        
        elapsed = time.time()-now
        time.sleep(SLEEP_TIME-elapsed)
        
        
        # old_action = linear_interpolation_random_action(old_action, bounded_state)
        
        bounded_image, bounded_state, now = step_twice(old_action)
        action_list.append(old_action)
        # writer1.writerows([[a,b,c,d]])
        # print(bounded_state)
        # print(old_action)
        print(bounded_state)

        
        
    env.step([0,0,0,0])
    image_list.append(bounded_image)
    state_list.append(bounded_state)
    action_list.append([0,0,0,0])
    #dump data
    save_dict={"images":image_list, "states":state_list, "traj_length":TRAJ_LENGTH, \
                "action_list":action_list,
                "READ_ME": "length of list should be traj_length + 1"}
                
#     FILE_NAME = "10hz_2obs/epoch{}.pickle".format(epoch)
#     with open(FILE_NAME, 'wb') as handle:
#     	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)



# save_dict = {"index":epoch_counter + NUM_EPOCH}
# with open("10hz_2obs/epoch_counter.pickle", "wb") as handle:
#     pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)