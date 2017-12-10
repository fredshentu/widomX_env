import scipy.misc
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16MultiArray
import time
from mock_gym_env import MockGymEnvWithStates, MockGymEnvWithStates_linear_interpolation
import random
import csv
import time
import numpy
import pickle


env = MockGymEnvWithStates_linear_interpolation()
EPOCH_LENGTH = 600
def sample(index):
    cam_HD_pub_data = Int16MultiArray()
    
    fileOpen = open('log.txt', 'a')
    INIT_POS = np.zeros(4)
    INIT_POS[0] = np.random.randint(0,4095)
    INIT_POS[1] = np.random.randint(1070, 3000)
    INIT_POS[2] = np.random.randint(2600, 3130)
    INIT_POS[3] = np.random.randint(1400, 2500)
    print("set env to the initial position")
    image, state = env.set_joint_position(INIT_POS)
    print("arm in initial position")
    
    # if (index % 10 == 0):

    #     print("cooling down the system....")
    #     for i in range(3000):
    #         if i%100==0:
    #             print(i)
    #         env.step([0,0,0,0])
    #         time.sleep(0.1)
    
    state_trajectory = []
    image_trajectory = []
    action_trajectory = []
    
    
    start_test = time.time()
    time.sleep(0.08)
    for i in range(EPOCH_LENGTH):
        action = [np.random.randint(-1023, 1023), np.random.randint(-200, 200),\
                    np.random.randint(-200, 200), np.random.randint(-1023, 1023)]
        action = [0,0,0,0]
        now = time.time()
        image, state = env.step(action)
        #save image and state
        image_trajectory.append(image)
        state_trajectory.append(state)
        action_trajectory.append(action)
        elapsed = time.time()-now
        print("{}sec elapsed".format(elapsed))
        if elapsed < 0.1:
            time.sleep(0.1-elapsed)
        else:
           
            fileOpen.write("WARNING: episode {} timestep{} timeout, this timestep takes {} sec \n".format(index, i+1, elapsed))
    
        
        
    
    image, state = env.reset()
    print("takes {} secs to excute this action_list with {} actions".format(time.time()-start_test,\
                                                        EPOCH_LENGTH))
    
    image_trajectory.append(image)
    state_trajectory.append(state)
    
    
    fileOpen.close()
    save_dict={"images":image_trajectory, "states":state_trajectory, 'init_pos': INIT_POS,\
                'action_list':action_trajectory}
                    
    FILE_NAME = "sample{}.pickle".format(index)
    with open(FILE_NAME, 'wb') as handle:
    	pickle.dump(save_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    print("traj{} saved".format(index))
    print("cooling down the system")
    for i in range(400):
        env.step([0,0,0,0])
        time.sleep(0.1)
    
if __name__ == '__main__':
    for i in range(477,600):
        sample(i)
