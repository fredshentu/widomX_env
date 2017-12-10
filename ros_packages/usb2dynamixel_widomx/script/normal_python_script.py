import numpy as np
from mock_gym import MockGymEnv
import time

env = MockGymEnv()
env.step(np.zeros(4))
print("set a final position")
import pdb; pdb.set_trace()
goal_joint_pos = np.array(env.step_state_only(np.zeros(4)))[[1,4,7,10]]

print("move the arm to the init position")
pdb.set_trace()
init_joint_pos = env.step_state_only(np.zeros(4))

print("go from current pos to goal pos")
env.set_joint_position(goal_joint_pos)
import time
time.sleep(10)
env.step(np.zeros(4))