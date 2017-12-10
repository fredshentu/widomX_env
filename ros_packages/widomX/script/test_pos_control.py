import time
from mock_gym_env import MockGym_pos
import numpy as np


env = MockGym_pos()
for i in range(2):
    for i in range(400):
        action = np.random.randint(-40,40,[4])
        print(action)
        state = env.step(action)
        print(state[0])