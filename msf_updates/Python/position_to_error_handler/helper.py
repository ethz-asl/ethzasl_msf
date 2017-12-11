import numpy as np
from transformation_functions import rot_to_quat

rotation=np.array([[0.33638, -0.01749,  0.94156],
          [-0.02078, -0.99972, -0.01114],
          [0.94150, -0.01582, -0.33665]])
print(rotation)
quat=rot_to_quat(rotation)
print(quat)
