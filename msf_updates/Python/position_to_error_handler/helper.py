import numpy as np
from transformation_functions import rot_to_quat, quaternion_to_matrix

rotation=np.array([[-0.9641744, 0.26524756, -0.00338395],
          [-0.26526245, -0.96416224, 0.00519704],
          [-0.00188418, 0.00590849, 0.99998077]])
print(rotation)
quat=rot_to_quat(rotation)
print(quat)

print("testing functions")
rot=rotation
print(rot)
print(rot_to_quat(rot))
print(quaternion_to_matrix(rot_to_quat(rot)))
