import numpy as np
from transformation_functions import rot_to_quat, quaternion_to_matrix

rotation=np.array([[0.92314157, 0.38227302, 0.04095092],
		[-0.38088244, 0.9238456, -0.03791941],
		[-0.05232789, 0.01940749, 0.99844136]])
print(rotation)
quat=rot_to_quat(rotation)
print(quat)

print("testing functions")
rot=rotation
print(rot)
print(rot_to_quat(rot))
print(quaternion_to_matrix(rot_to_quat(rot)))
