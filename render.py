import numpy as np
import quaternion as quat
import matplotlib.pyplot as plt


####################################################################################################################################################################################################################################################################################################################


#render helpers
null_rotation_matrix = np.array([[1, 0, 0], #x
                                  [0, 1, 0], #y
                                  [0, 0, 1]]) #z


####################################################################################################################################################################################################################################################################################################################


#rotation matrix
def rotation_matrix(quaternion):
    return np.array([quat.rotate_vectors(quaternion, null_rotation_matrix[0]), 
                    quat.rotate_vectors(quaternion, null_rotation_matrix[1]), 
                    quat.rotate_vectors(quaternion, null_rotation_matrix[2])])

def thrust_cone(cone_rot, cone_origin, cone_r, cone_l, cone_subdivs):
    pass

def body_cylinder():
    pass