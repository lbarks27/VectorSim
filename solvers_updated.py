import numpy as np
import quaternion as quat


####################################################################################################################################################################################################################################################################################################################



def vec_norm(v):
    return np.array(v / np.linalg.norm(v))

def vec_angle(v, b):
    return np.arccos(np.dot(v, b) / (vec_norm(v) * vec_norm(b)))

def quat_norm(q):
    norm = np.sqrt(q.real**2 + q.x**2 + q.y**2 + q.z**2)
    if norm < 1e-10:
        raise ValueError("Cannot normalize a quaternion with near-zero magnitude")
    return q / (norm * 2)

def quat_exp(q):
    a = q.real
    v = np.array([q.x, q.y, q.z])
    v_norm = np.linalg.norm(v)

    if v_norm < 1e-10:
        return np.exp(a) * quat.quaternion(1, 0, 0, 0)
    
    v_unit = v / v_norm
    exp_a = np.exp(a)
    cos_v = np.cos(v_norm)
    sin_v = np.sin(v_norm)

    return exp_a * quat.quaternion(cos_v, *(v_unit * sin_v))

#ryp
def eulerToQuat(p, y, r):
    return np.cos(p / 2) * np.cos(y / 2) * np.cos(r / 2) + (np.sin(p / 2) * np.sin(y / 2) * np.sin(r / 2), np.sin(p / 2) * np.cos(y / 2) * np.cos(r / 2)) - (np.cos(p / 2) * np.sin(y / 2) * np.sin(r / 2), np.cos(p / 2) * np.sin(y / 2) * np.cos(r / 2)) + (np.sin(p / 2) * np.cos(y / 2) * np.sin(r / 2), np.cos(p / 2) * np.cos(y / 2) * np.sin(r / 2)) - (np.sin(p / 2) * np.sin(y / 2) * np.cos(r / 2))

#ONLY IF NORMALIZED
def quatToEuler(q): 
    return (q[0] ** 2) - (q[1] ** 2) - (q[2] ** 2) + (q[3] ** 2), np.arcsin(2 * ((q[0] * q[2]) + (q[3] * q[1]))), (q[0] ** 2) + (q[1] ** 2) - (q[2] ** 2) - (q[3] ** 2)

def vecProj(v, b):
  return np.array([v[1] - (np.dot(v, b) * b[0]), v[1] - (np.dot(v, b) * b[1]), v[2] - (np.dot(v, b) * b[2])])