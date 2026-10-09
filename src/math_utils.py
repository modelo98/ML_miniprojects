import numpy as np

def euclidean_distance(a,b):

    a = np.array(a)
    b = np.array(b)

    vector = b - a

    distance = np.linalg.norm(vector)

    return distance


def normalize_vector(v):

    v = np.array(v)

    norm = np.linalg.norm(v)

    if norm == 0 :
        return np.zeros_like(v)
    
    return v / norm

def scalar_product(a,b):

    a = np.array(a)
    b = np.array(b)

    return np.dot(a,b)

def angle_of_vector(v): 
    return np.arctan2(v[1],v[0]) # te lo devuelve en radianes

def angle_between_vectors(target_angle,robot_theta):
    return target_angle - robot_theta

def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def normalize_angle_deg(angle):
    return (angle + 180) % 360 - 180


def heading_from_angle(theta): # convierte un ángulo en vector dirección. # theta se mete en grados
     theta = np.deg2rad(theta)
     v = np.array([np.cos(theta),np.sin(theta)])
     return v

def rotation_matrix_2d(theta): # crea una matriz que gira vectores. # theta se mete en grados
     theta = np.deg2rad(theta)
     R = np.array([
          [np.cos(theta), -np.sin(theta)],
          [np.sin(theta), np.cos(theta)]
     ])
     return R
     

def rotate_vector(v, theta): # gira un vector usando la matriz de rotación
     R = rotation_matrix_2d(theta)
     return R @ v

def angle_error(vector_to_goal, robot_theta): #theta en grados
    target_angle = np.rad2deg(angle_of_vector(vector_to_goal))
    angle_error = target_angle - robot_theta
    return normalize_angle_deg(angle_error)