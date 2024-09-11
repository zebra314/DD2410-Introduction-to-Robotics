#! /usr/bin/env python3
import numpy as np

"""
    # YING PEI LIN
    # yplin@kth.se
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    # cos(beta) * sqrt(x^2 + y^2) = link_1 + link_2 * cos(q2) 

    link_0 = 0.07
    link_1 = 0.3
    link_2 = 0.35

    x = x - link_0

    theta_2 = np.arccos((x**2 + y**2 - link_1**2 - link_2**2)/(2 * link_1 * link_2))

    alpha = np.arctan2(y, x)
    beta = np.arccos((x**2 + y**2 + link_1**2 - link_2**2)/(2 * link_1 * np.sqrt(x**2 + y**2)))

    theta_1 = alpha - beta # or alpha + beta

    q[0] = theta_1
    q[1] = theta_2
    q[2] = z

    return q

def get_transform_matrices(joint_positions):
    # DH parameters
    L = 0.4
    M = 0.39
    alpha = [np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d = [0, 0, L, 0, M, 0, 0]
    a = [0, 0, 0, 0, 0, 0, 0]
    theta = joint_positions

    # Base frame
    T_base = np.eye(4)
    T_base[2, 3] = 0.311 # z-axis offset

    # End-effector frame
    T_end = np.eye(4)
    T_end[2, 3] = 0.078 # z-axis offset

    # Transformation matrices from base to i-th joint
    T = []
    T.append(T_base)    
    for i in range(len(joint_positions)):
        # DH transformation matrix
        A = np.array([[np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],
                      [np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])    
        T.append(T[i] @ A)

    T.append(T[-1] @ T_end)
    return T

def get_jacobian(joint_positions, transform_matrices):
    J_v = []
    J_w = []
    z = []
    p = []

    for T in transform_matrices:
        z.append(T[0:3, 2])
        p.append(T[0:3, 3])        

    for i in range(joint_positions.size):
        J_v.append(np.cross(z[i], (p[-1] - p[i])))
        J_w.append(z[i])

    J = np.vstack((np.array(J_v).T, np.array(J_w).T))
    return J

def kuka_IK(point, R, joint_positions):
    """
    point = (x, y, z), the desired position of the end-effector.
    R = 3x3 rotation matrix, the desired orientation of the end-effector.
    joint_positions = (q1, q2, q3, q4, q5, q6, q7) the current joint positions.
    """

    # Convert the inputs to numpy arrays
    point = np.array(point)
    R = np.array(R)
    joint_positions = np.array(joint_positions)

    # Loop parameters
    tol = 1e-2
    max_iter = 100            
    iter_count = 0

    # Target
    pos_target = np.array(point)
    rot_target = np.array(R)

    # Inverse Kinematics Loop
    while iter_count < max_iter:
        T = get_transform_matrices(joint_positions)
        J = get_jacobian(joint_positions, T)

        # Current
        pos_current = np.array(T[-1][0:3, 3])
        rot_current = np.array(T[-1][0:3, 0:3])

        # Error
        pos_error = pos_current - pos_target
        rot_error = 0.5 * np.array([rot_current[2, 1] - rot_target[1, 2],
                                    rot_current[0, 2] - rot_target[2, 0],
                                    rot_current[1, 0] - rot_target[0, 1]])

        # Get the norm of the error
        pos_error_norm = np.linalg.norm(pos_error)
        rot_error_norm = np.linalg.norm(rot_error)

        # Check if the error is within the tolerance
        # if pos_error_norm < tol and rot_error_norm < tol:
        #     break        

        # Inverse Jacobian
        J_inv = np.linalg.pinv(J)

        # Θ_error = J^-1 * X_error
        theta_error = np.matmul(J_inv, np.hstack((pos_error, rot_error)))
        joint_positions = joint_positions - theta_error

        iter_count += 1
    return joint_positions
