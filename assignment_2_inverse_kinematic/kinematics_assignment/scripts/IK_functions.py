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

def get_dh_matrix(alpha, a, d, theta):
    A = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return A

def get_transform_matrices(joint_positions):
    # DH parameters
    L = 0.4
    M = 0.39
    alpha = [np.pi, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d = [0, 0, L, 0, M, 0, 0]
    a = [0, 0, 0, 0, 0, 0, 0]
    theta = [joint_positions[0], joint_positions[1], joint_positions[2], 
             joint_positions[3], joint_positions[4], joint_positions[5], 
             joint_positions[6]]

    # Base frame
    T0 = np.eye(4)
    T0[2, 3] = 0.311 # z-axis offset

    # End-effector frame
    T7 = np.eye(4)
    T7[2, 3] = 0.078 # z-axis offset

    # Transformation matrices from base to i-th joint
    T = []
    T.append(T0)
    for i in range(7):
        A = get_dh_matrix(alpha[i], a[i], d[i], theta[i])
        T.append(T[i] @ A)
    T.append(T[7] @ T7)
    
    return T

def jacobian(joint_positions, transform_matrices):
    T = transform_matrices
    
    # Jacobian
    J_v = []
    J_w = []
    z = [np.array([0, 0, 1])]
    p = [np.array([0, 0, 0])]

    for i in range(7):
        z.append(T[i][0:3, 2])
        p.append(T[i][0:3, 3])

    for i in range(7):
        J_v.append(np.cross(z[i], (p[6] - p[i])))
        J_w.append(z[i])

    J = np.vstack((np.array(J_v).T, np.array(J_w).T))
    return J

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = np.array(joint_positions)

    """
    point = (x, y, z), the desired position of the end-effector.
    R = 3x3 rotation matrix, the desired orientation of the end-effector.
    joint_positions = (q1, q2, q3, q4, q5, q6, q7) the current joint positions.
    """

    tol = 0.001
    max_iter = 1000

    iter_count = 0
    pos_error_norm = np.inf

    # Inverse Kinematics Loop
    while pos_error_norm > tol and iter_count < max_iter:
        T = get_transform_matrices(q)
        J = jacobian(q, T)

        pos_current = T[-1][0:3, 3]
        pos_final = np.array([x, y, z])

        pos_error = pos_final - pos_current
        pos_error_norm = np.linalg.norm(pos_error)

        R_error = R @ T[-1][0:3, 0:3].T
        rot_error = 0.5 * np.array([R_error[2, 1] - R_error[1, 2], 
                                    R_error[0, 2] - R_error[2, 0], 
                                    R_error[1, 0] - R_error[0, 1]])   
        rot_error_norm = np.linalg.norm(rot_error)

        if pos_error_norm < tol and rot_error_norm < tol:
            break

        J_inv = np.linalg.pinv(J)

        ang_error = J_inv @ np.hstack((pos_error, rot_error))
        q = q + ang_error * 0.1

        iter_count += 1
        print(f"Iteration {iter_count}, Position error norm: {pos_error_norm}")
    
    if iter_count == max_iter:
        print("Warning: IK did not converge within the maximum iterations.")

    return q
