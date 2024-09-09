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

def transform_matrix(joint_positions):
    # DH parameters
    L = 0.4
    M = 0.39
    alpha = [np.pi, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d = [0, 0, L, 0, M, 0, 0]
    a = [0, 0, 0, 0, 0, 0, 0]
    theta = [joint_positions[0], joint_positions[1], joint_positions[2], 
             joint_positions[3], joint_positions[4], joint_positions[5], 
             joint_positions[6]]

    # Transformation matrices
    T_0_i = []
    for i in range(7):
        A = np.array([[np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],
                    [np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i])],
                    [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                    [0, 0, 0, 1]])
        
        if(i == 0):
            T_0_i.append(A)
        else:
            T_0_i.append(T_0_i[i-1] @ A)
    
    return T_0_i

def jacobian(joint_positions, transform_matrix):
    T_0_i = transform_matrix
    
    # Jacobian
    J_v = []
    J_w = []
    Z = [np.array([0, 0, 1])]
    t = [np.array([0, 0, 0])]

    for i in range(7):
        Z.append(T_0_i[i][0:3, 2])
        t.append(T_0_i[i][0:3, 3])
    
    for i in range(7):
        J_v.append(np.cross(Z[i], (t[6] - t[i])))
        J_w.append(Z[i])

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
        T_0_i = transform_matrix(q)
        J = jacobian(q, T_0_i)

        pos_current = T_0_i[6][0:3, 3]
        pos_final = np.array([x, y, z])

        pos_error = pos_final - pos_current
        pos_error_norm = np.linalg.norm(pos_error)

        R_error = R @ T_0_i[6][0:3, 0:3].T
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
