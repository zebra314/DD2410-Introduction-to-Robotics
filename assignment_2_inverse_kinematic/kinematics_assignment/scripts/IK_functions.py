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

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
