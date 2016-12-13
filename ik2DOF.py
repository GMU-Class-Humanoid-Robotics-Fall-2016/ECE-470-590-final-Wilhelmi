#! /usr/bin/env python

"""



"""

import numpy as np

def ik2DOF(x , y , l1 , l2):


    theta2 = np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2*l1 * l2))

    a = y * (l1 + l2 * np.cos(theta2)) - x * (l2*np.sin(theta2))

    b = x * (l1 + l2 * np.cos(theta2)) + y * (l2 * np.sin(theta2))

    theta1 = np.arctan2(a,b)

    return theta1 , theta2