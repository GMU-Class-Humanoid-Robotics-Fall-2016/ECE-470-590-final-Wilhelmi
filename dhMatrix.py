#! /usr/bin/env python

"""

theta , d , a  , alpha
"""



import numpy as np


def dhMatrixCalculation(input):

    mat = np.eye(4,4)

    for i in range(np.shape(input)[0]):
        mat = np.dot( mat , np.array([[np.cos(input[i,0]) , -np.sin(input[i,0])*np.cos(input[i,3]) , np.sin(input[i,0]) * np.sin(input[i,3]) , input[i,2]*np.cos(input[i,0]) ],
                       [np.cos(input[i,0]) , np.cos(input[i,0])*np.cos(input[i,3]) , -np.cos(input[i,0])*np.sin(input[i,3]) , input[i,2]*np.cos(input[i,0])],
                       [0. , np.sin(input[i,3]) , np.cos(input[i,3]) , input[i,1]],
                        [0.,0.,0.,1]])
                     )


    return mat