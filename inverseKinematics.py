#! /usr/bin/env python

"""




"""

import numpy as np
from dhMatrix import dhMatrixCalculation


def getJacobian(deltaTheta , dh):
    out = np.zeros([3 , np.shape(dh)[0]])

    for i in range(np.shape(out)[1]):
        newCurrent = dh[0,i] + deltaTheta

        output = dhMatrixCalculation(newCurrent)

        out[:,i] = (np.dot(output[0:3,0:3], output[0:3,3])) / deltaTheta

    return out

def inverseKinematics(deltaTheta , dh , error , goal):

    jacobian = getJacobian(deltaTheta , dh)
    jacobian = np.linalg.pinv(jacobian)
    nStep = nextStep(step , error , goal)

    return np.dot(jacobian,nStep)


def nextStep(stepSize , distance , goal):
    current = dhMatrixCalculation(dh)
    dx = (goal[0] - current[0]) * stepSize / distance
    dy = (goal[1] - current[1]) * stepSize / distance
    dz = (goal[2] - current[2]) * stepSize / distance

    return np.transpose(np.array([dx , dy , dz]))



