#! /usr/bin/env python


"""





"""

import ach
import numpy as np
from inverseKinematics import inverseKinematics
import hubo_ach as ha
import subprocess
from ik2DOF import ik2DOF
import time
from dhMatrix import dhMatrixCalculation
import math


def getJ(theta, dtheta):
	jac = np.zeros((3,6))
	for i in range((np.shape(jac))[0]):
		for j in range((np.shape(jac))[1]):
			tempTheta = np.copy(theta)
			tempTheta[j] = theta[j] + dtheta
			fk = getFK(tempTheta)
			jac[i,j] = (fk[i,0]) / dtheta
	return jac

def getMet(e, G):
	met = math.sqrt(math.pow(e[0] - G[0],2) + math.pow(e[1] - G[1],2) + math.pow(e[2] - G[2],2))
	return met

def getNext(e, G, de, h):
	dx = (G[0] - e[0]) * de / h
	dy = (G[1] - e[1]) * de / h
	dz = (G[2] - e[2]) * de / h
	DE = np.array([[round(dx,3)],[round(dy,3)],[round(dz,3)]])
	return DE



def RotationMatrix_x(theta_x):
	Rx = np.identity(4)
	Rx[1,1] = np.cos(theta_x)
	Rx[1,2] = np.sin(theta_x) * -1.0
	Rx[2,1] = np.sin(theta_x)
	Rx[2,2] = np.cos(theta_x)
	return Rx

def RotationMatrix_y(theta_y):
	Ry = np.identity(4)
	Ry[0,0] = np.cos(theta_y)
	Ry[0,2] = np.sin(theta_y)
	Ry[2,0] = np.sin(theta_y) * -1.0
	Ry[2,2] = np.cos(theta_y)
	return Ry

def RotationMatrix_z(theta_z):
	Rz = np.identity(4)
	Rz[0,0] = np.cos(theta_z)
	Rz[0,1] = np.sin(theta_z) * -1.0
	Rz[1,0] = np.sin(theta_z)
	Rz[1,1] = np.cos(theta_z)
	return Rz

def getFK(theta):
	T1 = np.identity(4)
	T1[1,3] = -94.5
	T2 = np.identity(4)
	T3 = np.identity(4)
	T4 = np.identity(4)
	T4[2,3] = -179.14
	T5 = np.identity(4)
	T5[2,3] = -181.59
	T6 = np.identity(4)

	Q1 = np.dot(RotationMatrix_y(theta[0,0]),T1)
	Q2 = np.dot(RotationMatrix_x(theta[1,0]),T2)
	Q3 = np.dot(RotationMatrix_z(theta[2,0]),T3)
	Q4 = np.dot(RotationMatrix_y(theta[3,0]),T4)
	Q5 = np.dot(RotationMatrix_z(theta[4,0]),T5)
	Q6 = np.dot(RotationMatrix_x(theta[5,0]),T6)

	Q = np.dot(np.dot(np.dot(np.dot(np.dot(Q1,Q2),Q3),Q4),Q5),Q6)

	position = np.array([[round(Q[0,3],3)],[round(Q[1,3],3)],[round(Q[2,3],3)]])

	return position
