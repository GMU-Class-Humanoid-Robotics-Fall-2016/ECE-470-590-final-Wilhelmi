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
from moveArm import *


class huboWalking(object):

    def __init__(self):




        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)


        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()
        [status, framesize] = s.get(state, wait=False, last=False)

        # self._initialStep(state,ref,s,r)
        #
        #
        # for x in range(4):
        #     print x
        #     self._followOnSteps(state,ref,s,r)
        #
        #
        # self._finalSteps(state,ref,s,r)
        #
        # self._standUp(state,ref,s,r)

        rTheta = np.zeros((6, 1))
        rGoal = np.array([[283.64], [125.5], [65.0]])

        self._getIK(rTheta , rGoal , ref , r)

    def _getIK(self,theta, G, ref, r):
        dtheta = 0.01
        de = 15
        e = getFK(theta)
        tempTheta = np.copy(theta)
        met = getMet(e, G)
        tempMet = met
        while (met > 5):
            jac = getJ(tempTheta, dtheta)
            jacInv = np.linalg.pinv(jac)
            DE = getNext(e, G, de, tempMet)
            Dtheta = np.dot(jacInv, DE)
            tempTheta = tempTheta +  Dtheta
            e = getFK(tempTheta)
            met = getMet(e, G)



            ref.ref[ha.RSP] = tempTheta[0]
            ref.ref[ha.RSR] = tempTheta[1]
            ref.ref[ha.RSY] = tempTheta[2]
            ref.ref[ha.REB] = tempTheta[3]
            ref.ref[ha.RWY] = tempTheta[4]
            ref.ref[ha.RWR] = tempTheta[5]

        r.put(ref)




    def _standUp(self,state,ref,s,r):

        prevTime = state.time

        ref.ref[ha.RHP] = 0.
        ref.ref[ha.RKN] = 0.
        ref.ref[ha.RAP] = 0.
        ref.ref[ha.LHP] = 0.
        ref.ref[ha.LKN] = 0.
        ref.ref[ha.LAP] = 0.

        r.put(ref)

        prevTime = self._simSleep(.1,state,s,prevTime)



    def _finalSteps(self,state,ref,s,r):

        prevTime = state.time

        moveThis = .01 * np.ones(shape=[28,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHR] -= moveThis[i,0]
            ref.ref[ha.RAR] += moveThis[i,0]
            ref.ref[ha.LHR] -= moveThis[i,0]
            ref.ref[ha.LAR] += moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        # moveThis = .01 * np.ones(shape=[12,1])
        # for i in range(np.shape(moveThis)[0]):
        #     ref.ref[ha.RHR] -= moveThis[i,0]
        #     ref.ref[ha.RAR] += moveThis[i,0]
        #     ref.ref[ha.LHR] -= moveThis[i,0]
        #     ref.ref[ha.LAR] += moveThis[i,0]
        #     r.put(ref)
        #
        #     prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[12,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.LHP] += moveThis[i,0]
            ref.ref[ha.LKN] -= 2 * moveThis[i,0]
            ref.ref[ha.LAP] += moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[12,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.LHP] += moveThis[i,0]
            ref.ref[ha.LAP] -= moveThis[i,0]
            ref.ref[ha.RHP] -= moveThis[i,0]
            ref.ref[ha.RAP] += moveThis[i,0]
            ref.ref[ha.LHP] -= moveThis[i,0]
            ref.ref[ha.LKN] += 2 * moveThis[i,0]
            ref.ref[ha.LAP] -= moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)


        moveThis = .01 * np.ones(shape=[14,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHR] = moveThis[i,0]
            ref.ref[ha.RAR] = -moveThis[i,0]
            ref.ref[ha.LHR] = moveThis[i,0]
            ref.ref[ha.LAR] = -moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)





    def _followOnSteps(self,state,ref,s,r):

        prevTime = state.time

        moveThis = .01 * np.ones(shape=[28,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHR] -= moveThis[i,0]
            ref.ref[ha.RAR] += moveThis[i,0]
            ref.ref[ha.LHR] -= moveThis[i,0]
            ref.ref[ha.LAR] += moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[24,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.LHP] += moveThis[i,0]
            ref.ref[ha.LKN] -= 2 * moveThis[i,0]
            ref.ref[ha.LAP] += moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[24,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.LHP] += moveThis[i,0]
            ref.ref[ha.LAP] -= moveThis[i,0]
            ref.ref[ha.RHP] -= moveThis[i,0]
            ref.ref[ha.RAP] += moveThis[i,0]
            ref.ref[ha.LHP] -= moveThis[i,0]
            ref.ref[ha.LKN] += 2 * moveThis[i,0]
            ref.ref[ha.LAP] -= moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[28,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHR] += moveThis[i,0]
            ref.ref[ha.RAR] -= moveThis[i,0]
            ref.ref[ha.LHR] += moveThis[i,0]
            ref.ref[ha.LAR] -= moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)

        moveThis = .01 * np.ones(shape=[24,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHP] += moveThis[i,0]
            ref.ref[ha.RKN] -= 2 * moveThis[i,0]
            ref.ref[ha.RAP] += moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)


        moveThis = .01 * np.ones(shape=[24,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.LHP] -= moveThis[i,0]
            ref.ref[ha.LAP] += moveThis[i,0]
            ref.ref[ha.RHP] += moveThis[i,0]
            ref.ref[ha.RAP] -= moveThis[i,0]
            ref.ref[ha.RHP] -= moveThis[i,0]
            ref.ref[ha.RKN] += 2 * moveThis[i,0]
            ref.ref[ha.RAP] -= moveThis[i,0]
            r.put(ref)

            prevTime = self._simSleep(0.1,state,s,prevTime)





    def _initialStep(self,state,ref,s,r):

        prevTime = state.time

        moveThis = np.linspace(0.1,.6,6)
        # prevTime = state.time
        for i in range(np.size(moveThis)):
            ref.ref[ha.RHP] = -moveThis[i]
            ref.ref[ha.RKN] = 2. * moveThis[i]
            ref.ref[ha.RAP] = -moveThis[i]
            ref.ref[ha.LHP] = -moveThis[i]
            ref.ref[ha.LKN] = 2 * moveThis[i]
            ref.ref[ha.LAP] = -moveThis[i]

            r.put(ref)

            prevTime = self._simSleep(.1,state,s,prevTime)

        print 'time to shift'


        #Shift
        moveThis = np.linspace(0.01,.14,14)
        for i in range(np.size(moveThis)):
            ref.ref[ha.RHR] = moveThis[i]
            ref.ref[ha.RAR] = -moveThis[i]
            ref.ref[ha.LHR] = moveThis[i]
            ref.ref[ha.LAR] = -moveThis[i]

            r.put(ref)

            prevTime = self._simSleep(.1,state,s,prevTime)

        print 'raise right leg'

        # Raise Right Leg
        moveThis = .01 * np.ones(shape=[12,1])
        for i in range(np.shape(moveThis)[0]):
            ref.ref[ha.RHP] += moveThis[i,0]
            ref.ref[ha.RKN] -= 2 * moveThis[i,0]
            ref.ref[ha.RAP] += moveThis[i,0]

            r.put(ref)

            prevTime = self._simSleep(.1,state,s,prevTime)

        print 'stepping right'
        # Right Step
        moveThis = .01 * np.ones(shape=[12,1])
        for i in range(np.shape(moveThis)[0]):

            ref.ref[ha.LHP] -= moveThis[i,0]
            ref.ref[ha.LAP] += moveThis[i,0]
            ref.ref[ha.RHP] += moveThis[i,0]
            ref.ref[ha.RAP] -= moveThis[i,0]
            ref.ref[ha.RHP] -= moveThis[i,0]
            ref.ref[ha.RKN] += 2 * moveThis[i,0]
            ref.ref[ha.RAP] -= moveThis[i,0]

            r.put(ref)

            prevTime = self._simSleep(.1 , state , s , prevTime)

        # prevTime = self._simSleep(.1,state,s,prevTime)



        # Shift






    def _simSleep(self,T,state,s,prev):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        while ((state.time - prev) < T):
            [statuss, framesizes] = s.get(state, wait=True, last=False)
        [statuss, framesizes] = s.get(state, wait=True, last=False)

        return state.time





if __name__=="__main__":

    huboWalking()


































