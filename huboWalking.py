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


class huboWalking(object):

    def __init__(self):


        self._defineGlobals()

        arm_1 = .211
        arm_2 = .182
        arm_3 = .164
        arm_4 = .121
        arm_e = .100

        leg_t = .187
        leg_1 = .088
        leg_2 = .167
        leg_3 = .30003
        leg_4 = .30038
        leg_5 = .095

        self.deltaTheta = .1
        self.numberOfRuns = 5
        self.acceptableError = .1

        self._defineGlobals()


        self.armLength = np.array([arm_e , arm_1 , arm_2 , arm_3 , arm_4])
        self.legLength = np.array([leg_t , leg_1 , leg_2 , leg_3 , leg_4 , leg_5])


        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)


        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()
        [status, framesize] = s.get(state, wait=False, last=False)


        self.rightArmCoordinateNames = np.array(['RSP','RSR' , 'RSY' , 'REB' , 'RWR' , 'RWP'])
        self.leftArmCoordinateNames = np.array(['LSP' , 'LSR' , 'LSY' , 'LEB' , 'LWR' , 'LWP'])
        self.rightLegCoordinateNames = np.array(['RHY' , 'RHR' , 'RHP' , 'RKN' , 'RAP' , 'RAR'])
        self.leftLegCoordinateNames = np.array(['LHY' , 'LHR' , 'LHP' , 'LKN' , 'LAP' , 'LAR'])

        # theta , d , a , alpha
        #
        self.rightGoal = np.array([
            [.125/2. , 0.5691 , .125/2.],
            [.125 , 0.5691 , 0.],
        ])

        #
        self.leftGoal = np.array([
            [-.125/2. , 0.5691 , .125/2.],
            [-.125 , 0.5691 , 0.],
        ])


        # self.goal = np.array([
        #     [.0 , 0.0625 , .125],
        #     [.0 , 0.125 , 0.],
        # ])


        self._run(state , ref , s , r)


    def _run(self , state , ref , s , r):


        standing = self._returnDOFValues(s , state )
        bendTheta1 , bendTheta2 = ik2DOF(.5 , 0. , self.legLength[3] , self.legLength[4])
        bendTheta3 = 0. - bendTheta1 - bendTheta2


        self._bendKnees(bendTheta1 , bendTheta2 , bendTheta3 , ref , r , s , state)


        time.sleep(10)


        for i in range(6):

            print "interrupt"
            time.sleep(2)

            if i%2 == 0:
                print 'right'
                current = self._returnDOFValues(s , state)
                left , right = self._legDH(current)
                error = dhMatrixCalculation(right)
                error = np.dot(error[0:3,0:3],error[0:3,3])

                print error

                error = np.sqrt((error[0] - self.rightGoal[0,0])**2 + (error[1] - self.rightGoal[0,1])**2 + (error[2] - self.rightGoal[0,2])**2)
                prevTime = time.time()
                print error
                while error > self.acceptableError:
                    # print error
                    new = inverseKinematics(self.deltaTheta,right,error,self.rightGoal[0,])
                    self._rightLegMovement(current , new , ref , r)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues(s , state)
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(right)
                    error = np.dot(error[0:3, 0:3], error[0:3, 3])
                    print error
                    error = np.sqrt((error[0] - self.rightGoal[0,0])**2 + (error[1] - self.rightGoal[0,1])**2 + (error[2] - self.rightGoal[0,2])**2)



                print "coming down"

                current = self._returnDOFValues(s , state)
                left , right = self._legDH(current)
                error = dhMatrixCalculation(right)
                error = np.dot(error[0:3,0:3],error[0:3,3])

                error = np.sqrt((error[0] - self.rightGoal[1,0])**2 + (error[1] - self.rightGoal[1,1])**2 + (error[2] - self.rightGoal[1,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,right,error,self.rightGoal[1,])
                    self._rightLegMovement(current , new , ref , r)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues(s , state)
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(right)
                    error = np.dot(error[0:3, 0:3], error[0:3, 3])
                    error = np.sqrt((error[0] - self.rightGoal[1,0])**2 + (error[1] - self.rightGoal[1,1])**2 + (error[2] - self.rightGoal[1,2])**2)


            else:
                print 'left'
                current = self._returnDOFValues(s , state)
                left , right = self._legDH(current)
                error = dhMatrixCalculation(left)
                error = np.dot(error[0:3,0:3],error[0:3,3])
                print 'left error ' , error
                error = np.sqrt((error[0] - self.leftGoal[0,0])**2 + (error[1] - self.leftGoal[0,1])**2 + (error[2] - self.leftGoal[0,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,left,error,self.leftGoal[0,])
                    self._leftLegMovement(current , new , ref , r)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues(s , state)
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(left)
                    error = np.dot(error[0:3, 0:3], error[0:3, 3])
                    error = np.sqrt((error[0] - self.leftGoal[0,0])**2 + (error[1] - self.leftGoal[0,1])**2 + (error[2] - self.leftGoal[0,2])**2)

                current = self._returnDOFValues(s , state)
                left , right = self._legDH(current)
                error = dhMatrixCalculation(left)
                error = np.dot(error[0:3,0:3],error[0:3,3])
                error = np.sqrt((error[0] - self.leftGoal[1,0])**2 + (error[1] - self.leftGoal[1,1])**2 + (error[2] - self.leftGoal[1,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,left,error,self.leftGoal[1,])
                    self._leftLegMovement(current , new , ref , r)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues(s , state)
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(left)
                    error = np.dot(error[0:3, 0:3], error[0:3, 3])
                    error = np.sqrt((error[0] - self.leftGoal[1,0])**2 + (error[1] - self.leftGoal[1,1])**2 + (error[2] - self.leftGoal[1,2])**2)


    def _rightLegMovement(self,current , val,ref,r):

        # self.rightLegCoordinateNames = np.array(['RHY', 'RHR', 'RHP', 'RKN', 'RAP', 'RAR'])
        ref.ref[ha.RHY] = current['RHY'] + val[0]
        ref.ref[ha.RHR] = current['RHR'] + val[1]
        ref.ref[ha.RHP] = current['RHP'] + val[2]
        ref.ref[ha.RKN] = current['RKN'] + val[3]
        ref.ref[ha.RAP] = current['RAP'] + val[4]
        ref.ref[ha.RAR] = current['RAR'] + val[5]

        r.put(ref)

    def _leftLegMovement(self, current , val , ref , r):

        # self.rightLegCoordinateNames = np.array(['RHY', 'RHR', 'RHP', 'RKN', 'RAP', 'RAR'])
        ref.ref[ha.LHY] = current['LHY'] + val[0]
        ref.ref[ha.LHR] = current['LHR'] + val[1]
        ref.ref[ha.LHP] = current['LHP'] + val[2]
        ref.ref[ha.LKN] = current['LKN'] + val[3]
        ref.ref[ha.LAP] = current['LAP'] + val[4]
        ref.ref[ha.LAR] = current['LAR'] + val[5]

        r.put(ref)

    def _bendKnees(self,theta1 , theta2 , theta3 , ref , r , s , state , increment = 5):
        current = self._returnDOFValues(s , state )

        ankle = np.linspace(current['RAP'] , theta1 , increment)
        knee = np.linspace(current['RKN'] , theta2 , increment)
        hip = np.linspace(current['RHP'] , theta3 , increment)

        for i in range(increment):
            ref.ref[ha.RAP] = ankle[i]
            ref.ref[ha.RKN] = knee[i]
            ref.ref[ha.RHP] = hip[i]
            ref.ref[ha.LAP] = ankle[i]
            ref.ref[ha.LKN] = knee[i]
            ref.ref[ha.LHP] = hip[i]

            r.put(ref)

            time.sleep(.1)


    def _returnDOFValues(self , s , state):
        [statuss, framesizes] = s.get(state, wait=False, last=False)

        returnThis = {}

        returnThis.update({'RSP': state.joint[ha.RSP].pos})
        returnThis.update({'RSR': state.joint[ha.RSR].pos})
        returnThis.update({'RSY': state.joint[ha.RSY].pos})
        returnThis.update({'REB': state.joint[ha.REB].pos})
        returnThis.update({'RWY': state.joint[ha.RWY].pos})
        returnThis.update({'RWP': state.joint[ha.RWP].pos})
        returnThis.update({'LSP': state.joint[ha.LSP].pos})
        returnThis.update({'LSR': state.joint[ha.LSR].pos})
        returnThis.update({'LSY': state.joint[ha.LSY].pos})
        returnThis.update({'LEB': state.joint[ha.LEB].pos})
        returnThis.update({'LWY': state.joint[ha.LWY].pos})
        returnThis.update({'LWP': state.joint[ha.LWP].pos})
        returnThis.update({'WST': state.joint[ha.WST].pos})
        returnThis.update({'RHY': state.joint[ha.RHY].pos})
        returnThis.update({'RHR': state.joint[ha.RHR].pos})
        returnThis.update({'RHP': state.joint[ha.RHP].pos})
        returnThis.update({'RKN': state.joint[ha.RKN].pos})
        returnThis.update({'RAP': state.joint[ha.RAP].pos})
        returnThis.update({'RAR': state.joint[ha.RAR].pos})
        returnThis.update({'LHY': state.joint[ha.LHY].pos})
        returnThis.update({'LHR': state.joint[ha.LHR].pos})
        returnThis.update({'LHP': state.joint[ha.LHP].pos})
        returnThis.update({'LKN': state.joint[ha.LKN].pos})
        returnThis.update({'LAP': state.joint[ha.LAP].pos})
        returnThis.update({'LAR': state.joint[ha.LAR].pos})

        return returnThis



    def _armDH(self,val):

        left = np.array([[val[self.leftArmCoordinateNames[0]] + np.pi/2. , 0. , 0. , np.pi/2.],
                              [val[self.leftArmCoordinateNames[1]] - np.pi/2. , 0. , 0. , np.pi/2.],
                               [val[self.leftArmCoordinateNames[2]] + np.pi/2. , -self.armLength[2] , 0. , -np.pi/2.],
                               [val[self.leftArmCoordinateNames[3]] , 0. , 0. , np.pi/2.],
                               [val[self.leftArmCoordinateNames[4]] , -self.armLength[3], 0. , -np.pi/2.],
                               [val[self.leftArmCoordinateNames[5]] + np.pi/2 , 0. , self.armLength[4] , 0.]
                              ])

        right = np.array([[val[self.rightArmCoordinateNames[0]] + np.pi / 2., 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[1]] - np.pi / 2., 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[2]]+ np.pi / 2., -self.armLength[2], 0., -np.pi / 2.],
                     [val[self.rightArmCoordinateNames[3]], 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[4]], -self.armLength[3], 0., -np.pi / 2.],
                     [val[self.rightArmCoordinateNames[5]] + np.pi / 2, 0., self.armLength[4], 0.]
                     ])

        return left , right

    def _legDH(self,val):
        left = np.array([[val[self.leftLegCoordinateNames[0]] , 0. , 0. , np.pi / 2.],
                              [val[self.leftLegCoordinateNames[1]] - np.pi/2. , 0. , 0. , -np.pi/2.],
                              [val[self.leftLegCoordinateNames[2]] , 0. , self.legLength[3] , 0.],
                              [val[self.leftLegCoordinateNames[3]] , 0. , self.legLength[4] , 0.],
                              [val[self.leftLegCoordinateNames[4]] , 0. , 0. , np.pi/2.],
                              [val[self.leftLegCoordinateNames[5]] , 0. , self.legLength[5] , 0.]
                                ])

        right = np.array([[val[self.rightLegCoordinateNames[0]], 0., 0., np.pi / 2.],
                         [val[self.rightLegCoordinateNames[1]] - np.pi / 2., 0., 0., -np.pi / 2.],
                         [val[self.rightLegCoordinateNames[2]], 0., self.legLength[3], 0.],
                         [val[self.rightLegCoordinateNames[3]], 0., self.legLength[4], 0.],
                         [val[self.rightLegCoordinateNames[4]], 0., 0., np.pi / 2.],
                         [val[self.rightLegCoordinateNames[5]], 0., self.legLength[5], 0.]
                         ])

        return left , right

    def _defineGlobals(self):
        global thetaA1
        global thetaA2
        global thetaA3
        global thetaA4
        global thetaA5
        global thetaA6

        global thetaL1
        global thetaL2
        global thetaL3
        global thetaL4
        global thetaL5
        global thetaL6

        global leg_1
        global leg_2
        global leg_3
        global leg_4
        global leg_5
        global leg_t

        global arm_1
        global arm_2
        global arm_3
        global arm_4
        global arm_5
        global arm_e



if __name__=="__main__":

    huboWalking()


































