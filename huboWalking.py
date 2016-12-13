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




        arm_1 = .211
        arm_2 = .182
        arm_3 = .164
        arm_4 = .121
        arm_e = .100

        leg_t = .187
        leg_1 = .088
        leg_2 = .167
        leg_3 = .280
        leg_4 = .280
        leg_5 = .095

        self.deltaTheta = .1
        self.numberOfRuns = 5
        self.acceptableError = .025

        self._defineGlobals()

        self.armLength = np.array([arm_1 , arm_2 , arm_3 , arm_4 , arm_e])
        self.legLength = np.array([leg_t , leg_1 , leg_2 , leg_3 , leg_4 , leg_5])

        self.s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        self.r = ach.Channel(ha.HUBO_CHAN_REF_NAME)



        self.state = ha.HUBO_STATE()
        self.ref = ha.HUBO_REF()

        [status, framesize] = self.s.get(self.state, wait=False, last=True)

        self.rightArmCoordinateNames = np.array(['RSP','RSR' , 'RSY' , 'REB' , 'RWR' , 'RWP'])
        self.leftArmCoordinateNames = np.array(['LSP' , 'LSR' , 'LSY' , 'LEB' , 'LWR' , 'LWP'])
        self.rightLegCoordinateNames = np.array(['RHY' , 'RHR' , 'RHP' , 'RKN' , 'RAP' , 'RAR'])
        self.leftLegCoordinateNames = np.array(['LHY' , 'LHR' , 'LHP' , 'LKN' , 'LAP' , 'LAR'])

        # theta , d , a , alpha

        self.goal = np.array([
            [.0625 , 0. , .125],
            [.125 , 0. , 0.],
        ])
        # self.secondGoal = np.array([
        #     [.375 , 0. , .25],
        #     [.50 , 0. , 0.]
        # ])

        self._run()

    def _run(self):

        standing = self._returnDOFValues()
        bendTheta1 , bendTheta2 = ik2DOF(.5 , 0. , self.legLength[3] , self.legLength[4])
        bendTheta3 = 0. - bendTheta1 - bendTheta2

        self._bendKnees(bendTheta1,bendTheta2,bendTheta3)

        time.sleep(10)

        for i in range(6):
            if i%2 == 1:
                print 'right'
                current = self._returnDOFValues()
                left , right = self._legDH(current)
                error = dhMatrixCalculation(right)
                error = np.sqrt((error[0] - self.goal[0,0])**2 + (error[1] - self.goal[0,1])**2 + (error[2] - self.goal[0,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,right,error,self.goal[0,])
                    self._rightLegMovement(new)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues()
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(right)
                    error = np.sqrt((error[0] - self.goal[0,0])**2 + (error[1] - self.goal[0,1])**2 + (error[2] - self.goal[0,2])**2)

                current = self._returnDOFValues()
                left , right = self._legDH(current)
                error = dhMatrixCalculation(right)
                error = np.sqrt((error[0] - self.goal[1,0])**2 + (error[1] - self.goal[1,1])**2 + (error[2] - self.goal[1,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,right,error,self.goal[1,])
                    self._rightLegMovement(new)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues()
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(right)
                    error = np.sqrt((error[0] - self.goal[1,0])**2 + (error[1] - self.goal[1,1])**2 + (error[2] - self.goal[1,2])**2)




            else:
                print 'left'
                current = self._returnDOFValues()
                left , right = self._legDH(current)
                error = dhMatrixCalculation(left)
                error = np.sqrt((error[0] - self.goal[0,0])**2 + (error[1] - self.goal[0,1])**2 + (error[2] - self.goal[0,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,left,error,self.goal[0,])
                    self._leftLegMovement(new)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues()
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(left)
                    error = np.sqrt((error[0] - self.goal[0,0])**2 + (error[1] - self.goal[0,1])**2 + (error[2] - self.goal[0,2])**2)

                current = self._returnDOFValues()
                left , right = self._legDH(current)
                error = dhMatrixCalculation(left)
                error = np.sqrt((error[0] - self.goal[1,0])**2 + (error[1] - self.goal[1,1])**2 + (error[2] - self.goal[1,2])**2)
                prevTime = time.time()
                while error > self.acceptableError:
                    new = inverseKinematics(self.deltaTheta,left,error,self.goal[1,])
                    self._leftLegMovement(new)
                    timeCorrection = time.time() - prevTime
                    if timeCorrection < .1:
                        time.sleep(.1-timeCorrection)
                    prevTime = time.time()
                    current = self._returnDOFValues()
                    left , right = self._legDH(current)
                    error = dhMatrixCalculation(left)
                    error = np.sqrt((error[0] - self.goal[1,0])**2 + (error[1] - self.goal[1,1])**2 + (error[2] - self.goal[1,2])**2)


    def _rightLegMovement(self,val):

        # self.rightLegCoordinateNames = np.array(['RHY', 'RHR', 'RHP', 'RKN', 'RAP', 'RAR'])
        self.ref.ref[ha.RHY] = val[0]
        self.ref.ref[ha.RHR] = val[1]
        self.ref.ref[ha.RHP] = val[2]
        self.ref.ref[ha.RKN] = val[3]
        self.ref.ref[ha.RAP] = val[4]
        self.ref.ref[ha.RAR] = val[5]

        self.r.put(self.ref.ref)

    def _leftLegMovement(self, val):

        # self.rightLegCoordinateNames = np.array(['RHY', 'RHR', 'RHP', 'RKN', 'RAP', 'RAR'])
        self.ref.ref[ha.LHY] = val[0]
        self.ref.ref[ha.LHR] = val[1]
        self.ref.ref[ha.LHP] = val[2]
        self.ref.ref[ha.LKN] = val[3]
        self.ref.ref[ha.LAP] = val[4]
        self.ref.ref[ha.LAR] = val[5]

        self.r.put(self.ref.ref)

    def _bendKnees(self,theta1 , theta2 , theta3 , increment = 5):
        current = self._returnDOFValues()

        ankle = np.linspace(current['RAP'] , theta1 , increment)
        knee = np.linspace(current['RKN'] , theta2 , increment)
        hip = np.linspace(current['RHP'] , theta3 , increment)

        for i in range(5):

            self.ref.ref[ha.RAP] = ankle[i]
            self.ref.ref[ha.RKN] = knee[i]
            self.ref.ref[ha.RHP] = hip[i]
            self.ref.ref[ha.LAP] = ankle[i]
            self.ref.ref[ha.LKN] = knee[i]
            self.ref.ref[ha.LHP] = hip[i]

            self.r.put(self.ref.ref)
            print i, self.ref.ref
            time.sleep(2)


    def _returnDOFValues(self ):
        [statuss, framesizes] = self.s.get(self.state, wait=False, last=False)

        returnThis = {}

        returnThis.update({'RSP': self.state.joint[ha.RSP].pos})
        returnThis.update({'RSR': self.state.joint[ha.RSR].pos})
        returnThis.update({'RSY': self.state.joint[ha.RSY].pos})
        returnThis.update({'REB': self.state.joint[ha.REB].pos})
        returnThis.update({'RWY': self.state.joint[ha.RWY].pos})
        returnThis.update({'RWP': self.state.joint[ha.RWP].pos})
        returnThis.update({'LSP': self.state.joint[ha.LSP].pos})
        returnThis.update({'LSR': self.state.joint[ha.LSR].pos})
        returnThis.update({'LSY': self.state.joint[ha.LSY].pos})
        returnThis.update({'LEB': self.state.joint[ha.LEB].pos})
        returnThis.update({'LWY': self.state.joint[ha.LWY].pos})
        returnThis.update({'LWP': self.state.joint[ha.LWP].pos})
        returnThis.update({'WST': self.state.joint[ha.WST].pos})
        returnThis.update({'RHY': self.state.joint[ha.RHY].pos})
        returnThis.update({'RHR': self.state.joint[ha.RHR].pos})
        returnThis.update({'RHP': self.state.joint[ha.RHP].pos})
        returnThis.update({'RKN': self.state.joint[ha.RKN].pos})
        returnThis.update({'RAP': self.state.joint[ha.RAP].pos})
        returnThis.update({'RAR': self.state.joint[ha.RAR].pos})
        returnThis.update({'LHY': self.state.joint[ha.LHY].pos})
        returnThis.update({'LHR': self.state.joint[ha.LHR].pos})
        returnThis.update({'LHP': self.state.joint[ha.LHP].pos})
        returnThis.update({'LKN': self.state.joint[ha.LKN].pos})
        returnThis.update({'LAP': self.state.joint[ha.LAP].pos})
        returnThis.update({'LAR': self.state.joint[ha.LAR].pos})

        return returnThis



    def _armDH(self,val):

        left = np.array([[val[self.leftArmCoordinateNames[0]] + np.pi/2. , 0. , 0. , np.pi/2.],
                              [val[self.leftArmCoordinateNames[1]] - np.pi/2. , 0. , 0. , np.pi/2.],
                               [val[self.leftArmCoordinateNames[2]] + np.pi/2. , -arm_2 , 0. , -np.pi/2.],
                               [val[self.leftArmCoordinateNames[3]] , 0. , 0. , np.pi/2.],
                               [val[self.leftArmCoordinateNames[4]] , -arm_3 , 0. , -np.pi/2.],
                               [val[self.leftArmCoordinateNames[5]] + np.pi/2 , 0. , arm_4 , 0.]
                              ])

        right = np.array([[val[self.rightArmCoordinateNames[0]] + np.pi / 2., 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[1]] - np.pi / 2., 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[2]]+ np.pi / 2., -arm_2, 0., -np.pi / 2.],
                     [val[self.rightArmCoordinateNames[3]], 0., 0., np.pi / 2.],
                     [val[self.rightArmCoordinateNames[4]], -arm_3, 0., -np.pi / 2.],
                     [val[self.rightArmCoordinateNames[5]] + np.pi / 2, 0., arm_4, 0.]
                     ])

        return left , right

    def _legDH(self,val):
        left = np.array([[val[self.leftLegCoordinateNames[0]] , 0. , 0. , np.pi / 2.],
                              [val[self.leftLegCoordinateNames[1]] - np.pi/2. , 0. , 0. , -np.pi/2.],
                              [val[self.leftLegCoordinateNames[2]] , 0. , leg_3 , 0.],
                              [val[self.leftLegCoordinateNames[3]] , 0. , leg_4 , 0.],
                              [val[self.leftLegCoordinateNames[4]] , 0. , 0. , np.pi/2.],
                              [val[self.leftLegCoordinateNames[5]] , 0. , leg_5 , 0.]
                                ])

        right = np.array([[val[self.rightLegCoordinateNames[0]], 0., 0., np.pi / 2.],
                         [val[self.rightLegCoordinateNames[1]] - np.pi / 2., 0., 0., -np.pi / 2.],
                         [val[self.rightLegCoordinateNames[2]], 0., leg_3, 0.],
                         [val[self.rightLegCoordinateNames[3]], 0., leg_4, 0.],
                         [val[self.rightLegCoordinateNames[4]], 0., 0., np.pi / 2.],
                         [val[self.rightLegCoordinateNames[5]], 0., leg_5, 0.]
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


































