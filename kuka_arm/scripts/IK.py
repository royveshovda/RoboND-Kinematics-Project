#!/usr/bin/env python

"""
Class for the Kuka kr210 arm
"""

from mpmath import *
from sympy import *
import tf
from collections import OrderedDict
import numpy as np

class IK(object):
    def __init__(self):

        # DH param variables
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8') #thetas
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8')
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7')
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7')

        # DH params (from derived table)
        self.s = {
            self.a0:      0, self.alpha0:     0, self.d1:  0.75, self.q1: self.q1,
            self.a1:   0.35, self.alpha1: -pi/2, self.d2:     0, self.q2: self.q2 - pi/2,
            self.a2:   1.25, self.alpha2:     0, self.d3:     0, self.q3: self.q3,
            self.a3: -0.054, self.alpha3: -pi/2, self.d4:   1.5, self.q4: self.q4,
            self.a4:      0, self.alpha4:  pi/2, self.d5:     0, self.q5: self.q5,
            self.a5:      0, self.alpha5: -pi/2, self.d6:     0, self.q6: self.q6,
            self.a6:      0, self.alpha6:     0, self.d7: 0.303, self.q7: 0}

        self.T = self._buildTransforms(self.s)

        #Correction for orientation difference between UDRF Gripper location and
        #modified DH parameter conventions
        #rotate 180 dg about z, then -90 dg about y
        self.R_corr = self._rot('Z',180)*self._rot('Y',-90)

        ## Constants used in inverse kinematics
        self.consts = {'gamma': atan2(-self.s[self.a3],self.s[self.d4]),
                       'l3': (self.s[self.a3]**2+ self.s[self.d4]**2)**0.5,
                       'a2': self.s[self.a2],
                       'a3': self.s[self.a3],
                       'd1': self.s[self.d1],
                       'd4': self.s[self.d4],
                       'd7': self.s[self.d7]}

    # Modified DH Transformation matrix
    def _body_fixed_transformation(self, s, i):
        '''
        Take in DH parameters for joints i-1 to i.
        Return transformation matrix from i-1 to i.
        For brevity, theta = theta_i, alpha = alpha_(i-1), a = a_(i-1), d = d_i
        '''

        #set variables using globally-defined symbols and input i
        theta = eval('s[self.q{0}]'.format(i))
        alpha = eval('s[self.alpha{0}]'.format(i-1))
        a = eval('s[self.a{0}]'.format(i-1))
        d = eval('s[self.d{0}]'.format(i))

        transform = Matrix([[            cos(theta),           -sin(theta),            0,              a],
                            [ sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                            [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                            [                     0,                     0,            0,              1]])

        return transform

    def _buildTransforms(self, s):
        T = {}
        for i in range(1,8):
            T[(i-1,i)] = self._body_fixed_transformation(s, i)
            if i>1:
                T[(0,i)] = T[(0,i-1)]*T[(i-1,i)]

        T[(3,6)] = T[(3,4)]*T[(4,5)]*T[(5,6)]

        return T

    def _rot(self, axis, q):
        '''
        Return rotation matrix about specified axis
        given rotation angle q (in radians).
        '''

        q *= pi/180.

        if axis == 'X':
            R = [[ 1,      0,       0],
                 [ 0, cos(q), -sin(q)],
                 [ 0, sin(q),  cos(q)]]
        elif axis == 'Y':
            R = [[  cos(q), 0, sin(q)],
                 [       0, 1,      0],
                 [ -sin(q), 0, cos(q)]]
        elif axis == 'Z':
            R = [[ cos(q), -sin(q), 0],
                 [ sin(q),  cos(q), 0],
                 [      0,       0, 1]]
        else:
            raise RuntimeError('{} is not a valid axis. Options are "X", "Y", or "Z".'.format(axis))

        return Matrix(R)

    def forwardKinematics(self, joint_angles):
        '''
        Return joint, wc, and ee positions given joint_angles
        and precalculated transformation matrices between joints
        '''

        s = dict(zip([self.q1, self.q2, self.q3, self.q4, self.q4, self.q5, self.q6],
              joint_angles))

        joint_positions = OrderedDict()

        for i in range(1,8):
            joint_positions[i] = self.T[(0,i)][:3,3].evalf(subs = s)

        return joint_positions

    def calculateIKError(self, wc_target, ee_target, joint_angles):
        '''
        Calculate error between actual end effector and wrist center
        positions versus target positions.
        '''

        joint_positions = self.forwardKinematics(joint_angles)
        wc_actual = joint_positions[4]
        ee_actual = joint_positions[7]

        wc_error = (wc_target - wc_actual).norm()
        ee_error = (ee_target - ee_actual).norm()

        return [wc_actual, wc_error, ee_actual, ee_error]

    def _createTheta1(self, wc):
        return atan2(wc[1],wc[0])

    def _createTheta23(self, consts, r24, default_orientation = True):
        #return theta 2 and theta 3 using law of cosines with sides
        ## For overview refer to writeup.md

        # gamma angle of declination of link 3
        gamma = consts['gamma']

        # l3 (legnth of line from joint 3 to joint 4 coord frame)
        l3 = consts['l3']

        # a2 (length of link 2)
        a2 = consts['a2']

        # r24 (vector from joint 2 to joint 4 coord frame)
        r24z = r24[2]
        r24xy = (r24[0]**2 + r24[1]**2)**0.5

        # angle r24 (angle of inclination of r24 from XY plane)
        angle_r24 = atan2(r24z, r24xy)
        r24_mag = (r24[0]**2 + r24[1]**2 + r24[2]**2)**0.5

        # angle_a between r24 and a2
        angle_a = acos((-l3**2 + a2**2 + r24_mag**2)/(2*a2*r24_mag))

        # angle_b between a2 and l3
        angle_b = acos((-r24_mag**2 + a2**2 + l3**2)/(2*a2*l3))

        theta2 = pi/2 - angle_a - angle_r24
        theta3 = pi/2 - gamma - angle_b

        return [theta2, theta3]

    def _createTheta456(self, theta1, theta2, theta3, Rrpy):
        '''
        Calculate theta 4, theta5, theta6 based on relations
        between elements in the rotation matrix from frame 3 to 6.
        '''
        #Use rotation matrix from joint 3 to 6 to calculate
        #theta 4, theta5, theta6
        R_03 = self.T[(0,3)][:3,:3].evalf(subs = {self.q1: theta1,

        R_36 = R_03.T*Rrpy
        theta4 = atan2(R_36[2,2], -R_36[0,2])
        theta5 = acos(R_36[1,2])#atan2((R_36[0,2]**2 + R_36[2,2]**2)**0.5, R_36[1,2])
        theta6 = atan2(-R_36[1,1], R_36[1,0])

        return [theta4, theta5, theta6]

    def _create_r24(self, wc, q1):
        r02 = self.T[(0,2)][:3,3].evalf(subs = {self.q1: q1})
        r24 = wc - r02

        return r24.evalf()

    def _createRrpy(self, quaternion):
        #EE rotation matrix
        Trpy = tf.transformations.quaternion_matrix(quaternion)
        Rrpy = Matrix(Trpy[:3,:3])

        #apply correction to orientation to account for coord misalignment
        Rrpy = Rrpy*self.R_corr

        return Rrpy

    def _createWristCenter(self, consts, ee_target, Rrpy):
        #wrist center wc = [[wx], [wy], [wz]] in base coords
        wc_target = ee_target - Rrpy*Matrix([0,0,consts['d7']])
        wc_target = wc_target.evalf()

        return wc_target

    def calculateJointAngles(self, px, py, pz, quaternion):
        #Calculate end effector orientation rotation matrix
        #from EE quaternion
        Rrpy = self._createRrpy(quaternion)

        #Calculate wrist center and end effector target positions
        ee_target = Matrix([px, py, pz])
        wc_target = self._createWristCenter(self.consts, ee_target, Rrpy)

        theta1 = self._createTheta1(wc_target)

        #Calculate r24 (vector from joint 2 to joint 4)
        r24 = self._create_r24(wc_target, theta1)

        theta2, theta3 = self._createTheta23(self.consts, r24)

        theta4, theta5, theta6 = self._createTheta456(theta1, theta2, theta3, Rrpy)

        joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]

        [wc_actual,
         wc_error,
         ee_actual,
         ee_error] = self.calculateIKError(wc_target, ee_target, joint_angles)

        return [joint_angles, wc_actual, wc_error, ee_actual, ee_error]
