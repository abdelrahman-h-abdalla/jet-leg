# -*- coding: utf-8 -*-
"""
Created on Sun Jul 22 21:26:46 2018

@author: romeoorsolino
"""
import numpy as np
from context import jet_leg

from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics

import random

import unittest
class TestLPGroundTruth(unittest.TestCase):
    
    epsilon = 10e-02
    assertPrecision = 3
           
    def test_LP_actuation_constraints_only(self):
        math = Math()
        # number of contacts
        nc = 3
        # number of generators, i.e. rays used to linearize the friction cone
        ng = 4
        
        # ONLY_ACTUATION or ONLY_FRICTION
        constraint_mode = 'ONLY_ACTUATION'

        useVariableJacobian = True
        
        """ contact points """    
        LF_foot = np.array([0.3, 0.2, -0.65])
        RF_foot = np.array([0.3, -0.2, -0.65])
        LH_foot = np.array([-0.2, 0.2, -0.4])
        RH_foot = np.array([-0.3, -0.2, -0.65])
        
        contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        contacts = contactsToStack[0:nc, :]
        
        ''' parameters to be tuned'''
        g = 9.81
        trunk_mass = 90.
        mu = 0.8
        
        axisZ= np.array([[0.0], [0.0], [1.0]])
        
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
        normals = np.vstack([n1, n2, n3, n4])
        comp_dyn = ComputationalDynamics()
        feasible, unfeasible, contact_forces = comp_dyn.LP_projection(constraint_mode, contacts, normals, trunk_mass, mu, ng, nc, mu, useVariableJacobian)

        expected_feasible = np.array([[ -1.50000000e-01,   2.50000000e-01,   5.00000000e-02],
                                      [ -1.50000000e-01,   2.50000000e-01,   1.00000000e-01],
                                      [ -1.50000000e-01,   3.00000000e-01,   5.00000000e-02],
                                      [ -1.50000000e-01,   3.00000000e-01,   1.00000000e-01],
                                      [ -1.50000000e-01,   3.50000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   1.00000000e-01,  -5.55111512e-17],
                                      [ -1.00000000e-01,   1.00000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   1.00000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   1.50000000e-01,  -5.55111512e-17],
                                      [ -1.00000000e-01,   1.50000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   1.50000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   2.00000000e-01,  -5.55111512e-17],
                                      [ -1.00000000e-01,   2.00000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   2.00000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   2.50000000e-01,  -5.55111512e-17],
                                      [ -1.00000000e-01,   2.50000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   2.50000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   3.00000000e-01,  -5.55111512e-17],
                                      [ -1.00000000e-01,   3.00000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   3.00000000e-01,   1.00000000e-01],
                                      [ -1.00000000e-01,   3.50000000e-01,   5.00000000e-02],
                                      [ -1.00000000e-01,   3.50000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,  -1.11022302e-16,  -1.00000000e-01],
                                      [ -5.00000000e-02,  -1.11022302e-16,  -5.00000000e-02],
                                      [ -5.00000000e-02,  -1.11022302e-16,  -5.55111512e-17],
                                      [ -5.00000000e-02,  -1.11022302e-16,   5.00000000e-02],
                                      [ -5.00000000e-02,  -1.11022302e-16,   1.00000000e-01],
                                      [ -5.00000000e-02,   5.00000000e-02,  -1.50000000e-01],
                                      [ -5.00000000e-02,   5.00000000e-02,  -1.00000000e-01],
                                      [ -5.00000000e-02,   5.00000000e-02,  -5.00000000e-02],
                                      [ -5.00000000e-02,   5.00000000e-02,  -5.55111512e-17],
                                      [ -5.00000000e-02,   5.00000000e-02,   5.00000000e-02],
                                      [ -5.00000000e-02,   5.00000000e-02,   1.00000000e-01],
                                      [ -5.00000000e-02,   1.00000000e-01,  -2.00000000e-01],
                                      [ -5.00000000e-02,   1.00000000e-01,  -1.50000000e-01],
                                      [ -5.00000000e-02,   1.00000000e-01,  -1.00000000e-01],
                                      [ -5.00000000e-02,   1.00000000e-01,  -5.00000000e-02],
                                      [ -5.00000000e-02,   1.00000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   1.00000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   1.00000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,   1.50000000e-01,  -2.00000000e-01],
                                      [ -5.00000000e-02,   1.50000000e-01,  -1.50000000e-01],
                                      [ -5.00000000e-02,   1.50000000e-01,  -1.00000000e-01],
                                      [ -5.00000000e-02,   1.50000000e-01,  -5.00000000e-02],
                                      [ -5.00000000e-02,   1.50000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   1.50000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   1.50000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,   2.00000000e-01,  -2.00000000e-01],
                                      [ -5.00000000e-02,   2.00000000e-01,  -1.50000000e-01],
                                      [ -5.00000000e-02,   2.00000000e-01,  -1.00000000e-01],
                                      [ -5.00000000e-02,   2.00000000e-01,  -5.00000000e-02],
                                      [ -5.00000000e-02,   2.00000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   2.00000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   2.00000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,   2.50000000e-01,  -2.00000000e-01],
                                      [ -5.00000000e-02,   2.50000000e-01,  -1.50000000e-01],
                                      [ -5.00000000e-02,   2.50000000e-01,  -1.00000000e-01],
                                      [ -5.00000000e-02,   2.50000000e-01,  -5.00000000e-02],
                                      [ -5.00000000e-02,   2.50000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   2.50000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   2.50000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,   3.00000000e-01,  -5.00000000e-02],
                                      [ -5.00000000e-02,   3.00000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   3.00000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   3.00000000e-01,   1.00000000e-01],
                                      [ -5.00000000e-02,   3.50000000e-01,  -5.55111512e-17],
                                      [ -5.00000000e-02,   3.50000000e-01,   5.00000000e-02],
                                      [ -5.00000000e-02,   3.50000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,  -5.00000000e-02,  -1.00000000e-01],
                                      [ -1.11022302e-16,  -5.00000000e-02,  -5.00000000e-02],
                                      [ -1.11022302e-16,  -5.00000000e-02,  -5.55111512e-17],
                                      [ -1.11022302e-16,  -5.00000000e-02,   5.00000000e-02],
                                      [ -1.11022302e-16,  -5.00000000e-02,   1.00000000e-01],
                                      [ -1.11022302e-16,  -1.11022302e-16,  -1.50000000e-01],
                                      [ -1.11022302e-16,  -1.11022302e-16,  -1.00000000e-01],
                                      [ -1.11022302e-16,  -1.11022302e-16,  -5.00000000e-02],
                                      [ -1.11022302e-16,  -1.11022302e-16,  -5.55111512e-17],
                                      [ -1.11022302e-16,  -1.11022302e-16,   5.00000000e-02],
                                      [ -1.11022302e-16,  -1.11022302e-16,   1.00000000e-01],
                                      [ -1.11022302e-16,   5.00000000e-02,  -1.50000000e-01],
                                      [ -1.11022302e-16,   5.00000000e-02,  -1.00000000e-01],
                                      [ -1.11022302e-16,   5.00000000e-02,  -5.00000000e-02],
                                      [ -1.11022302e-16,   5.00000000e-02,  -5.55111512e-17],
                                      [ -1.11022302e-16,   5.00000000e-02,   5.00000000e-02],
                                      [ -1.11022302e-16,   5.00000000e-02,   1.00000000e-01],
                                      [ -1.11022302e-16,   1.00000000e-01,  -2.00000000e-01],
                                      [ -1.11022302e-16,   1.00000000e-01,  -1.50000000e-01],
                                      [ -1.11022302e-16,   1.00000000e-01,  -1.00000000e-01],
                                      [ -1.11022302e-16,   1.00000000e-01,  -5.00000000e-02],
                                      [ -1.11022302e-16,   1.00000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   1.00000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   1.00000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,   1.50000000e-01,  -2.00000000e-01],
                                      [ -1.11022302e-16,   1.50000000e-01,  -1.50000000e-01],
                                      [ -1.11022302e-16,   1.50000000e-01,  -1.00000000e-01],
                                      [ -1.11022302e-16,   1.50000000e-01,  -5.00000000e-02],
                                      [ -1.11022302e-16,   1.50000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   1.50000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   1.50000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,   2.00000000e-01,  -2.00000000e-01],
                                      [ -1.11022302e-16,   2.00000000e-01,  -1.50000000e-01],
                                      [ -1.11022302e-16,   2.00000000e-01,  -1.00000000e-01],
                                      [ -1.11022302e-16,   2.00000000e-01,  -5.00000000e-02],
                                      [ -1.11022302e-16,   2.00000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   2.00000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   2.00000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,   2.50000000e-01,  -1.50000000e-01],
                                      [ -1.11022302e-16,   2.50000000e-01,  -1.00000000e-01],
                                      [ -1.11022302e-16,   2.50000000e-01,  -5.00000000e-02],
                                      [ -1.11022302e-16,   2.50000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   2.50000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   2.50000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,   3.00000000e-01,  -5.00000000e-02],
                                      [ -1.11022302e-16,   3.00000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   3.00000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   3.00000000e-01,   1.00000000e-01],
                                      [ -1.11022302e-16,   3.50000000e-01,  -5.55111512e-17],
                                      [ -1.11022302e-16,   3.50000000e-01,   5.00000000e-02],
                                      [ -1.11022302e-16,   3.50000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,  -5.00000000e-02,  -1.00000000e-01],
                                      [  5.00000000e-02,  -5.00000000e-02,  -5.00000000e-02],
                                      [  5.00000000e-02,  -5.00000000e-02,  -5.55111512e-17],
                                      [  5.00000000e-02,  -5.00000000e-02,   5.00000000e-02],
                                      [  5.00000000e-02,  -5.00000000e-02,   1.00000000e-01],
                                      [  5.00000000e-02,  -1.11022302e-16,  -1.00000000e-01],
                                      [  5.00000000e-02,  -1.11022302e-16,  -5.00000000e-02],
                                      [  5.00000000e-02,  -1.11022302e-16,  -5.55111512e-17],
                                      [  5.00000000e-02,  -1.11022302e-16,   5.00000000e-02],
                                      [  5.00000000e-02,  -1.11022302e-16,   1.00000000e-01],
                                      [  5.00000000e-02,   5.00000000e-02,  -1.00000000e-01],
                                      [  5.00000000e-02,   5.00000000e-02,  -5.00000000e-02],
                                      [  5.00000000e-02,   5.00000000e-02,  -5.55111512e-17],
                                      [  5.00000000e-02,   5.00000000e-02,   5.00000000e-02],
                                      [  5.00000000e-02,   5.00000000e-02,   1.00000000e-01],
                                      [  5.00000000e-02,   1.00000000e-01,  -2.00000000e-01],
                                      [  5.00000000e-02,   1.00000000e-01,  -1.50000000e-01],
                                      [  5.00000000e-02,   1.00000000e-01,  -1.00000000e-01],
                                      [  5.00000000e-02,   1.00000000e-01,  -5.00000000e-02],
                                      [  5.00000000e-02,   1.00000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   1.00000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   1.00000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,   1.50000000e-01,  -2.00000000e-01],
                                      [  5.00000000e-02,   1.50000000e-01,  -1.50000000e-01],
                                      [  5.00000000e-02,   1.50000000e-01,  -1.00000000e-01],
                                      [  5.00000000e-02,   1.50000000e-01,  -5.00000000e-02],
                                      [  5.00000000e-02,   1.50000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   1.50000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   1.50000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,   2.00000000e-01,  -2.00000000e-01],
                                      [  5.00000000e-02,   2.00000000e-01,  -1.50000000e-01],
                                      [  5.00000000e-02,   2.00000000e-01,  -1.00000000e-01],
                                      [  5.00000000e-02,   2.00000000e-01,  -5.00000000e-02],
                                      [  5.00000000e-02,   2.00000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   2.00000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   2.00000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,   2.50000000e-01,  -1.50000000e-01],
                                      [  5.00000000e-02,   2.50000000e-01,  -1.00000000e-01],
                                      [  5.00000000e-02,   2.50000000e-01,  -5.00000000e-02],
                                      [  5.00000000e-02,   2.50000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   2.50000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   2.50000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,   3.00000000e-01,  -5.00000000e-02],
                                      [  5.00000000e-02,   3.00000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   3.00000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   3.00000000e-01,   1.00000000e-01],
                                      [  5.00000000e-02,   3.50000000e-01,  -5.55111512e-17],
                                      [  5.00000000e-02,   3.50000000e-01,   5.00000000e-02],
                                      [  5.00000000e-02,   3.50000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,  -5.00000000e-02,  -5.55111512e-17],
                                      [  1.00000000e-01,  -5.00000000e-02,   5.00000000e-02],
                                      [  1.00000000e-01,  -5.00000000e-02,   1.00000000e-01],
                                      [  1.00000000e-01,  -1.11022302e-16,  -5.55111512e-17],
                                      [  1.00000000e-01,  -1.11022302e-16,   5.00000000e-02],
                                      [  1.00000000e-01,  -1.11022302e-16,   1.00000000e-01],
                                      [  1.00000000e-01,   5.00000000e-02,  -5.55111512e-17],
                                      [  1.00000000e-01,   5.00000000e-02,   5.00000000e-02],
                                      [  1.00000000e-01,   5.00000000e-02,   1.00000000e-01],
                                      [  1.00000000e-01,   1.00000000e-01,  -1.00000000e-01],
                                      [  1.00000000e-01,   1.00000000e-01,  -5.00000000e-02],
                                      [  1.00000000e-01,   1.00000000e-01,  -5.55111512e-17],
                                      [  1.00000000e-01,   1.00000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   1.00000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
                                      [  1.00000000e-01,   1.50000000e-01,  -1.50000000e-01],
                                      [  1.00000000e-01,   1.50000000e-01,  -1.00000000e-01],
                                      [  1.00000000e-01,   1.50000000e-01,  -5.00000000e-02],
                                      [  1.00000000e-01,   1.50000000e-01,  -5.55111512e-17],
                                      [  1.00000000e-01,   1.50000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   1.50000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,   2.00000000e-01,  -2.00000000e-01],
                                      [  1.00000000e-01,   2.00000000e-01,  -1.50000000e-01],
                                      [  1.00000000e-01,   2.00000000e-01,  -1.00000000e-01],
                                      [  1.00000000e-01,   2.00000000e-01,  -5.00000000e-02],
                                      [  1.00000000e-01,   2.00000000e-01,  -5.55111512e-17],
                                      [  1.00000000e-01,   2.00000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   2.00000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,   2.50000000e-01,  -1.00000000e-01],
                                      [  1.00000000e-01,   2.50000000e-01,  -5.00000000e-02],
                                      [  1.00000000e-01,   2.50000000e-01,  -5.55111512e-17],
                                      [  1.00000000e-01,   2.50000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   2.50000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,   3.00000000e-01,  -5.55111512e-17],
                                      [  1.00000000e-01,   3.00000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   3.00000000e-01,   1.00000000e-01],
                                      [  1.00000000e-01,   3.50000000e-01,   5.00000000e-02],
                                      [  1.00000000e-01,   3.50000000e-01,   1.00000000e-01],
                                      [  1.50000000e-01,   1.50000000e-01,  -2.00000000e-01],
                                      [  1.50000000e-01,   1.50000000e-01,  -5.55111512e-17],
                                      [  1.50000000e-01,   1.50000000e-01,   5.00000000e-02],
                                      [  1.50000000e-01,   2.00000000e-01,  -2.00000000e-01],
                                      [  1.50000000e-01,   2.00000000e-01,  -1.50000000e-01],
                                      [  1.50000000e-01,   2.00000000e-01,  -1.00000000e-01],
                                      [  1.50000000e-01,   2.00000000e-01,  -5.00000000e-02],
                                      [  1.50000000e-01,   2.00000000e-01,  -5.55111512e-17],
                                      [  1.50000000e-01,   2.00000000e-01,   5.00000000e-02],
                                      [  1.50000000e-01,   2.50000000e-01,  -1.00000000e-01],
                                      [  1.50000000e-01,   2.50000000e-01,  -5.00000000e-02],
                                      [  1.50000000e-01,   2.50000000e-01,  -5.55111512e-17],
                                      [  1.50000000e-01,   2.50000000e-01,   5.00000000e-02],
                                      [  1.50000000e-01,   3.00000000e-01,  -5.55111512e-17],
                                      [  1.50000000e-01,   3.00000000e-01,   5.00000000e-02],
                                      [  1.50000000e-01,   3.50000000e-01,   5.00000000e-02],
                                      [  2.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
                                      [  2.00000000e-01,   2.00000000e-01,  -1.50000000e-01],
                                      [  2.00000000e-01,   2.50000000e-01,  -5.00000000e-02],
                                      [  2.00000000e-01,   2.50000000e-01,  -5.55111512e-17],
                                      [  2.00000000e-01,   3.00000000e-01,  -5.55111512e-17]]);
#        print expected_feasible
        rows = np.size(expected_feasible, 0)
        cols = np.size(expected_feasible, 1) 
        for i in range(0, rows):
            for j in range (0, cols):
                self.assertAlmostEquals(expected_feasible[i,j], feasible[i,j], self.assertPrecision)
                
    def test_LP_friction_constraints_only(self):
        
        math = Math()
        # number of contacts
        nc = 3
        # number of generators, i.e. rays used to linearize the friction cone
        ng = 4
        
        # ONLY_FRICTION
        constraint_mode = 'ONLY_FRICTION'

        useVariableJacobian = True

        
        LF_foot = np.array([0.3, 0.2, -0.65])
        RF_foot = np.array([0.3, -0.2, -0.65])
        LH_foot = np.array([-0.2, 0.2, -0.4])
        RH_foot = np.array([-0.3, -0.2, -0.65])
        
        contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        contacts = contactsToStack[0:nc, :]
        
        ''' parameters to be tuned'''
        g = 9.81
        trunk_mass = 90.
        mu = 0.8
        
        axisZ= np.array([[0.0], [0.0], [1.0]])
        
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
        normals = np.vstack([n1, n2, n3, n4])
        comp_dyn = ComputationalDynamics()
        feasible, unfeasible, contact_forces = comp_dyn.LP_projection(constraint_mode, contacts, normals, trunk_mass, mu, ng, nc, mu, useVariableJacobian)

        expected_feasible = np.array([[ -1.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [ -1.00000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [ -1.00000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [ -1.00000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [ -1.00000000e-01,   1.50000000e-01,  -5.55111512e-17],
       [ -1.00000000e-01,   1.50000000e-01,   5.00000000e-02],
       [ -1.00000000e-01,   1.50000000e-01,   1.00000000e-01],
       [ -5.00000000e-02,   1.00000000e-01,  -2.00000000e-01],
       [ -5.00000000e-02,   1.00000000e-01,  -1.50000000e-01],
       [ -5.00000000e-02,   1.00000000e-01,  -1.00000000e-01],
       [ -5.00000000e-02,   1.00000000e-01,  -5.00000000e-02],
       [ -5.00000000e-02,   1.00000000e-01,  -5.55111512e-17],
       [ -5.00000000e-02,   1.00000000e-01,   5.00000000e-02],
       [ -5.00000000e-02,   1.00000000e-01,   1.00000000e-01],
       [ -5.00000000e-02,   1.50000000e-01,  -2.00000000e-01],
       [ -5.00000000e-02,   1.50000000e-01,  -1.50000000e-01],
       [ -5.00000000e-02,   1.50000000e-01,  -1.00000000e-01],
       [ -5.00000000e-02,   1.50000000e-01,  -5.00000000e-02],
       [ -5.00000000e-02,   1.50000000e-01,  -5.55111512e-17],
       [ -5.00000000e-02,   1.50000000e-01,   5.00000000e-02],
       [ -5.00000000e-02,   1.50000000e-01,   1.00000000e-01],
       [ -1.11022302e-16,   5.00000000e-02,  -2.00000000e-01],
       [ -1.11022302e-16,   5.00000000e-02,  -1.50000000e-01],
       [ -1.11022302e-16,   5.00000000e-02,  -1.00000000e-01],
       [ -1.11022302e-16,   5.00000000e-02,  -5.00000000e-02],
       [ -1.11022302e-16,   5.00000000e-02,  -5.55111512e-17],
       [ -1.11022302e-16,   5.00000000e-02,   5.00000000e-02],
       [ -1.11022302e-16,   5.00000000e-02,   1.00000000e-01],
       [ -1.11022302e-16,   1.00000000e-01,  -2.00000000e-01],
       [ -1.11022302e-16,   1.00000000e-01,  -1.50000000e-01],
       [ -1.11022302e-16,   1.00000000e-01,  -1.00000000e-01],
       [ -1.11022302e-16,   1.00000000e-01,  -5.00000000e-02],
       [ -1.11022302e-16,   1.00000000e-01,  -5.55111512e-17],
       [ -1.11022302e-16,   1.00000000e-01,   5.00000000e-02],
       [ -1.11022302e-16,   1.00000000e-01,   1.00000000e-01],
       [ -1.11022302e-16,   1.50000000e-01,  -2.00000000e-01],
       [ -1.11022302e-16,   1.50000000e-01,  -1.50000000e-01],
       [ -1.11022302e-16,   1.50000000e-01,  -1.00000000e-01],
       [ -1.11022302e-16,   1.50000000e-01,  -5.00000000e-02],
       [ -1.11022302e-16,   1.50000000e-01,  -5.55111512e-17],
       [ -1.11022302e-16,   1.50000000e-01,   5.00000000e-02],
       [ -1.11022302e-16,   1.50000000e-01,   1.00000000e-01],
       [  5.00000000e-02,  -1.11022302e-16,  -2.00000000e-01],
       [  5.00000000e-02,  -1.11022302e-16,  -1.50000000e-01],
       [  5.00000000e-02,  -1.11022302e-16,  -1.00000000e-01],
       [  5.00000000e-02,  -1.11022302e-16,  -5.00000000e-02],
       [  5.00000000e-02,  -1.11022302e-16,  -5.55111512e-17],
       [  5.00000000e-02,  -1.11022302e-16,   5.00000000e-02],
       [  5.00000000e-02,  -1.11022302e-16,   1.00000000e-01],
       [  5.00000000e-02,   5.00000000e-02,  -2.00000000e-01],
       [  5.00000000e-02,   5.00000000e-02,  -1.50000000e-01],
       [  5.00000000e-02,   5.00000000e-02,  -1.00000000e-01],
       [  5.00000000e-02,   5.00000000e-02,  -5.00000000e-02],
       [  5.00000000e-02,   5.00000000e-02,  -5.55111512e-17],
       [  5.00000000e-02,   5.00000000e-02,   5.00000000e-02],
       [  5.00000000e-02,   5.00000000e-02,   1.00000000e-01],
       [  5.00000000e-02,   1.00000000e-01,  -2.00000000e-01],
       [  5.00000000e-02,   1.00000000e-01,  -1.50000000e-01],
       [  5.00000000e-02,   1.00000000e-01,  -1.00000000e-01],
       [  5.00000000e-02,   1.00000000e-01,  -5.00000000e-02],
       [  5.00000000e-02,   1.00000000e-01,  -5.55111512e-17],
       [  5.00000000e-02,   1.00000000e-01,   5.00000000e-02],
       [  5.00000000e-02,   1.00000000e-01,   1.00000000e-01],
       [  5.00000000e-02,   1.50000000e-01,  -2.00000000e-01],
       [  5.00000000e-02,   1.50000000e-01,  -1.50000000e-01],
       [  5.00000000e-02,   1.50000000e-01,  -1.00000000e-01],
       [  5.00000000e-02,   1.50000000e-01,  -5.00000000e-02],
       [  5.00000000e-02,   1.50000000e-01,  -5.55111512e-17],
       [  5.00000000e-02,   1.50000000e-01,   5.00000000e-02],
       [  5.00000000e-02,   1.50000000e-01,   1.00000000e-01],
       [  5.00000000e-02,   2.00000000e-01,  -2.00000000e-01],
       [  5.00000000e-02,   2.00000000e-01,  -1.50000000e-01],
       [  5.00000000e-02,   2.00000000e-01,  -1.00000000e-01],
       [  5.00000000e-02,   2.00000000e-01,  -5.00000000e-02],
       [  5.00000000e-02,   2.00000000e-01,  -5.55111512e-17],
       [  5.00000000e-02,   2.00000000e-01,   5.00000000e-02],
       [  5.00000000e-02,   2.00000000e-01,   1.00000000e-01],
       [  1.00000000e-01,  -1.11022302e-16,  -2.00000000e-01],
       [  1.00000000e-01,  -1.11022302e-16,  -1.50000000e-01],
       [  1.00000000e-01,  -1.11022302e-16,  -1.00000000e-01],
       [  1.00000000e-01,  -1.11022302e-16,  -5.00000000e-02],
       [  1.00000000e-01,  -1.11022302e-16,  -5.55111512e-17],
       [  1.00000000e-01,  -1.11022302e-16,   5.00000000e-02],
       [  1.00000000e-01,  -1.11022302e-16,   1.00000000e-01],
       [  1.00000000e-01,   5.00000000e-02,  -2.00000000e-01],
       [  1.00000000e-01,   5.00000000e-02,  -1.50000000e-01],
       [  1.00000000e-01,   5.00000000e-02,  -1.00000000e-01],
       [  1.00000000e-01,   5.00000000e-02,  -5.00000000e-02],
       [  1.00000000e-01,   5.00000000e-02,  -5.55111512e-17],
       [  1.00000000e-01,   5.00000000e-02,   5.00000000e-02],
       [  1.00000000e-01,   5.00000000e-02,   1.00000000e-01],
       [  1.00000000e-01,   1.00000000e-01,  -2.00000000e-01],
       [  1.00000000e-01,   1.00000000e-01,  -1.50000000e-01],
       [  1.00000000e-01,   1.00000000e-01,  -1.00000000e-01],
       [  1.00000000e-01,   1.00000000e-01,  -5.00000000e-02],
       [  1.00000000e-01,   1.00000000e-01,  -5.55111512e-17],
       [  1.00000000e-01,   1.00000000e-01,   5.00000000e-02],
       [  1.00000000e-01,   1.00000000e-01,   1.00000000e-01],
       [  1.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [  1.00000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [  1.00000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [  1.00000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [  1.00000000e-01,   1.50000000e-01,  -5.55111512e-17],
       [  1.00000000e-01,   1.50000000e-01,   5.00000000e-02],
       [  1.00000000e-01,   1.50000000e-01,   1.00000000e-01],
       [  1.50000000e-01,  -5.00000000e-02,  -2.00000000e-01],
       [  1.50000000e-01,  -5.00000000e-02,  -1.50000000e-01],
       [  1.50000000e-01,  -5.00000000e-02,  -1.00000000e-01],
       [  1.50000000e-01,  -5.00000000e-02,  -5.00000000e-02],
       [  1.50000000e-01,  -5.00000000e-02,  -5.55111512e-17],
       [  1.50000000e-01,  -5.00000000e-02,   5.00000000e-02],
       [  1.50000000e-01,  -1.11022302e-16,  -2.00000000e-01],
       [  1.50000000e-01,  -1.11022302e-16,  -1.50000000e-01],
       [  1.50000000e-01,  -1.11022302e-16,  -1.00000000e-01],
       [  1.50000000e-01,  -1.11022302e-16,  -5.00000000e-02],
       [  1.50000000e-01,  -1.11022302e-16,  -5.55111512e-17],
       [  1.50000000e-01,  -1.11022302e-16,   5.00000000e-02],
       [  1.50000000e-01,   5.00000000e-02,  -2.00000000e-01],
       [  1.50000000e-01,   5.00000000e-02,  -1.50000000e-01],
       [  1.50000000e-01,   5.00000000e-02,  -1.00000000e-01],
       [  1.50000000e-01,   5.00000000e-02,  -5.00000000e-02],
       [  1.50000000e-01,   5.00000000e-02,  -5.55111512e-17],
       [  1.50000000e-01,   5.00000000e-02,   5.00000000e-02],
       [  1.50000000e-01,   1.00000000e-01,  -2.00000000e-01],
       [  1.50000000e-01,   1.00000000e-01,  -1.50000000e-01],
       [  1.50000000e-01,   1.00000000e-01,  -1.00000000e-01],
       [  1.50000000e-01,   1.00000000e-01,  -5.00000000e-02],
       [  1.50000000e-01,   1.00000000e-01,  -5.55111512e-17],
       [  1.50000000e-01,   1.00000000e-01,   5.00000000e-02],
       [  1.50000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [  1.50000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [  1.50000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [  1.50000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [  1.50000000e-01,   1.50000000e-01,  -5.55111512e-17],
       [  1.50000000e-01,   1.50000000e-01,   5.00000000e-02],
       [  2.00000000e-01,  -1.00000000e-01,  -2.00000000e-01],
       [  2.00000000e-01,  -1.00000000e-01,  -1.50000000e-01],
       [  2.00000000e-01,  -1.00000000e-01,  -1.00000000e-01],
       [  2.00000000e-01,  -1.00000000e-01,  -5.00000000e-02],
       [  2.00000000e-01,  -1.00000000e-01,  -5.55111512e-17],
       [  2.00000000e-01,  -1.00000000e-01,   5.00000000e-02],
       [  2.00000000e-01,  -5.00000000e-02,  -2.00000000e-01],
       [  2.00000000e-01,  -5.00000000e-02,  -1.50000000e-01],
       [  2.00000000e-01,  -5.00000000e-02,  -1.00000000e-01],
       [  2.00000000e-01,  -5.00000000e-02,  -5.00000000e-02],
       [  2.00000000e-01,  -5.00000000e-02,  -5.55111512e-17],
       [  2.00000000e-01,  -5.00000000e-02,   5.00000000e-02],
       [  2.00000000e-01,  -1.11022302e-16,  -2.00000000e-01],
       [  2.00000000e-01,  -1.11022302e-16,  -1.50000000e-01],
       [  2.00000000e-01,  -1.11022302e-16,  -1.00000000e-01],
       [  2.00000000e-01,  -1.11022302e-16,  -5.00000000e-02],
       [  2.00000000e-01,  -1.11022302e-16,  -5.55111512e-17],
       [  2.00000000e-01,  -1.11022302e-16,   5.00000000e-02],
       [  2.00000000e-01,   5.00000000e-02,  -2.00000000e-01],
       [  2.00000000e-01,   5.00000000e-02,  -1.50000000e-01],
       [  2.00000000e-01,   5.00000000e-02,  -1.00000000e-01],
       [  2.00000000e-01,   5.00000000e-02,  -5.00000000e-02],
       [  2.00000000e-01,   5.00000000e-02,  -5.55111512e-17],
       [  2.00000000e-01,   5.00000000e-02,   5.00000000e-02],
       [  2.00000000e-01,   1.00000000e-01,  -2.00000000e-01],
       [  2.00000000e-01,   1.00000000e-01,  -1.50000000e-01],
       [  2.00000000e-01,   1.00000000e-01,  -1.00000000e-01],
       [  2.00000000e-01,   1.00000000e-01,  -5.00000000e-02],
       [  2.00000000e-01,   1.00000000e-01,  -5.55111512e-17],
       [  2.00000000e-01,   1.00000000e-01,   5.00000000e-02],
       [  2.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [  2.00000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [  2.00000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [  2.00000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [  2.00000000e-01,   1.50000000e-01,  -5.55111512e-17],
       [  2.00000000e-01,   1.50000000e-01,   5.00000000e-02],
       [  2.00000000e-01,   2.00000000e-01,  -2.00000000e-01],
       [  2.00000000e-01,   2.00000000e-01,  -1.50000000e-01],
       [  2.00000000e-01,   2.00000000e-01,  -1.00000000e-01],
       [  2.00000000e-01,   2.00000000e-01,  -5.00000000e-02],
       [  2.00000000e-01,   2.00000000e-01,  -5.55111512e-17],
       [  2.00000000e-01,   2.00000000e-01,   5.00000000e-02],
       [  2.50000000e-01,  -1.50000000e-01,  -2.00000000e-01],
       [  2.50000000e-01,  -1.50000000e-01,  -1.50000000e-01],
       [  2.50000000e-01,  -1.50000000e-01,  -1.00000000e-01],
       [  2.50000000e-01,  -1.50000000e-01,  -5.00000000e-02],
       [  2.50000000e-01,  -1.50000000e-01,  -5.55111512e-17],
       [  2.50000000e-01,  -1.00000000e-01,  -2.00000000e-01],
       [  2.50000000e-01,  -1.00000000e-01,  -1.50000000e-01],
       [  2.50000000e-01,  -1.00000000e-01,  -1.00000000e-01],
       [  2.50000000e-01,  -1.00000000e-01,  -5.00000000e-02],
       [  2.50000000e-01,  -1.00000000e-01,  -5.55111512e-17],
       [  2.50000000e-01,  -5.00000000e-02,  -2.00000000e-01],
       [  2.50000000e-01,  -5.00000000e-02,  -1.50000000e-01],
       [  2.50000000e-01,  -5.00000000e-02,  -1.00000000e-01],
       [  2.50000000e-01,  -5.00000000e-02,  -5.00000000e-02],
       [  2.50000000e-01,  -5.00000000e-02,  -5.55111512e-17],
       [  2.50000000e-01,  -1.11022302e-16,  -2.00000000e-01],
       [  2.50000000e-01,  -1.11022302e-16,  -1.50000000e-01],
       [  2.50000000e-01,  -1.11022302e-16,  -1.00000000e-01],
       [  2.50000000e-01,  -1.11022302e-16,  -5.00000000e-02],
       [  2.50000000e-01,  -1.11022302e-16,  -5.55111512e-17],
       [  2.50000000e-01,   5.00000000e-02,  -2.00000000e-01],
       [  2.50000000e-01,   5.00000000e-02,  -1.50000000e-01],
       [  2.50000000e-01,   5.00000000e-02,  -1.00000000e-01],
       [  2.50000000e-01,   5.00000000e-02,  -5.00000000e-02],
       [  2.50000000e-01,   5.00000000e-02,  -5.55111512e-17],
       [  2.50000000e-01,   1.00000000e-01,  -2.00000000e-01],
       [  2.50000000e-01,   1.00000000e-01,  -1.50000000e-01],
       [  2.50000000e-01,   1.00000000e-01,  -1.00000000e-01],
       [  2.50000000e-01,   1.00000000e-01,  -5.00000000e-02],
       [  2.50000000e-01,   1.00000000e-01,  -5.55111512e-17],
       [  2.50000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [  2.50000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [  2.50000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [  2.50000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [  2.50000000e-01,   1.50000000e-01,  -5.55111512e-17],
       [  3.00000000e-01,  -1.50000000e-01,  -2.00000000e-01],
       [  3.00000000e-01,  -1.50000000e-01,  -1.50000000e-01],
       [  3.00000000e-01,  -1.50000000e-01,  -1.00000000e-01],
       [  3.00000000e-01,  -1.50000000e-01,  -5.00000000e-02],
       [  3.00000000e-01,  -1.50000000e-01,  -5.55111512e-17],
       [  3.00000000e-01,   1.50000000e-01,  -2.00000000e-01],
       [  3.00000000e-01,   1.50000000e-01,  -1.50000000e-01],
       [  3.00000000e-01,   1.50000000e-01,  -1.00000000e-01],
       [  3.00000000e-01,   1.50000000e-01,  -5.00000000e-02],
       [  3.00000000e-01,   1.50000000e-01,  -5.55111512e-17]]);
#        print expected_feasible
        rows = np.size(expected_feasible, 0)
        cols = np.size(expected_feasible, 1) 
        for i in range(0, rows):
            for j in range (0, cols):
                self.assertAlmostEquals(expected_feasible[i,j], feasible[i,j], self.assertPrecision)