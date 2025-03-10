# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.robots.hyq.hyq_kinematics import HyQKinematics
from jet_leg.kinematics.kinematics_pinocchio import robotKinematics


class KinematicsInterface:
    def __init__(self, robot_name):

        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.robotName = robot_name
        self.ik_success = True
        if robot_name == 'hyq':
            self.hyqKin = HyQKinematics()
        else:
            self.robotKin = robotKinematics(robot_name)

    def get_jacobians(self):
        if self.robotName == 'hyq':
            return self.hyqKin.getLegJacobians()
        else:
            return self.robotKin.getLegJacobians()

    def get_current_q(self):
        if self.robotName == 'hyq':
            return self.hyqKin.getCurrentQ()
        else:
            return self.robotKin.getCurrentQ()

    def inverse_kin(self, contactsBF, foot_vel, stance_index, q_0=None):

        if self.robotName == 'hyq':
            q = self.hyqKin.fixedBaseInverseKinematics(contactsBF, foot_vel)
            return q
        else:
            q, self.ik_success = self.robotKin.fixedBaseInverseKinematics(contactsBF, stance_index, q_0)
            return q

    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        if self.robotName == 'hyq':
            return self.hyqKin.isOutOfJointLims(joint_positions, joint_limits_max, joint_limits_min)
        else:
            return self.robotKin.isOutOfJointLims(joint_positions, joint_limits_max, joint_limits_min)


    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):

        if self.robotName == 'hyq':
            return self.hyqKin.isOutOfWorkSpace(contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel)
        else:
            return self.robotKin.isOutOfWorkSpace(contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel)