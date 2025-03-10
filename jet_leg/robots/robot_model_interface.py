# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

from jet_leg.robots.hyq.hyq_model import HyqModel
from jet_leg.robots.anymal.anymal_model import AnymalModel
from jet_leg.robots.hyqreal.hyqreal_model import HyqrealModel
from jet_leg.robots.crex.crex_model import CrexModel
from jet_leg.robots.aliengo.aliengo_model import AliengoModel
from jet_leg.robots.mantis_belt_star.mantis_belt_star_model import MantisBeltStarModel


class RobotModelInterface:
    def __init__(self, robot_name):
        self.robotName = robot_name
        if self.robotName == 'hyq':
            self.robotModel = HyqModel()
        elif self.robotName == 'anymal_boxy' or self.robotName == 'anymal_coyote' or self.robotName == 'anymal':
            self.robotModel = AnymalModel(self.robotName)
        elif self.robotName == 'hyqreal':
            self.robotModel = HyqrealModel()
        elif self.robotName == 'crex':
            self.robotModel = CrexModel()
        elif self.robotName == 'aliengo':
            self.robotModel = AliengoModel()
            
        self.joint_torque_limits = self.robotModel.joint_torque_limits
        self.joint_limits_max = self.robotModel.joint_limits_max
        self.joint_limits_min = self.robotModel.joint_limits_min
        self.contact_torque_limits = self.robotModel.contact_torque_limits
        self.trunkMass = self.robotModel.trunkMass
        self.trunkInertia = self.robotModel.trunkInertia
        self.com_BF = self.robotModel.com_BF
        self.nominal_stance_LF = self.robotModel.nominal_stance_LF
        self.nominal_stance_RF = self.robotModel.nominal_stance_RF
        self.nominal_stance_LH = self.robotModel.nominal_stance_LH
        self.nominal_stance_RH = self.robotModel.nominal_stance_RH
        self.max_dev_from_nominal = self.robotModel.max_dev_from_nominal
        self.max_ext_force = self.robotModel.max_ext_force
        self.max_ext_torque = self.robotModel.max_ext_torque
