# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 15:07:45 2018

@author: Romeo Orsolino
"""
from __future__ import absolute_import

import numpy as np
from .math_tools import Math
from scipy.spatial.transform import Rotation as Rot
from jet_leg.robots.robot_model_interface import RobotModelInterface


class IterativeProjectionParameters:
	def __init__(self, robot_name):

		self.robotName = robot_name
		self.robotModel = RobotModelInterface(self.robotName)
		self.no_of_legs = 4
		self.stride = 3 # Used for receiving data from ROS arrays

		self.math = Math()
		self.q = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
		self.comPositionBF = [0., 0., 0.]  # var used only for IK inside constraints.py
		self.comPositionWF = [0., 0., 0.]
		self.comLinAcc = [0., 0., 0.]
		self.comAngAcc = [0., 0., 0.]
		self.comLinVel = [0., 0., 0.] # Used for ICP computation
		self.comAngVel = [0., 0., 0.]
		self.externalForce = [0., 0., 0.]
		self.externalCentroidalTorque = [0., 0., 0.]
		self.externalCentroidalWrench = np.hstack([self.externalForce, self.externalCentroidalTorque])

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0
		self.eurlerAngles = [self.roll, self.pitch, self.yaw]

		self.LF_tau_lim = [50.0, 50.0, 50.0]
		self.RF_tau_lim = [50.0, 50.0, 50.0]
		self.LH_tau_lim = [50.0, 50.0, 50.0]
		self.RH_tau_lim = [50.0, 50.0, 50.0]

		self.LF_q_lim_max = [0., 0., 0.]
		self.LF_q_lim_min = [0., 0., 0.]
		self.RF_q_lim_max = [0., 0., 0.]
		self.RF_q_lim_min = [0., 0., 0.]
		self.LH_q_lim_max = [0., 0., 0.]
		self.LH_q_lim_min = [0., 0., 0.]
		self.RH_q_lim_max = [0., 0., 0.]
		self.RH_q_lim_min = [0., 0., 0.]

		self.torque_limits = np.array([self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim])
		self.leg_self_weight = np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])
		self.joint_limits_max = np.array([self.LF_q_lim_max, self.RF_q_lim_max, self.LH_q_lim_max, self.RH_q_lim_max])
		self.joint_limits_min = np.array([self.LF_q_lim_min, self.RF_q_lim_min, self.LH_q_lim_min, self.RH_q_lim_min])

		self.state_machineLF = True
		self.state_machineRF = True
		self.state_machineLH = True
		self.state_machineRH = True
		self.state_machine = True
		self.stanceFeet = [0, 0, 0, 0]
		self.numberOfContacts = 0
		#        self.contactsHF = np.zeros((4,3))
		self.contactsBF = np.zeros((4, 3))
		self.contactsWF = np.zeros((4, 3))

		axisZ = np.array([[0.0], [0.0], [1.0]])
		n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
		# %% Cell 2
		self.normals = np.vstack([n1, n2, n3, n4])
		self.constraintMode = ['FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION',
							   'FRICTION_AND_ACTUATION']

		self.friction = 0.8
		self.robotMass = self.robotModel.trunkMass #Kg
		self.robotInertia = self.robotModel.trunkInertia #Kg
		self.numberOfGenerators = 4
		self.useContactTorque = False  # False if contact torques are allowed (e.g. humanoid foot or double support quadruped).
		self.useInstantaneousCapturePoint = False # Used for margin definition
		self.actual_swing = 0

		self.plane_normal = [0, 0, 1]

		self.inertialForces = [0, 0, 0]
		self.inertialMoments = [0, 0, 0]

		# CoM height (projected along z axis)
		self.com_vertical_shift = 0

		# Planning targets
		self.target_CoM_WF = np.array([0., 0., 0.])

	def computeContactsPosBF(self):
		self.contactsBF = np.zeros((4, 3))
		rpy = self.getOrientation()
		rot = Rot.from_euler('xyz',[rpy[0], rpy[1], rpy[2]], degrees=False)
		B_R_W = rot.inv().as_dcm()
		for j in np.arange(0, 4):
			j = int(j)
			self.contactsBF[j,:] = np.add(np.dot(B_R_W, (self.contactsWF[j, :] - self.comPositionWF)),
				 self.comPositionBF)
		return self.contactsBF

	def setContactsPosBF(self, contactsBF):
		self.contactsBF = contactsBF

	def setContactsPosWF(self, contactsWF):
		self.contactsWF = contactsWF

	def setCoMPosWF(self, comWF):
		self.comPositionWF = comWF

	def setCoMPosBF(self, comBF):
		self.comPositionBF = comBF

	def setOrientation(self, rpy):
		self.roll = rpy[0]
		self.pitch = rpy[1]
		self.yaw = rpy[2]

	def setCoMLinAcc(self, comLinAcc):
		self.comLinAcc = comLinAcc

	def setCoMAngAcc(self, comAngAcc):
		self.comAngAcc = comAngAcc

	def setCoMLinVel(self, comLinVel):
		self.comLinVel = comLinVel

	def setCoMAngVel(self, comAngVel):
		self.comAngVel = comAngVel

	def setTorqueLims(self, torqueLims):
		self.torque_limits = torqueLims

	def setJointLimsMax(self, jointLims_max):
		self.joint_limits_max = jointLims_max

	def setJointLimsMin(self, jointLims_min):
		self.joint_limits_min = jointLims_min

	def setActiveContacts(self, activeContacts):
		self.stanceFeet = activeContacts
		self.numberOfContacts = np.sum(self.stanceFeet)
		self.useContactTorque = True if self.numberOfContacts < 3 else False

	# print self.stanceFeet

	def setContactNormals(self, normals):
		self.normals = normals

	def setConstraintModes(self, constraintMode):
		self.constraintMode = constraintMode

	def setFrictionCoefficient(self, mu):
		self.friction = mu

	def setNumberOfFrictionConesEdges(self, ng):
		self.numberOfGenerators = ng

	def setTotalMass(self, mass):
		self.robotMass = mass
	
	def setTotalInertia(self, inertia):
		self.robotInertia = inertia
	
	def setEulerAngles(self, eurlerAngles):
		self.roll = eurlerAngles[0]
		self.pitch = eurlerAngles[1]
		self.yaw = eurlerAngles[2]
		self.eurlerAngles = [self.roll, self.pitch, self.yaw]

	def setInstantaneousCapturePoint(self, ICP):
		self.instantaneousCapturePoint = ICP
	
	def set_plane_normal(self, plane_normal):
		self.plane_normal = plane_normal

	def setRobotName(self,robotName):
		self.robot_name = robotName
		
	def setNoOfLegs(self, numberOfLegs):
		self.no_of_legs = numberOfLegs

	def getContactsPosWF(self):
		return self.contactsWF

	def getContactsPosBF(self):  # used only for IK inside constraints.py
		return self.contactsBF

	def getCoMPosWF(self):
		return self.comPositionWF

	def getCoMPosBF(self):
		return self.comPositionBF

	def getCoMLinAcc(self):
		return self.comLinAcc

	def getCoMAngAcc(self):
		return self.comAngAcc
	
	def getCoMLinVel(self):
		return self.comLinVel

	def getCoMAngVel(self):
		return self.comAngVel

	def getInertialForces(self):
		return self.inertialForces

	def getInertialMoments(self):
		return self.inertialMoments

	def getTorqueLims(self):
		return self.torque_limits

	def getLegSelfWeight(self):
		return self.leg_self_weight

	def getJointLimsMax(self):
		return self.joint_limits_max

	def getJointLimsMin(self):
		return self.joint_limits_min

	def getExternalForce(self):
		return self.externalForce

	def getExternalCentroidalTorque(self):
		return self.externalCentroidalTorque

	def getExternalCentroidalWrench(self):
		return self.externalCentroidalWrench

	def getStanceFeet(self):
		return self.stanceFeet

	def getNormals(self):
		return self.normals

	def getOrientation(self):
		return [self.roll, self.pitch, self.yaw]

	def getConstraintModes(self):
		return self.constraintMode

	def getFrictionCoefficient(self):
		return self.friction

	def getNumberOfFrictionConesEdges(self):
		return self.numberOfGenerators

	def getTotalMass(self):
		return self.robotMass
	
	def getTotalInertia(self):
		return self.robotInertia
	
	def getInstantaneousCapturePoint(self):
		return self.instantaneousCapturePoint

	def get_plane_normal(self):
		return self.plane_normal

	def get_com_vertical_shift(self):
		return self.com_vertical_shift

	def get_CoM_plane_z_intercept(self):
		point_on_plane = self.comPositionWF
		return self.math.plane_z_intercept(point_on_plane, self.plane_normal)

	def get_terrain_plane_z_intercept(self):
		point_on_plane = self.comPositionWF - np.array([0, 0, self.com_vertical_shift])
		return self.math.plane_z_intercept(point_on_plane, self.plane_normal)

	def get_target_CoM_WF(self):
		return self.target_CoM_WF
		
	def getNoOfLegs(self):
		return self.no_of_legs
	
	def getRobotName(self):
		return self.robot_name
	
	''' Compute average height of robot from feet in WF
		This computes the com_vertical_shift if not provided'''
	def computeAverageRobotHeight(self):
		avg_foot_height = 0.0
		nc = np.sum(self.stanceFeet)
		stanceID = self.getStanceIndex(self.stanceFeet)
		for j in range(0, nc):  # only contact positions and normals of the feet that are in stance
			idx = int(stanceID[j])
			avg_foot_height += self.contactsWF[idx][2]
		
		numberOfStanceFeet = sum(self.stanceFeet)
		avg_foot_height = avg_foot_height/numberOfStanceFeet
		avg_height = self.comPositionWF[2] - avg_foot_height
		return avg_height

	def getStanceIndex(self, stanceLegs):
		stanceIdx = []

		#        print 'stance', stanceLegs
		for iter in range(0, self.no_of_legs):
			if stanceLegs[iter] == 1:
				#                print 'new poly', stanceIndex, iter
				stanceIdx = np.hstack([stanceIdx, int(iter)])

		try:
			stanceIdx = stanceIdx.astype(np.int)
		except AttributeError as e:
			pass

		return stanceIdx

	def getParamsFromRosDebugTopic(self, received_data):
		# print 'number of elements: ', num_of_elements
		#            print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")

		# Torque Limits
		# TO-DO in framework:
		# Changing torque limits in urdf still doesn't change
		# value used by trunk controller and Jet-leg.
		# For now, change manually here.
		self.torque_limits = np.zeros((self.no_of_legs, 3))
		self.leg_self_weight = np.zeros((self.no_of_legs, 3))
		for leg in range(0, self.no_of_legs):
			self.torque_limits[leg] = (received_data.tau_lim.data[self.stride * leg: self.stride * leg + 3])
			self.leg_self_weight[leg] = (received_data.leg_self_weight.data[self.stride * leg: self.stride * leg + 3])

		# Joint Limits
		self.joint_limits_max = np.zeros((self.no_of_legs, 3))
		self.joint_limits_min = np.zeros((self.no_of_legs, 3))
		for leg in range(0, self.no_of_legs):
			self.joint_limits_max[leg] = received_data.q_lim_max.data[self.stride * leg: self.stride * leg + 3]
		for leg in range(0, self.no_of_legs):
			self.joint_limits_min[leg] = received_data.q_lim_min.data[self.stride * leg: self.stride * leg + 3]
		#
		# 		# the inputs are all in the WF this way we can compute generic regions for generic contact sets and generic com position
		self.contactsWF = np.zeros((self.no_of_legs, 3))
		for leg in range(0, self.no_of_legs):
			self.contactsWF[leg] = received_data.contactsWF.data[self.stride * leg: self.stride * leg + 3]

		self.comPositionWF = received_data.actual_CoM

		self.comPositionBF = received_data.off_CoM

		# External wrench
		self.externalForce = received_data.ext_wrench[0:3]
		self.externalCentroidalTorque = received_data.ext_wrench[3:6]
		self.externalCentroidalWrench = np.hstack([self.externalForce, self.externalCentroidalTorque])
		self.instantaneousCapturePoint = [0.0, 0.0]

		# 		# print 'ext force ',self.externalForceWF

		# 		# they are in WF
		
		self.normals = np.zeros((self.no_of_legs, 3))
		for leg in range(0, self.no_of_legs):
			self.normals[leg] = received_data.normals.data[self.stride * leg: self.stride * leg + 3]

		self.robotMass = received_data.robot_mass
		# for i in range(3):
		# 	self.robotInertia[i] = received_data.inertia_matrix[i*3 : i*3+3]

		self.friction = received_data.mu_estimate

		self.plane_normal = received_data.plane_normal

		# what it matters is the relative position of act com wrt the region, however for visualization purposes we
		self.com_vertical_shift = received_data.com_vertical_shift
		#
		self.roll = received_data.roll
		self.pitch = received_data.pitch
		self.yaw = received_data.yaw
		#
		self.actual_swing = received_data.actual_swing  # variable doesn't change in framework. Needs fix

		for dir in range(0, 3):
			self.target_CoM_WF[dir] = received_data.target_CoM_WF[dir]

		self.comLinAcc = np.array(received_data.desired_acceleration)
		# self.comAngAcc = np.array(received_data.desired_ang_acceleration)
		# self.comLinVel = np.array(received_data.desired_lin_velocity)
		# self.comAngVel = np.array(received_data.desired_ang_velocity)

		# for dir in range(0, 3):
		# 	self.inertialForces[dir] = received_data.inertial_force[dir]
		# for dir in range(0, 3):
		# 	self.inertialMoments[dir] = received_data.inertial_moment[dir]

	def getRobotNameFromMsg(self, received_data):
		self.robot_name = received_data.robot_name.data

	def getNoOfLegsFromMsg(self, received_data):
		self.no_of_legs = received_data.no_of_legs

	def getFutureStanceFeetFlags(self, received_data):

		self.stanceFeet = received_data.future_stance_legs

		self.numberOfContacts = np.sum(self.stanceFeet)

	def getCurrentStanceFeetFlags(self, received_data):

		self.stanceFeet = received_data.current_stance_legs

		self.numberOfContacts = np.sum(self.stanceFeet)
		self.useContactTorque = True if self.numberOfContacts < 3 else False

