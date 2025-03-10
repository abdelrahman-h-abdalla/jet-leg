# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Abdelrahman Abdalla
"""

from numpy import array, cos, sin, cross, pi
from scipy.linalg import norm
from scipy.spatial import ConvexHull
from jet_leg.computational_geometry.geometry import Geometry
import time
import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.kinematics.kinematics_interface import KinematicsInterface



class FeasibleWorkspace:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.kin = KinematicsInterface(self.robotName)
		self.geom = Geometry()
		self.math = Math()
		self.points = []
		self.points_states = []

	# @property
	# def params(self):
	# 	return self.params
	#
	# @params.setter
	# def params(self, _params):
	# 	self.params = _params

	def getcontactsBF(self, params, comPositionWF):

		comPositionBF = params.getCoMPosBF()
		contactsWF = params.getContactsPosWF()
		contactsBF = np.zeros((4, 3))
		rpy = params.getOrientation()

		for j in np.arange(0, 4):
			j = int(j)
			contactsBF[j, :] = np.add(np.dot(self.math.rpyToRot(rpy[0], rpy[1], rpy[2]),
											 (contactsWF[j, :] - comPositionWF)),
									  comPositionBF)

		return contactsBF

	# def compute_vertix(self, com_pos_x, com_pos_y, com_pos_z, params, theta, dir_step):
	# 	"""
	# 	Compute vertix of projected polygon in vdir direction.
	#
	# 	Solves nonlinear optimization problem by iteratively testing
	# 	over beta-spaced possible CoM positions along vdir direction
	#
	# 	Returns
	# 	-------
	# 	poly: Polygon Output polygon.
	# 	"""
	#
	# 	# Search for a CoM position in direction vdir
	# 	vdir = array([cos(theta), sin(theta), 0])
	#
	# 	c_t = [com_pos_x, com_pos_y, com_pos_z]  # CoM to be tested
	# 	foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
	#
	# 	stanceIndex = params.getStanceIndex(params.getStanceFeet())
	#
	# 	print np.linalg.norm(params.getContactsPosWF()[0] - params.getCoMPosWF())
	# 	max_iter = (np.linalg.norm(params.getContactsPosWF()[0] - params.getCoMPosWF()))
	# 	i = 0
	#
	# 	while i < max_iter:
	# 		# print "cxy_t: ", c_t
	# 		c_t += dir_step * vdir  # iterate along direction vector by step cm
	#
	# 		contactsBF = self.getcontactsBF(params, c_t)
	# 		q = self.kin.inverse_kin(contactsBF, foot_vel)
	# 		q_to_check = np.concatenate(
	# 			[list(q[leg * 3: leg * 3 + 3]) for leg in stanceIndex])  # To check 3 or 4 feet stance
	#
	# 		out = self.kin.isOutOfJointLims(q_to_check, params.getJointLimsMax()[stanceIndex,:],
	# 													params.getJointLimsMin()[stanceIndex,:])
	#
	# 		cxy_t = [c_t[0], c_t[1]]
	# 		self.points.append(cxy_t)
	# 		self.points_states.append(out)
	#
	# 		i = np.linalg.norm(c_t - params.getCoMPosWF())
	# 		# print "i: ", i
	# 		# print "optimal: ", cxy_opt
	#
	# 		# cxy_opt = [cxy_t[0], cxy_t[1]]

	def check_point_reachability(self, com_pos_x, com_pos_y, com_pos_z, params, q_CoM=None):

		c_t = [com_pos_x, com_pos_y, com_pos_z]  # CoM to be tested
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

		stanceIndex = params.getStanceIndex(params.getStanceFeet())
		stanceIndex = np.array(stanceIndex, dtype=int)  # Ensure stance_index is an array of integers
		
		contactsBF = self.getcontactsBF(params, c_t)
		q = self.kin.inverse_kin(contactsBF, foot_vel, stanceIndex, q_CoM)
		q_to_check = np.concatenate([q[leg*3 : leg*3+3] for leg in stanceIndex]) # To check 3 or 4 feet stance

		out = self.kin.isOutOfJointLims(q_to_check, params.getJointLimsMax()[stanceIndex, :],
										params.getJointLimsMin()[stanceIndex, :])

		cxy_t = [c_t[0], c_t[1]]
		self.points.append(cxy_t)
		self.points_states.append(out)

	def compute_polygon(self, params, theta_step, dir_step):
		"""
		Compute projected Polytope.

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""

		comPosWF_0 = params.getCoMPosWF()
		

		# Check if current configuration is already feasible
		stanceIndex = params.getStanceIndex(params.getStanceFeet())
		contactsBF = self.getcontactsBF(params, comPosWF_0)
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
		if self.kin.isOutOfWorkSpace(contactsBF, params.getJointLimsMax(), params.getJointLimsMin(), stanceIndex, foot_vel):
			print ("Couldn't compute a reachable region! Current configuration is already out of joint limits!")
		else:
			# polygon = Polygon()
			theta = 0
			boundary_x = np.linalg.norm(params.getContactsPosWF()[0][0] - comPosWF_0[0]) * 1.75
			boundary_y = np.linalg.norm(params.getContactsPosWF()[0][1] - comPosWF_0[1]) * 1.75
			q_CoM = self.kin.get_current_q()

			for point_y in np.arange(comPosWF_0[0] - boundary_x, comPosWF_0[0] + boundary_x, 0.025):
				for point_x in np.arange(comPosWF_0[1] - boundary_y, comPosWF_0[1] + boundary_y, 0.015):
					self.check_point_reachability(point_x, point_y, comPosWF_0[2], params, q_CoM)
			# while theta < 360. * pi / 180.:
			# 	# print "theta: ", theta
			#
			# 	# Compute region for the current CoM position (in world frame)
			# 	self.compute_vertix(comPosWF_0[0], comPosWF_0[1], comPosWF_0[2], params, theta, dir_step)
			# 	print "points: ", self.points
			# 	print "points: ", self.points_states
			#
			# 	theta += theta_step  # increase search angle by step rad
			# 	print "theta: ", theta

		return self.points, self.points_states

	def project_polytope(self, params, theta_step=5. * pi / 180, dir_step=0.02):
		"""
		Project a polytope into a 2D polygon using the incremental projection
		algorithm from [Bretl08].

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""
		ip_start = time.time()
		polygon, polygon_states = self.compute_polygon(params, theta_step, dir_step)
		# polygon.sort_vertices()
		# vertices_list = polygon.export_vertices()
		# vertices_list = polygon
		# vertices = [array([v.x, v.y]) for v in vertices_list]
		compressed_vertices = np.compress([True, True], polygon, axis=1)
		# print compressed_vertices
		# try:
		# 	hull = ConvexHull(compressed_vertices)
		# except Exception as err:
		# 	print("QHull type error: " + str(err))
		# 	print("matrix to compute qhull:", compressed_vertices)
		# 	return False, False, False
		#
		# compressed_hull = compressed_vertices[hull.vertices]
		# compressed_hull = self.geom.clockwise_sort(compressed_hull)
		# vertices = compressed_hull
		# compressed_vertices = self.geom.clockwise_sort(compressed_vertices)
		# vertices = compressed_vertices
		computation_time = (time.time() - ip_start)

		# print "size: ", len(vertices)
		return compressed_vertices, polygon_states, computation_time
