# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Abdelrahman Abdalla
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from numpy import array, cos, sin, cross, pi
from scipy.linalg import norm
from scipy.spatial import ConvexHull
from jet_leg.computational_geometry.geometry import Geometry
import time
import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.kinematics.kinematics_interface import KinematicsInterface


class Vertex:

	def __init__(self, p):
		self.x = p[0]
		self.y = p[1]


class Polygon:

	def from_vertices(self, v1, v2, v3):
		v1.next = v2
		v2.next = v3
		v3.next = v1
		self.vertices = [v1, v2, v3]

	def sort_vertices(self):
		"""
		Export vertices starting from the left-most and going clockwise.
		Assumes all vertices are on the positive halfplane.
		"""
		minsd = 1e10
		ibottom = 0
		for i in range(len(self.vertices)):
			v = self.vertices[i]
			if (v.y + v.next.y) < minsd:
				ibottom = i
				minsd = v.y + v.next.y
		for v in self.vertices:
			v.checked = False
		vcur = self.vertices[ibottom]
		newvertices = []
		while not vcur.checked:
			vcur.checked = True
			newvertices.append(vcur)
			vcur = vcur.__next__
		newvertices.reverse()
		vfirst = newvertices.pop(-1)
		newvertices.insert(0, vfirst)
		self.vertices = newvertices

	def export_vertices(self, threshold=1e-2):
		export_list = [self.vertices[0]]
		for i in range(1, len(self.vertices) - 1):
			vcur = self.vertices[i]
			vlast = export_list[-1]
			if norm([vcur.x - vlast.x, vcur.y - vlast.y]) > threshold:
				export_list.append(vcur)
		export_list.append(self.vertices[-1])  # always add last vertex
		return export_list


class NonlinearProjectionBretl:
	def __init__(self, robot_name):
		self.robotName = robot_name
		self.kin = KinematicsInterface(self.robotName)
		self.geom = Geometry()
		self.math = Math()

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

	def compute_vertix(self, com_pos_x, com_pos_y, plane_normal, CoM_plane_z_intercept, params, q_0, theta, min_dir_step, max_iter):
		"""
		Compute vertix of projected polygon in vdir direction.

		Solves nonlinear optimization problem by iteratively testing
		over beta-spaced possible CoM positions along vdir direction

		Returns
		-------
		poly: Polygon Output polygon.
		"""

		# Search for a CoM position in direction vdir

		vdir = array([cos(theta), sin(theta)])

		c_t_xy = [com_pos_x, com_pos_y]  # CoM to be tested
		cxy_opt = c_t_xy  # Optimal CoM so far
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
		stanceIndex = params.getStanceIndex(params.getStanceFeet())

		i = 0

		# Find closest feasible and non-feasible points to start bisection algorithm

		# Initial step is large to speed up initial search
		# However, region is not convex so too large of a step can be non-conservative and skip close vertices
		dir_step = 0.1
		c_t_feasible = True  # Flag for out of limits search

		while c_t_feasible and i < max_iter:

			c_t_xy += dir_step * vdir  # Iterate along direction vector by step cm
			z_coordinate = self.math.compute_z_component_of_plane(c_t_xy, plane_normal, CoM_plane_z_intercept)
			c_t = np.append(c_t_xy, z_coordinate)
			contactsBF = self.getcontactsBF(params, c_t)
			stanceIndex = np.array(stanceIndex, dtype=int)  # Ensure stance_index is an array of integers
			q = self.kin.inverse_kin(contactsBF, foot_vel, stanceIndex, q_0)
			q_to_check = np.concatenate([q[leg*3 : leg*3+3] for leg in stanceIndex]) # To check 3 or 4 feet stance

			# if (not self.kin.hyqreal_ik_success) or self.kin.isOutOfJointLims(q, params.getJointLimsMax(), params.getJointLimsMin()):
			if (not self.kin.ik_success) or \
					self.kin.isOutOfJointLims(q_to_check, params.getJointLimsMax()[stanceIndex,:],
											  params.getJointLimsMin()[stanceIndex,:]): # kin.hyqreal_ik_success is always true for Hyq
				c_t_feasible = False
			else:
				cxy_opt = [c_t[0], c_t[1]]
				dir_step += dir_step / 2
				q_0 = q

			i += 1

		# Perform bisection algorithm using two points from previous step
		# Switch direction to go back to feasible region
		dir_step = -dir_step / 2
		while abs(dir_step) >= min_dir_step and i < max_iter:

			old_c_t_feasible = c_t_feasible
			c_t_xy += dir_step * vdir
			z_coordinate = self.math.compute_z_component_of_plane(c_t_xy, plane_normal, CoM_plane_z_intercept)
			c_t = np.append(c_t_xy, z_coordinate)
			contactsBF = self.getcontactsBF(params, c_t)
			q = self.kin.inverse_kin(contactsBF, foot_vel, stanceIndex, q_0)
			q_to_check = np.concatenate([list(q[leg * 3: leg * 3 + 3]) for leg in stanceIndex]) # To check 3 or 4 feet stance

			# If new point is on the same side (feasible or infeasible region) as last point, continue in same direction
			# if self.kin.hyqreal_ik_success and (not self.kin.isOutOfJointLims(q, params.getJointLimsMax(), params.getJointLimsMin())):
			if self.kin.ik_success and \
					(not self.kin.isOutOfJointLims(q_to_check, params.getJointLimsMax()[stanceIndex,:],
												   params.getJointLimsMin()[stanceIndex,:])):
				c_t_feasible = True
				cxy_opt = [c_t[0], c_t[1]]
				q_0 = q
			else:
				c_t_feasible = False

			if c_t_feasible == old_c_t_feasible:
				dir_step = dir_step/2
			else:
				dir_step = -dir_step / 2

			i += 1
		# print "new dir_step: ", dir_step

		return cxy_opt

	def compute_polygon(self, params, q_CoM, theta_step, dir_step, max_iter):
		"""
		Compute projected Polytope.

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""

		comPosWF_0 = params.getCoMPosWF()

		plane_normal = params.get_plane_normal()
		CoM_plane_z_intercept = params.get_CoM_plane_z_intercept()
		polygon = []

		# polygon = Polygon()
		theta = 0

		while theta < 360. * pi /180.:
			# print "theta: ", theta

			# Compute region for the current CoM position (in world frame)
			v = Vertex(self.compute_vertix(comPosWF_0[0], comPosWF_0[1], plane_normal, CoM_plane_z_intercept, params, q_CoM, theta, dir_step, max_iter))
			polygon.append(v)

			theta += theta_step  # increase search angle by step rad

		return polygon

	def fill_general_plane_region_z_component(self, vertices, plane_normal, plane_z_intercept):

		vertices_3d = np.ndarray(shape=(vertices.shape[0], 3), dtype=float)
		for i, vertix in enumerate(vertices):
			vertices_3d[i] = np.append(vertices[i],
									   self.math.compute_z_component_of_plane(vertix, plane_normal, plane_z_intercept))

		return vertices_3d

	def project_polytope(self, params, com_wf_check=None, theta_step=20. * pi / 180, dir_step=0.03, max_iter=1000):
		"""
		Project a polytope into a 2D polygon using the incremental projection
		algorithm from [Bretl08].

		Returns
		-------
		vertices: list of arrays List of vertices of the
				projected polygon.
		"""

		ip_start = time.time()

		# Check if current configuration is already feasible
		stanceIndex = params.getStanceIndex(params.getStanceFeet())
		contactsBF = self.getcontactsBF(params, params.getCoMPosWF())
		foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

		if self.kin.isOutOfWorkSpace(contactsBF, params.getJointLimsMax(), params.getJointLimsMin(), stanceIndex, foot_vel):
			print("Couldn't compute a reachable region! Current configuration is already out of joint limits!")
			return np.array([]), (time.time() - ip_start)

		if com_wf_check is not None:

			contactsBF_check = self.getcontactsBF(params, com_wf_check)
			if self.kin.isOutOfWorkSpace(contactsBF_check, params.getJointLimsMax(), params.getJointLimsMin(), stanceIndex, foot_vel):
				print("Ouch!")
				return np.array([]), (time.time() - ip_start)

		# Compute region
		q_CoM = self.kin.get_current_q()
		polygon = self.compute_polygon(params, q_CoM, theta_step, dir_step, max_iter)
		# polygon.sort_vertices()
		# vertices_list = polygon.export_vertices()
		vertices_list = polygon

		if not vertices_list:
			return np.array([]), (time.time() - ip_start)
		else:
			vertices = [array([v.x, v.y]) for v in vertices_list]
			compressed_vertices = np.compress([True, True], vertices, axis=1)
# USE THIS TO COMPUTE THE REGION AT THE COM LEVEL
			compressed_vertices = self.fill_general_plane_region_z_component(compressed_vertices,
													   params.get_plane_normal(),
													   params.get_CoM_plane_z_intercept())
# USE THIS TO COMPUTE THE REGION AT THE FEET LEVEL
#			compressed_vertices = self.fill_general_plane_region_z_component(compressed_vertices,
#													   params.get_plane_normal(),
#													   params.get_terrain_plane_z_intercept())                            
                
			vertices = compressed_vertices
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
		computation_time = (time.time() - ip_start)

		# print "size: ", len(vertices)
		#print "Done!"
		return vertices, computation_time
