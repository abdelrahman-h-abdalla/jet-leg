# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.leg_force_polytopes import LegForcePolytopes
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as Rot
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.constraints.friction_cone_constraint import FrictionConeConstraint
from numpy import matlib


class ForcePolytopeConstraint:
    def __init__(self, robot_kinematics):
        # self.robotName = robot_name
        self.kin = robot_kinematics
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.frictionConeConstr = FrictionConeConstraint()
        self.compGeom = ComputationalGeometry()

    def compute_actuation_constraints(self, contact_iterator, torque_limits, leg_self_weight, euler_angles, use_contact_torque, contact_torque_lims):

        jacobianMatrices, isOutOfWorkspace = self.kin.get_jacobians()
        # print(jacobianMatrices)
        if isOutOfWorkspace:
            C1 = np.zeros((0, 0))
            d1 = np.zeros((1, 0))
            current_actuation_polygons_WF = 0
            print('Out of workspace IK!!!')
        else:
            #            print 'Jacobians',jacobianMatrices
            current_actuation_polygons = self.computeActuationPolygons(jacobianMatrices, torque_limits, leg_self_weight)
            rot = Rot.from_euler('xyz', [euler_angles[0], euler_angles[1], euler_angles[2]], degrees=False)
            W_R_B = rot.as_matrix()
            current_actuation_polygons_WF = W_R_B.dot(current_actuation_polygons[contact_iterator])
            ''' in the case of the IP alg. the contact force limits must be divided by the mass
			because the gravito inertial wrench is normalized'''

            C1 = np.zeros((0, 0))
            d1 = np.zeros((1, 0))

            if not use_contact_torque:
                halfSpaceConstraints, offsetTerm = self.hexahedron(current_actuation_polygons_WF)
            else:
                hexahedronHalfSpaceConstraints, d = self.hexahedron(current_actuation_polygons_WF)
                wrench_term = np.array([[0, 0, 0, +1, 0],
                    [0, 0, 0, 0, +1],
                    [0, 0, 0, -1, 0],
                    [0, 0, 0, 0, -1]])
                force_term = np.hstack([hexahedronHalfSpaceConstraints, np.zeros((6,2))])
                halfSpaceConstraints = np.vstack([force_term, wrench_term])
                max_contact_torque = contact_torque_lims[1]
                min_contact_torque = contact_torque_lims[0]
                offsetTerm = np.vstack([d, max_contact_torque, max_contact_torque, -min_contact_torque, -min_contact_torque])

            C1 = block_diag(C1, halfSpaceConstraints)
            d1 = np.hstack([d1, offsetTerm.T])

            # print("H description: ",C1, d1)
            # print C1[0,0]
            # print "theta angles: "
            # for i in range(0,6):
            #    theta = np.arctan(C1[i,2]/C1[i,0])
            #    if (C1[i,2]<0):
            #        theta+=np.pi
            #    #print theta, "[rad] ", theta/np.pi*180, "[deg]"
            # print "V description: "
            # print actuation_polygons[contactIterator]

        return C1, d1, current_actuation_polygons_WF, isOutOfWorkspace

    def zonotope(self, dx=100, dy=100, dz=100):
        constraint = np.vstack([np.eye(3), -np.eye(3)])
        offsetTerm = np.array([[dx], [dy], [dz], [dx], [dy], [dz]])
        return constraint, offsetTerm

    def hexahedron(self, v_rep):

        h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6 = self.compGeom.get_hexahedron_halfspace_rep(v_rep)
        h_rep = np.vstack([h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6])

        if (h_rep[1, 3] > 0):
            h_rep = np.vstack([h_rep1, -h_rep2, -h_rep3, -h_rep4, -h_rep5, h_rep6])
            constraint = h_rep[:, 0:3]
            offsetTerm = np.vstack([[-h_rep1[3]],
                                    [h_rep2[3]],
                                    [h_rep3[3]],
                                    [h_rep4[3]],
                                    [h_rep5[3]],
                                    [-h_rep6[3]]])
        else:
            h_rep = np.vstack([-h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, -h_rep6])
            constraint = h_rep[:, 0:3]
            offsetTerm = np.vstack([[h_rep1[3]],
                                    [-h_rep2[3]],
                                    [-h_rep3[3]],
                                    [-h_rep4[3]],
                                    [-h_rep5[3]],
                                    [h_rep6[3]]])

            # print constraint, known_term
        return constraint, offsetTerm

    def computeActuationPolygons(self, legsJacobians, torque_limits, leg_self_weight):

        #        if np.sum(stanceFeet) == 4:
        #            print 'test', torque_limits[0]

        actuation_polygons = []
        for leg in range(len(legsJacobians)):
            actuation_polygons.append(self.computeLegActuationPolygon(
                                        legsJacobians[leg],torque_limits[leg], leg_self_weight[leg]))
        actuation_polygons = np.array(actuation_polygons)
        #        if np.sum(stanceFeet) == 3:
        ##            print 'test', torque_limits, stanceIndex
        #            actuation_polygons = np.array([self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[0])], torque_limits[int(stanceIndex[0])]),
        #                                       self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[1])], torque_limits[int(stanceIndex[1])]),
        #                                       self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[2])], torque_limits[int(stanceIndex[2])]),
        #                                        self.computeLegActuationPolygon(legsJacobians[int(swingIndex)], torque_limits[int(swingIndex)])])

        return actuation_polygons

    """ 
	This function computes the actuation polygon of a given mechanical chain (i.e. one leg).
	This function assumes the same mechanical structure of the HyQ robot, meaning that 
	it is restricted to 3 DoFs and point contacts. If the latter assumption is not
	respected the Jacobian matrix might become not invertible.
	"""

    def computeLegActuationPolygon(self, leg_jacobian, tau_lim, leg_self_weight):
        dx = tau_lim[0]
        dy = tau_lim[1]
        dz = tau_lim[2]

        #        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
        #                         [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
        #                         [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])

        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                             [-dy, dy, dy, -dy, -dy, dy, dy, -dy],
                             [-dz, -dz, -dz, -dz, dz, dz, dz, dz]])

        if (np.size(leg_jacobian, 0) == 2):
            torque_lims_xz = np.vstack([vertices[0, :], vertices[2, :]])

            actuation_polygon_xy = np.matmul(np.linalg.inv(np.transpose(leg_jacobian)), - torque_lims_xz)
            actuation_polygon = np.vstack([actuation_polygon_xy[0, :],
                                           vertices[1, :],
                                           actuation_polygon_xy[1, :]])

        elif (np.size(leg_jacobian, 0) == 3):
            leg_self_weight.shape = (3, 1)
            legs_gravity = np.matlib.repmat(leg_self_weight, 1, 8)

            actuation_polygon = np.matmul(np.linalg.pinv(np.transpose(leg_jacobian)), legs_gravity - vertices)

        # Only for debugging:
        # actuation_polygon = vertices
        return actuation_polygon
