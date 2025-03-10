# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 15:57:17 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pypoman
import numpy as np
from numpy import array, dot, eye, hstack, vstack, zeros
from scipy.spatial import ConvexHull
from jet_leg.constraints.constraints import Constraints
from jet_leg.kinematics.kinematics_interface import KinematicsInterface
from jet_leg.robots.robot_model_interface import RobotModelInterface
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.geometry import Geometry
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from cvxopt import matrix, solvers
import time


class ComputationalDynamics:
    def __init__(self, robot_name):
        self.robotName = robot_name
        self.geom = Geometry()
        self.math = Math()
        self.kin = KinematicsInterface(self.robotName)
        self.robotModel = RobotModelInterface(self.robotName)
        self.constr = Constraints(self.kin, self.robotModel)
        self.ineq = ([],[])
        self.eq = ([],[])
        self.rbd = RigidBodyDynamics()
        self.compGeom = ComputationalGeometry()

    ''' 
    This function computes the linear and angular inertial terms '''
    def compute_inertial_terms(self, robotMass, robotInertia,
                               acceleration_lin = [0., 0., 0.],
                               acceleration_ang = [0., 0., 0.],
                               velocity_ang = [[0.], [0.], [0.]]):

        linear_momentum_dot = np.multiply(robotMass, acceleration_lin)
        coriolis = robotInertia.dot(velocity_ang)
        coriolis = np.cross(np.transpose(velocity_ang), np.transpose(coriolis))
        coriolis = coriolis[0]
        angular_momentum_dot = np.matmul(robotInertia, acceleration_ang) + coriolis
        
        return linear_momentum_dot, angular_momentum_dot


    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the Iterative Projection algorithm 
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    comWF = position of the CoM
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    trunk_mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    mu = friction coefficient (we assume here the same coeff for all the contact points)
    saturate_normal_force = if True this sets a max constant value on the normal force of the friction cones
    '''
    def setup_iterative_projection(self, iterative_projection_params, saturate_normal_force):
        
        stanceLegs = iterative_projection_params.getStanceFeet()
        contactsWF = iterative_projection_params.getContactsPosWF()
        robotMass = iterative_projection_params.getTotalMass()
        robotInertia = iterative_projection_params.getTotalInertia()
        height = iterative_projection_params.comPositionWF[2]

        ''' parameters to be tuned'''
        g = 9.81
        contactsNumber = np.sum(stanceLegs)
        
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))
        G = np.zeros((6,0))

        extForce = iterative_projection_params.getExternalForce()
        extTorque = iterative_projection_params.getExternalCentroidalTorque()
        acceleration_lin = iterative_projection_params.getCoMLinAcc()
        acceleration_ang = iterative_projection_params.getCoMAngAcc()
        velocity_ang = iterative_projection_params.getCoMAngVel()

        linear_momentum_dot, angular_momentum_dot = self.compute_inertial_terms(robotMass,
                                                                               robotInertia,
                                                                               acceleration_lin,
                                                                               acceleration_ang)
        # linear_momentum_dot = [0., 0., 0.]
        # angular_momentum_dot = [0., 0., 0.]
        stanceIndex = iterative_projection_params.getStanceIndex(stanceLegs)

        for j in range(0,contactsNumber):

            # Get the 3D position r of contact point of stance leg j
            r = contactsWF[int(stanceIndex[j]),:]
            #print 'r is ', r

            # Build grasp matrix of the set of contact points
            # See equations 12 and 13 in Feasible Region paper
            # or equation 6 in Bretl

            # Get the transformation for forces (3D) or wrenches (6D)
            if not iterative_projection_params.useContactTorque:
                graspMatrix = self.math.getGraspMatrix(r)[:,0:3]  # forces (3D)
            else:
                graspMatrix = self.math.getGraspMatrix(r)[:,0:5]  # wrenches (6D)
            # print("grasp mat", graspMatrix)
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            # print "Ex", Ex
            # print "Ey", Ey
            G = hstack([G, graspMatrix])  # Full grasp matrix
            
#        print 'grasp matrix',G
        E = vstack((Ex, Ey)) / (g*robotMass - extForce[2] + linear_momentum_dot[2])
        # print "E", E
        f = np.array([(-extTorque[1] + angular_momentum_dot[1] - (extForce[0] - linear_momentum_dot[0]) * height) /
                      (g*robotMass - extForce[2] + linear_momentum_dot[2]),
                      (extTorque[0] - angular_momentum_dot[0] - (extForce[1] - linear_momentum_dot[1]) * height) /
                      (g*robotMass - extForce[2] + linear_momentum_dot[2])])
        proj = (E, f)  # y = E * x + f
        
        # see Equation (52) in "ZMP Support Areas for Multicontact..."
        A_f_and_tauz = array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]])
        A = dot(A_f_and_tauz, G)
        # Extension of equation 51 to add additional CoM constraint resulting from external forces and torques
        # Added instead of 0 block (upper right) in C_ext in Pypoman
        A_y = np.zeros((A_f_and_tauz.shape[0], 2))
        A_y[-1] = np.array([[extForce[1] - linear_momentum_dot[1], - extForce[0] + linear_momentum_dot[0]]])
#        print A
        t = hstack([- extForce[0] + linear_momentum_dot[0],
                    - extForce[1] + linear_momentum_dot[1],
                    g*robotMass - extForce[2] + linear_momentum_dot[2],
                    - extTorque[2] + angular_momentum_dot[2]])
#        print extForceWF, t
#        print 'mass ', robotMass
#        print A,t
        eq = (A, t)  # A * x == t

        C, d, isIKoutOfWorkSpace, actuation_polygons = self.constr.getInequalities(iterative_projection_params)
        ineq = (C, d)
        return proj, eq, A_y, ineq, actuation_polygons, isIKoutOfWorkSpace

    def setup_general_plane_iterative_projection(self, iterative_projection_params, saturate_normal_force):

        stanceLegs = iterative_projection_params.getStanceFeet()
        contactsWF = iterative_projection_params.getContactsPosWF()
        robotMass = iterative_projection_params.robotMass
        robotInertia = iterative_projection_params.getTotalInertia()
        g = 9.81
        contactsNumber = np.sum(stanceLegs)

        extForce = iterative_projection_params.getExternalForce()
        extTorque = iterative_projection_params.getExternalCentroidalTorque()
        acceleration_lin = iterative_projection_params.getCoMLinAcc()
        acceleration_ang = iterative_projection_params.getCoMAngAcc()
        velocity_ang = iterative_projection_params.getCoMAngVel()

        linear_momentum_dot, angular_momentum_dot = self.compute_inertial_terms(robotMass,
                                                                               robotInertia,
                                                                               acceleration_lin,
                                                                               acceleration_ang)
        # linear_momentum_dot = robotMass*acceleration
        # linear_momentum_dot = np.array([0., 0., 0.])
        # linear_momentum_dot = iterative_projection_params. getInertialForces()
        # angular_momentum_dot = np.array([0., 0., 0.])
        # angular_momentum_dot = iterative_projection_params.getInertialMoments()
        plane_normal = iterative_projection_params.get_plane_normal()
        projection_plane_z_intercept = iterative_projection_params.get_CoM_plane_z_intercept()
        stanceIndex = iterative_projection_params.getStanceIndex(stanceLegs)

        G = np.zeros((6, 0))

        for j in range(0,contactsNumber):

            # Get the 3D position r of contact point of stance leg j
            r = contactsWF[int(stanceIndex[j]),:]

            # Build grasp matrix of the set of contact points
            # See equations 12 and 13 in Feasible Region paper
            # or equation 6 in Bretl

            # Get the transformation for forces (3D) or wrenches (6D)
            if not iterative_projection_params.useContactTorque:
                graspMatrix = self.math.getGraspMatrix(r)[:, 0:3]  # forces (3D)
            else:
                graspMatrix = self.math.getGraspMatrix(r)[:, 0:5]  # wrenches (6D)
            # print "grasp mat", graspMatrix
            G = hstack([G, graspMatrix])  # Full grasp matrix

        A21_zeros = np.zeros((3,2))
        A22 = self.compute_A22_block(g*robotMass, extForce, linear_momentum_dot, plane_normal)
        A = np.hstack((G, np.vstack((A21_zeros, A22))))
        t_linear = np.add(np.add(-np.array([0.0, 0.0, -g*robotMass]), linear_momentum_dot), -np.array(extForce))
        t_angular = self.compute_t_angular_vector(projection_plane_z_intercept, extForce, extTorque,
                                                  linear_momentum_dot, angular_momentum_dot)
        t = np.concatenate([t_linear, t_angular])

        eq = (A, t)  # A * x == t

        C, d, isIKoutOfWorkSpace, actuation_polygons = self.constr.getInequalities(iterative_projection_params)
        ineq = (C, d)

        return eq, ineq, actuation_polygons, isIKoutOfWorkSpace


    def compute_A22_block(self, gravity_term, ext_force, linear_momentum_dot, plane_normal):

        A22 = np.zeros((3,2))

        A22[0][0] = plane_normal[0]/plane_normal[2]*ext_force[1] + \
                    - plane_normal[0]/plane_normal[2]*linear_momentum_dot[1]
        A22[0][1] = - gravity_term + plane_normal[1]/plane_normal[2]*ext_force[1] + \
                    ext_force[2] - plane_normal[1]/plane_normal[2]*linear_momentum_dot[1] - linear_momentum_dot[2]
        A22[1][0] = -plane_normal[0]/plane_normal[2]*ext_force[0] - ext_force[2] + \
                    plane_normal[0]/plane_normal[2]*linear_momentum_dot[0] + linear_momentum_dot[2] + gravity_term
        A22[1][1] = - plane_normal[1]/plane_normal[2]*ext_force[0] + \
                    plane_normal[1]/plane_normal[2]*linear_momentum_dot[0]
        A22[2][0] = ext_force[1] - linear_momentum_dot[1]
        A22[2][1] = -ext_force[0] + linear_momentum_dot[0]

        return A22

    def compute_t_angular_vector(self, d_intercept, ext_force, ext_torque, linear_momentum_dot, angular_momentum_dot):

        return np.array([ext_force[1]*d_intercept - linear_momentum_dot[1]*d_intercept,
                         -ext_force[0]*d_intercept + linear_momentum_dot[0]*d_intercept, 0]) + \
               angular_momentum_dot - ext_torque

    def reorganizeActuationPolytopes(self, actPolytope):
        outputPolytopeX = np.zeros((1,8))
        outputPolytopeY = np.zeros((1,8))
        outputPolytopeZ = np.zeros((1,8))
        outputPolytope = np.array([[outputPolytopeX], [outputPolytopeY], [outputPolytopeZ]])
#        print 'out poly',outputPolytope[1]
#        print 'out poly X',outputPolytopeX
        for i in range(0,8):
#            print i
            if(actPolytope[0][i]>0) and (actPolytope[1][i]<0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][0] = actPolytope[0][i]
                outputPolytopeY[0][0] = actPolytope[1][i]
                outputPolytopeZ[0][0] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]>-10.) and (actPolytope[2][i]<0):
                outputPolytopeX[0][1] = actPolytope[0][i]
                outputPolytopeY[0][1] = actPolytope[1][i]
                outputPolytopeZ[0][1] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]>0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][2] = actPolytope[0][i]
                outputPolytopeY[0][2] = actPolytope[1][i]
                outputPolytopeZ[0][2] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]<0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][3] = actPolytope[0][i]
                outputPolytopeY[0][3] = actPolytope[1][i]
                outputPolytopeZ[0][3] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]<0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][4] = actPolytope[0][i]
                outputPolytopeY[0][4] = actPolytope[1][i]
                outputPolytopeZ[0][4] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]>0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][5] = actPolytope[0][i]
                outputPolytopeY[0][5] = actPolytope[1][i]
                outputPolytopeZ[0][5] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]>0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][6] = actPolytope[0][i]
                outputPolytopeY[0][6] = actPolytope[1][i]
                outputPolytopeZ[0][6] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]<10.) and (actPolytope[2][i]>0):
                outputPolytopeX[0][7] = actPolytope[0][i]
                outputPolytopeY[0][7] = actPolytope[1][i]
                outputPolytopeZ[0][7] = actPolytope[2][i]
#        outputPolytope = actPolytope
        
        outputPolytope = np.array([[outputPolytopeX], [outputPolytopeY], [outputPolytopeZ]])
#        print 'out poly',outputPolytope
#        print 'input ', actPolytope
#        print 'out poly X',outputPolytopeX
#        print 'out poly Y',outputPolytopeY
#        print 'out poly Z',outputPolytopeZ
        return outputPolytope

    def fill_general_plane_region_z_component(self, vertices, plane_normal, plane_z_intercept):

        vertices_3d = np.ndarray(shape=(vertices.shape[0], 3), dtype=float)
        for i, vertix in enumerate(vertices):
            vertices_3d[i] = np.append(vertices[i], self.math.compute_z_component_of_plane(vertix, plane_normal, plane_z_intercept))

        return vertices_3d


    def try_iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):
        try:
            compressed_hull, actuation_polygons, computation_time = self.iterative_projection_bretl(iterative_projection_params, saturate_normal_force)
            return compressed_hull, actuation_polygons, computation_time
        except ValueError as err:
            print('Could not compute the feasible region')
            print((err.args))
            return False, False, False


    def iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):

        start_t_IP = time.time()

        # Stop if no feet on the ground
        stanceLegs = iterative_projection_params.getStanceFeet()
        stanceIndex = iterative_projection_params.getStanceIndex(stanceLegs)
        if len(stanceIndex) < 1:
            return False, False, False

        # if iterative_projection_params.plane_normal == [0,0,1]:
        if False:
            proj, self.eq, self.A_y, self.ineq, actuation_polygons, isIKoutOfWorkSpace = self.setup_iterative_projection(iterative_projection_params, saturate_normal_force)
        else:
            self.eq, self.ineq, actuation_polygons, isIKoutOfWorkSpace = self.setup_general_plane_iterative_projection(iterative_projection_params, saturate_normal_force)

        if isIKoutOfWorkSpace:
            return False, False, False
        else:
            try:
                # if iterative_projection_params.plane_normal == [0,0,1]:
                if False:
                    vertices_WF = pypoman.project_polytope(proj, self.ineq, self.A_y, self.eq, method='bretl', max_iter=500, init_angle=0.0)
                else:
                    vertices_WF = pypoman.project_polytope_general_plane(self.ineq, self.eq, max_iter=500, init_angle=0.0)
            except (ValueError, Exception) as e:
                vertices_WF = False
                if hasattr(e, 'message'):
                    print((e.message))
                else:
                    print(e)

            if vertices_WF is False:
                print('Project polytope function is False')
                return False, False, False

            else:
                compressed_vertices = np.compress([True, True], vertices_WF, axis=1)
                try:
                    hull = ConvexHull(compressed_vertices)
                except Exception as err:
                    print(("QHull type error: " + str(err)))
                    print(("matrix to compute qhull:",compressed_vertices))
                    return False, False, False
                
                
#        print 'hull ', hull.vertices
            compressed_hull = compressed_vertices[hull.vertices]
            compressed_hull = self.geom.clockwise_sort(compressed_hull)
            # USE THIS TO COMPUTE THE REGION AT THE COM LEVEL
            compressed_hull = self.fill_general_plane_region_z_component(compressed_hull,
                                                                         iterative_projection_params.get_plane_normal(),
                                                                         iterative_projection_params.get_CoM_plane_z_intercept())
#            # USE THIS TO COMPUTE THE REGION AT THE FEET LEVEL
#            compressed_hull = self.fill_general_plane_region_z_component(compressed_hull,
#                                                                         iterative_projection_params.get_plane_normal(),
#                                                                         iterative_projection_params.get_terrain_plane_z_intercept())                                                                         
            compressed_hull = compressed_hull
            computation_time = (time.time() - start_t_IP)

        return compressed_hull, actuation_polygons, computation_time
        
        
    def instantaneous_actuation_region_bretl(self, stanceLegs, contacts, normals, total_mass, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_ACTUATION'
        number_of_generators = 4
        mu = 1.0
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, total_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
        return IP_points, actuation_polygons, computation_time

    def support_region_bretl(self, stanceLegs, contacts, normals, total_mass, number_of_generators = 4, mu = 1.0, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_FRICTION'
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, total_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
        return IP_points, actuation_polygons, computation_time

    def check_equilibrium(self, LPparams, useVariableJacobian = False, verbose = False):

        p, G, h, A, b, isIKoutOfWorkSpace, LP_actuation_polygons = self.setup_lp(LPparams)

        if isIKoutOfWorkSpace:
            #unfeasible_points = np.vstack([unfeasible_points, com_WF])
            print('something is wrong in the inequalities or the point is out of workspace')
            x = -1
            return False, x, LP_actuation_polygons
        else:
            # print('Solving LP')
            sol = solvers.lp(p, G, h, A, b)
            x = sol['x']
            status = sol['status']
            if x == None:
                isConfigurationStable = False
            else:
                isConfigurationStable = True
            return isConfigurationStable, x, LP_actuation_polygons
                
    ''' 
    This function is used to check every single CoM position and see if there is a feasible set of contact forces for that configuration.
    This function can consider either friction constraints or actuation constraints separately (depending on the value of the constraint_mode variable).
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    '''
    def LP_projection(self, LPparams, useVariableJacobian = False, stepX = 0.05, stepY = 0.05, stepZ = 0.05):
        start_t_LP = time.time()
        stanceLegs = LPparams.getStanceFeet()
        nc = np.sum(stanceLegs)
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,nc*3))  
        verbose = False
        com_WF = np.array([0.0, 0.0, 0.0])
#        default_com_WF = com_WF
#        p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        
        """ Defining the equality constraints """
        for com_x in np.arange(-0.25,0.25,stepX):
            for com_y in np.arange(-0.2,0.2,stepY):
                for com_z in np.arange(-0.2,0.25,stepZ):
                    com_WF = np.array([com_x, com_y, com_z])
                    #LPparams.setCoMPosWF(com_WF)
                    status, x, force_polytopes = self.check_equilibrium(LPparams)
        #            if useVariableJacobian:
        #                p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        #            else:
        #                p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        #
        #            if not isConstraintOk:
        #                unfeasible_points = np.vstack([unfeasible_points, com_WF])
        #                if verbose:
        #                    print 'something is wrong in the inequalities or the point is out of workspace'
        #            else:
        #                sol=solvers.lp(p, G, h, A, b)
        #                x = sol['x']
        #                status = sol['status']
                        #print x
                    if status == 'optimal':
                        feasible_points = np.vstack([feasible_points, com_WF])
                        contact_forces = np.vstack([contact_forces, np.transpose(x)])
                    else:
                        unfeasible_points = np.vstack([unfeasible_points, com_WF])

                    
        print(("LP test: --- %s seconds ---" % (time.time() - start_t_LP)))
        
        return feasible_points, unfeasible_points, contact_forces

    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the LP ground truth. 
    
    The arguments are: 
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    contacts = location of the nc contacts
    numberOfGenerators = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    comWorldFrame = position of the CoM
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    '''
    def setup_lp(self, LPparams):

        g = 9.81

        #        trunk_mass = LPparams.getTrunkMass()
        stanceLegs = LPparams.getStanceFeet()
        totMass = LPparams.robotMass
        nc = np.sum(stanceLegs)
        grav = np.array([[0.], [0.], [-g*totMass]])
        if LPparams.useContactTorque:
            p = matrix(np.zeros((5*nc,1)))
        else:
            p = matrix(np.zeros((3*nc,1)))

        contactsPosWF = LPparams.getContactsPosWF()
        comWorldFrame = LPparams.getCoMPosWF()
        extForce = LPparams.getExternalForce()
        extTorque = LPparams.getExternalCentroidalTorque()
        totForce = grav
        totForce[0] += extForce[0]
        totForce[1] += extForce[1]
        totForce[2] += extForce[2]
        totalCentroidalWrench = self.rbd.computeCentroidalWrench(LPparams.robotMass,
                                                                 LPparams.robotInertia,
                                                                 LPparams.getCoMPosWF(),
                                                                 LPparams.externalCentroidalWrench,
                                                                 LPparams.getCoMLinAcc(),
                                                                 LPparams.getCoMAngAcc())

        if np.sum(stanceLegs) == 1:
            A = np.zeros((5,0))
        else:
            A = np.zeros((6,0))
        stanceIndex = LPparams.getStanceIndex(stanceLegs)
        for j in stanceIndex:
            j = int(j)
            # print 'index in lp ',j
            r = contactsPosWF[j, :]
            GraspMat = self.math.getGraspMatrix(r)
            if not LPparams.useContactTorque:
                A = np.hstack((A, GraspMat[:, 0:3]))
                # print(A)
                A = matrix(A)
                b = matrix(totalCentroidalWrench.reshape((6)))
            else:
                ''' non-zero foot size case'''
                if np.sum(stanceLegs) == 1:
                    print ("ciaooooo")
                    ''' if there is only one stance foot the problem is overconstrained and we can remove the constraint on tau_z'''
                    A = np.hstack((A, GraspMat[0:5,0:5]))
                    A = matrix(A)
                    totW = totalCentroidalWrench[0:5]
                    b = matrix(totW.reshape((5)))
                else:
                    A = np.hstack((A, GraspMat[:,0:5]))
                    A = matrix(A)
                    b = matrix(totalCentroidalWrench.reshape((6)))


        G, h, isIKoutOfWorkSpace, LP_actuation_polygons = self.constr.getInequalities(LPparams)
        G = matrix(G)
        h = matrix(h)

        lp = p, G, h, A, b
        return p, G, h, A, b, isIKoutOfWorkSpace, LP_actuation_polygons

    def compute_IP_margin(self, iterative_projection_params, reference_type, saturate_normal_force = False):
        ''' compute iterative projection
        Outputs of "iterative_projection_bretl" are:
        IP_points = resulting 2D vertices
        actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
        computation_time = how long it took to compute the iterative projection
        '''
        IP_points, force_polytopes, IP_computation_time = self.try_iterative_projection_bretl(iterative_projection_params)
        if IP_points is not False:
            facets = self.compGeom.compute_halfspaces_convex_hull(IP_points[:,:2])
            comWF = iterative_projection_params.getCoMPosWF()
            reference_point = np.array([comWF[0], comWF[1]])
            isPointFeasible, margin = self.compGeom.isPointRedundant(facets, reference_point)
            return isPointFeasible, margin
        else:
            print("Warning! IP failed.")
            return False, -0.15
    
    def getReferencePointWrtBaseFrame(self, iterative_projection_params):
        
        pendulum_height = 100.0
        iterative_projection_params.setCoMPosWF([0.0, 0.0, pendulum_height])
        comWF = iterative_projection_params.getCoMPosWF()
        if(iterative_projection_params.useInstantaneousCapturePoint):
            ICP = self.icp.compute(iterative_projection_params)
            iterative_projection_params.instantaneousCapturePoint = ICP
            referencePoint = np.array([ICP[0], ICP[1]])
        else:
            referencePoint = np.array([comWF[0], comWF[1]])

        return referencePoint