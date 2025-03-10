# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Abdelrahman Abdalla
"""

import numpy as np
import time

from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.kinematics import kinematics_interface
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.kinematics import full_feasible_workspace
from jet_leg.optimization import nonlinear_projection
from shapely.geometry import Polygon
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

plt.close('all')

robot_name = 'hyqreal'
full_projection = full_feasible_workspace.FeasibleWorkspace(robot_name)
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

math = Math()


# comWF = np.array([.55, 0.005, 0.5])  # pos of COM in world frame
comWF = np.array([0.0107, 0.0003, 0.49])  # pos of COM in world frame w.o. trunk controller
# comWF = np.array([0.009, 0.0001, 0.549])  # pos of COM in world frame w. trunk controller
comBF = np.array([0.0094,  0.0002, -0.0433])  # pos of COM in body frame w.o. trunk controller
# comBF = np.array([-0.00, 0.000, -0.0])
# comBF = np.array([0.0094, 0.0002, -0.0458])  # pos of COM in body frame w. trunk controller
rpy = np.array([0.00012, 0.00601, 3.6e-05])  # orientation of body frame w.o. trunk controller
# rpy = np.array([0.00001589, -0.00000726, -0.00000854])  # orientation of body frame w. trunk controller

""" contact points in the World Frame"""
# LF_foot = np.array([0.9, 0.3, 0.02])
# RF_foot = np.array([1., -0.3, 0.02])
# LH_foot = np.array([0.1, 0.3, 0.02])
# RH_foot = np.array([0.2, -0.3, 0.02])
# HyQ
# LF_foot = np.array([0.36, 0.32, 0.02])  # Starting configuration w.o. trunk controller
# RF_foot = np.array([0.36, -0.32, 0.02])
# LH_foot = np.array([-0.36, 0.32, 0.02])
# RH_foot = np.array([-0.36, -0.32, 0.02])
# HyQReal
LF_foot = np.array([0.44, 0.34, 0.02])  # Starting configuration w.o. trunk controller
RF_foot = np.array([0.44, -0.34, 0.02])
LH_foot = np.array([-0.44, 0.34, 0.02])
RH_foot = np.array([-0.44, -0.34, 0.02])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' stanceFeet vector contains 1 if the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

tripleStance = False
randomSwingLeg = random.randint(0, 3)  # if you want you can define a swing leg using this variable

''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
# HyQ
# LF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
# LF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
# RF_q_lim_max = [0.44, 1.2217, -0.3491]  # HAA, HFE, KFE
# RF_q_lim_min = [-1.22, -0.8727, -2.4435]  # HAA, HFE, KFE
# LH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, HFE, KFE
# LH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
# RH_q_lim_max = [0.44, 0.8727, 2.4435]  # HAA, H FE, KFE
# RH_q_lim_min = [-1.22, -1.2217, 0.3491]  # HAA, HFE, KFE
# HyQReal
LF_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
LF_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
RF_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
RF_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
LH_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
LH_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
RH_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
RH_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''

params = IterativeProjectionParameters(robot_name)
params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)
params.setActiveContacts(stanceFeet)

''' Compute FULL workspace'''
# polygon, computation_time = projection.project_polytope(params, 20. * np.pi / 180, 0.03)
full_polygon, full_polygon_states, computation_time = full_projection.project_polytope(params)
print ("vertices", full_polygon)
print ("Computation Time: ", computation_time, " seconds")

''' Compute reachable region'''
com_check = np.array([0.3, -0.1, 0.5])
com_check = None
# polygon, computation_time = projection.project_polytope(params, com_check)
polygon, computation_time = projection.project_polytope(params, com_check, 10. * np.pi / 180, 0.01)
final = polygon
print("Computation time: ", computation_time)

plt.close()
h1 = plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plotter = Plotter()
plt.legend()

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)

''' Plot full workspace'''
for i in range(0, len(full_polygon)):
    if full_polygon_states[i] == True:
        h2 = plt.plot(full_polygon[i][0], full_polygon[i][1], 'ro', markersize=5.5)
    else:
        h2 = plt.plot(full_polygon[i][0], full_polygon[i][1], 'go', markersize=5.5)

'''' Plot reachable region'''
h6 = plotter.plot_polygon(np.transpose(polygon), '--k', 5, 'reachable region')

for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15)

handles, labels = plt.gca().get_legend_handles_labels()

green_circle = Line2D(range(1), range(1), linestyle='none', color="green", marker='o', markersize=7, label='CoM workspace')
red_circle = Line2D(range(1), range(1), linestyle='none', color="red", marker='o', markersize=7, label='unreachable CoM position')
handles.extend([green_circle,red_circle])
plt.legend(handles=handles)
#
# plt.legend()
plt.show()
