# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import, division, print_function

import numpy as np
import copy
import matplotlib.pyplot as plt
import random

from scipy.spatial.transform import Rotation as Rot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from jet_leg.plotting.plotting_tools import Plotter
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization import nonlinear_projection
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.plotting.arrow3D import Arrow3D

def initialize_robot_parameters():
    robot_name = 'hyqreal'
    ng = 4
    constraint_mode_IP = ['FRICTION_AND_ACTUATION'] * 4
    mu = 0.7
    stanceFeet = [1, 1, 1, 1]
    return robot_name, ng, constraint_mode_IP, mu, stanceFeet

def setup_contact_points():
    LF_foot = np.array([ 0.36,  0.21 , -0.47])
    RF_foot = np.array([ 0.36, -0.21 , -0.47])
    LH_foot = np.array([-0.36,  0.21 , -0.47])
    RH_foot = np.array([-0.36, -0.21 , -0.47])
    contactsBF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
    return contactsBF

def transform_contacts_to_world_frame(contactsBF, comWF, rpy_base):
    rot = Rot.from_euler('xyz', rpy_base, degrees=False)
    W_R_B = rot.as_matrix()
    contactsWF = copy.copy(contactsBF)
    for j in range(contactsBF.shape[0]):
        contactsWF[j, :] = np.dot(W_R_B, contactsBF[j, :]) + comWF
    return contactsWF, W_R_B

def compute_normals():
    math = Math()
    axisZ = np.array([[0.0], [0.0], [1.0]])
    normals = np.vstack([
        math.rpyToRot(0.0, 0.0, 0.0).dot(axisZ).flatten() for _ in range(4)
    ])
    return normals

def setup_parameters(robot_name, contactsWF,
                     comWF, comBF, rpy_base,
                     normals, mu,
                     stanceFeet, feas_projector,
                     ext_force, ext_torque):
    params = IterativeProjectionParameters(robot_name)
    params.setContactsPosWF(contactsWF)
    params.setCoMPosWF(comWF)
    params.setCoMPosBF(comBF)
    params.setOrientation(rpy_base)
    params.setTorqueLims(feas_projector.robotModel.robotModel.joint_torque_limits)
    params.setJointLimsMax(feas_projector.robotModel.robotModel.joint_limits_max)
    params.setJointLimsMin(feas_projector.robotModel.robotModel.joint_limits_min)
    params.externalForce = ext_force  # params.externalForceWF is not used internally yet
    params.externalCentroidalTorque = ext_torque
    params.setActiveContacts(stanceFeet)
    params.setConstraintModes(['FRICTION_AND_ACTUATION'] * 4)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(4)
    params.setTotalMass(feas_projector.robotModel.robotModel.trunkMass)
    return params

def compute_projections(feas_projector, reach_projector, params, comWF):
    # Stability (torque) polygon
    stab_torque_polygon, _, _ = feas_projector.iterative_projection_bretl(params)
    # Reach polygon
    reach_polygon, _ = reach_projector.project_polytope(params, comWF, 
                                                        20.0 * np.pi / 180, 
                                                        0.03)
    return stab_torque_polygon, reach_polygon

def check_com_stability(feas_projector, params):
    isCoMStable, _, _ = feas_projector.check_equilibrium(params)
    return isCoMStable

def plot_3d_and_top_view(contactsWF, normals, isCoMStable, comWF,
                         stab_torque_polygon, reach_polygon):
    """
    Plots both a 3D view and a 2D top view of the contact points, CoM,
    and stability/reach polygons.
    """

    # Helper function: convert 2D polygon => 3D polygon at z=comWF[2]
    def polygon_to_3d(poly_2d, z_level):
        if poly_2d.shape[1] == 3:  # Already 3D
            poly_3d = poly_2d.copy()
            poly_3d[:, 2] = z_level  # Update z-level
        else:  # Convert 2D to 3D
            poly_3d = np.zeros((poly_2d.shape[0], 3))
            poly_3d[:, :2] = poly_2d  # Copy X, Y
            poly_3d[:, 2] = z_level   # Set Z
        return poly_3d

    # Convert the stability and reach polygons into 3D at the CoM height
    stab_polygon_3d = polygon_to_3d(stab_torque_polygon, comWF[2])
    reach_polygon_3d = polygon_to_3d(reach_polygon, comWF[2])

    # 3D Plot
    fig = plt.figure(figsize=(12, 6))
    ax_3d = fig.add_subplot(121, projection='3d')

    # Plot contact points (black)
    ax_3d.scatter(
        contactsWF[:, 0],
        contactsWF[:, 1],
        contactsWF[:, 2],
        c='k', s=100, label='Contact Points'
    )

    # Plot CoM (green if stable, red if not)
    color_com = 'g' if isCoMStable else 'r'
    ax_3d.scatter(
        comWF[0],
        comWF[1],
        comWF[2],
        c=color_com,
        s=100,
        label='CoM'
    )

    # Draw 3D arrows for contact normals
    for j in range(contactsWF.shape[0]):
        arrow = Arrow3D(
            [contactsWF[j, 0], contactsWF[j, 0] + normals[j, 0] / 5],
            [contactsWF[j, 1], contactsWF[j, 1] + normals[j, 1] / 5],
            [contactsWF[j, 2], contactsWF[j, 2] + normals[j, 2] / 5],
            mutation_scale=20, lw=2, arrowstyle="-|>", color="r"
        )
        ax_3d.add_artist(arrow)

    # Create a Poly3DCollection for each polygon
    def make_poly_3d_collection(polygon_3d, facecolor, label, alpha=0.4):
        verts = [polygon_3d]
        polyc = Poly3DCollection([verts[0]], alpha=alpha)
        polyc.set_facecolor(facecolor)
        polyc.set_label(label)  # Add label for the legend
        return polyc

    # Feasible (stability) region in blue
    stab_poly_collection = make_poly_3d_collection(
        stab_polygon_3d, 'blue', 'Feasible Region', 0.4
    )
    ax_3d.add_collection3d(stab_poly_collection)

    # Reachable region in purple
    reach_poly_collection = make_poly_3d_collection(
        reach_polygon_3d, 'purple', 'Reachable Region', 0.4
    )
    ax_3d.add_collection3d(reach_poly_collection)

    # Adjust plot limits based on all geometry
    all_x = np.hstack([contactsWF[:, 0], stab_polygon_3d[:, 0], reach_polygon_3d[:, 0], [comWF[0]]])
    all_y = np.hstack([contactsWF[:, 1], stab_polygon_3d[:, 1], reach_polygon_3d[:, 1], [comWF[1]]])
    all_z = np.hstack([contactsWF[:, 2], stab_polygon_3d[:, 2], reach_polygon_3d[:, 2], [comWF[2]]])

    x_range = np.ptp(all_x)  # Range of X
    y_range = np.ptp(all_y)  # Range of Y
    z_range = np.ptp(all_z)  # Range of Z
    max_range = max(x_range, y_range)  # Largest range

    x_mid = (np.max(all_x) + np.min(all_x)) / 2
    y_mid = (np.max(all_y) + np.min(all_y)) / 2

    ax_3d.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
    ax_3d.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
    ax_3d.set_zlim(np.min(all_z), 0)

    ax_3d.set_xlabel('X axis')
    ax_3d.set_ylabel('Y axis')
    ax_3d.set_zlabel('Z axis')
    ax_3d.set_title("3D View")
    ax_3d.legend(loc='upper right', bbox_to_anchor=(1.5, 1.05))

    # 2D Top View
    ax_2d = fig.add_subplot(122)
    ax_2d.set_aspect('equal')

    # Plot contact points
    ax_2d.scatter(
        contactsWF[:, 0],
        contactsWF[:, 1],
        c='k', s=100
    )

    # Plot CoM
    ax_2d.scatter(
        comWF[0],
        comWF[1],
        c=color_com,
        s=100
    )

    # Plot polygons
    ax_2d.fill(
        stab_torque_polygon[:, 0],
        stab_torque_polygon[:, 1],
        color='blue', alpha=0.4
    )
    ax_2d.fill(
        reach_polygon[:, 0],
        reach_polygon[:, 1],
        color='purple', alpha=0.4
    )

    ax_2d.set_xlabel('X axis')
    ax_2d.set_ylabel('Y axis')
    ax_2d.set_title("2D Top View")

    plt.tight_layout()
    plt.show()


def main():
    robot_name, ng, constraint_mode_IP, mu, stanceFeet = initialize_robot_parameters()
    comWF = np.array([-0.0108847352, 0.0, -0.0277828674])
    comBF = comWF.copy()
    rpy_base = np.array([0.0, 0.0, 0.0])
    # Weight of the arm including wrist - 36.38 kg
    # Fully extended including wrist - 4475.5 mm
    current_arm_reach =0.475
    current_arm_mass = 20.38
    extForceW = np.array([0.0, 0.0, -current_arm_mass * 9.81])
    extTorqueW = np.array([
            0.0,
            current_arm_mass * 9.81 * (current_arm_reach / 2.0),
            0.0
        ])

    contactsBF = setup_contact_points()
    contactsWF, W_R_B = transform_contacts_to_world_frame(contactsBF, comWF, rpy_base)
    normals = compute_normals()

    feas_projector = ComputationalDynamics(robot_name)
    reach_projector = nonlinear_projection.NonlinearProjectionBretl(robot_name)

    params = setup_parameters(robot_name, contactsWF,
                              comWF, comBF, rpy_base,
                              normals, mu,
                              stanceFeet, feas_projector,
                              extForceW, extTorqueW)

    stab_torque_polygon, reach_polygon = compute_projections(feas_projector, reach_projector, params, comWF)
    isCoMStable = check_com_stability(feas_projector, params)

    plot_3d_and_top_view(contactsWF, normals, isCoMStable, comWF, stab_torque_polygon, reach_polygon)

if __name__ == "__main__":
    main()
