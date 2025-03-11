# -*- coding: utf-8 -*-
"""
Interactive version of your stability & reach polygon code
with extra sliders for arm_reach and arm_mass in the top-right.
"""

from __future__ import absolute_import, division, print_function

import numpy as np
import copy
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as Rot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider
from matplotlib.patches import Rectangle

# jet_leg library imports (make sure these are available)
from jet_leg.plotting.arrow3D import Arrow3D
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.optimization import nonlinear_projection
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry


plt.rcParams.update({'font.size': 12}) 

ARM_REACH_FOLDED = 1.695 
ARM_REACH_MAX_DELTA = 1.
FEET_MAX_DELTA = 0.4

###############################################################################
# 1. Computation code pieces
###############################################################################


def initialize_robot_parameters():
    robot_name = 'hyqreal'
    ng = 4
    constraint_mode_IP = ['FRICTION_AND_ACTUATION'] * 4
    mu = 0.7
    stanceFeet = [1, 1, 1, 1]
    return robot_name, ng, constraint_mode_IP, mu, stanceFeet


def setup_contact_points():
    """
    These are the nominal contact points in the body frame (BF).
    We'll make them global so we can be manipulated by the sliders.
    """
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
        contactsWF[j, :] = np.dot(W_R_B, contactsBF[j, :]) + 0.0
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
    """
    Prepares the Bretl iterative projection parameters (stability region).
    """
    params = IterativeProjectionParameters(robot_name)
    params.setContactsPosWF(contactsWF)
    params.setCoMPosWF(comWF)
    params.setCoMPosBF(comBF)
    params.setOrientation(rpy_base)
    params.setTorqueLims(feas_projector.robotModel.robotModel.joint_torque_limits)
    params.setJointLimsMax(feas_projector.robotModel.robotModel.joint_limits_max)
    params.setJointLimsMin(feas_projector.robotModel.robotModel.joint_limits_min)
    params.externalForce = ext_force
    params.externalCentroidalTorque = ext_torque
    params.setActiveContacts(stanceFeet)
    params.setConstraintModes(['FRICTION_AND_ACTUATION'] * 4)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(4)
    params.setTotalMass(feas_projector.robotModel.robotModel.trunkMass)
    return params


def compute_projections(feas_projector, reach_projector, params, comWF):
    """
    Returns:
      - stab_torque_polygon (np.array Nx2 or Nx3)
      - reach_polygon      (np.array Nx2 or Nx3)
    Raises:
      - RuntimeError if projections cannot be computed.
    """
    try:
        stab_torque_polygon, _, _ = feas_projector.iterative_projection_bretl(params)
    except Exception as e:
        print(f"Error computing stability polygon: {e}")
        stab_torque_polygon = None

    try:
        reach_polygon, _ = reach_projector.project_polytope(
            params, comWF,
            20.0 * np.pi / 180,
            0.01
        )
    except Exception as e:
        print(f"Error computing reach polygon: {e}")
        reach_polygon = None

    return stab_torque_polygon, reach_polygon


def check_com_stability(feas_projector, params):
    try:
        isCoMStable, _, _ = feas_projector.check_equilibrium(params)
    except Exception as e:
        print(f"Error checking CoM stability: {e}")
        isCoMStable = False
    return isCoMStable

def plot_feet_arm(ax, contactsBF, arm_x, arm_y, min_x, max_x, min_y, max_y):
    """
    Draws a 2D plot of the foot contacts (BF x & y) and 
    a rectangle representing the arm from (0, arm_y - width/2) to (arm_x, arm_y + width/2).
    Only the bottom spine (x-axis) is shown with fixed axis limits.
    
    Parameters:
        ax (matplotlib.axes.Axes): The axes to plot on.
        contactsBF (np.array): Current contact points in body frame (shape: 4x3).
        arm_x (float): Current arm reach value on x-axis.
        arm_y (float): Current arm position on y-axis (set to 0.0 if not applicable).
        min_x (float): Fixed minimum x-axis limit based on hind contacts.
        max_x (float): Fixed maximum x-axis limit based on arm reach.
        y_min (float): Fixed minimum y-axis limit based on contact y-values.
        y_max (float): Fixed maximum y-axis limit based on contact y-values.
    """
    ax.cla()  # Clear previous content
    
    # Plot foot contacts in BF (x vs y)
    foot_x = contactsBF[:, 0]
    foot_y = contactsBF[:, 1]
    ax.scatter(foot_x, foot_y, c='k', s=60)
    
    # Define arm rectangle parameters
    arm_width = 0.15  # Small width on y-axis; adjust as needed
    
    # Create a Rectangle patch for the arm
    arm_rect = Rectangle(
        (0, arm_y - arm_width / 2),  # (x, y) of lower-left corner
        arm_x,                       # width
        arm_width,                   # height
        facecolor='grey',            # Fill color
        edgecolor='black',           # Border color 
        linewidth=1.5,
        label='Arm'
    )
    ax.add_patch(arm_rect)

    # Hide top, left, and right spines
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    
    # Hide y-axis ticks and labels
    ax.set_yticks([])
    
    # Set fixed axis limits
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    # Optional: Add a legend
    ax.legend(loc='upper right', fontsize=8)


def plot_3d_and_top_view(ax_3d, ax_2d,
                         contactsWF, normals,
                         isCoMStable, comWF,
                         stab_torque_polygon,
                         reach_polygon):
    """
    Plots both a 3D view (ax_3d) and a 2D top view (ax_2d).
    We'll re-draw everything from scratch each time 'update()' is called.
    Handles invalid polygons gracefully by displaying warnings.
    """

    def polygon_to_3d(poly_2d, z_level):
        """Convert 2D polygon => 3D polygon at z=comWF[2]."""
        if poly_2d is None:
            return None
        if len(poly_2d.shape) < 2:
            return None
        if poly_2d.shape[1] == 3:  # Already 3D
            poly_3d = poly_2d.copy()
            poly_3d[:, 2] = z_level  # Force it to the same Z
        else:  # If it's Nx2, convert to Nx3
            poly_3d = np.zeros((poly_2d.shape[0], 3))
            poly_3d[:, :2] = poly_2d
            poly_3d[:, 2] = z_level
        return poly_3d

    stab_polygon_3d = polygon_to_3d(stab_torque_polygon, comWF[2])
    reach_polygon_3d = polygon_to_3d(reach_polygon, comWF[2])

    # --- Clear old drawings ---
    ax_3d.cla()
    ax_2d.cla()

    # 3D PLOT ---------------------------------------------------------------
    ax_3d.scatter(
        contactsWF[:, 0],
        contactsWF[:, 1],
        contactsWF[:, 2],
        c='k', s=100, label='Contact Points'
    )

    color_com = 'g' if isCoMStable else 'r'
    ax_3d.scatter(
        comWF[0],
        comWF[1],
        comWF[2],
        c=color_com,
        s=100,
        zorder=2,
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
        if polygon_3d is None:
            return None
        polyc = Poly3DCollection([polygon_3d], alpha=alpha)
        polyc.set_facecolor(facecolor)
        polyc.set_label(label)
        return polyc

    # Feasible (stability) region in blue
    stab_poly_collection = make_poly_3d_collection(
        stab_polygon_3d, 'blue', 'Feasible Dynamics Region', 0.4
    )
    if stab_poly_collection:
        stab_poly_collection.set_zorder(1)  # Lower zorder
        ax_3d.add_collection3d(stab_poly_collection)

    # Reachable region in purple
    reach_poly_collection = make_poly_3d_collection(
        reach_polygon_3d, 'purple', 'Reachable Region', 0.6
    )
    if reach_poly_collection:
        reach_poly_collection.set_zorder(2)  # Upper zorder
        ax_3d.add_collection3d(reach_poly_collection)

    # Adjust plot limits
    all_x = np.hstack([
        contactsWF[:, 0],
        stab_polygon_3d[:, 0] if stab_polygon_3d is not None else [],
        reach_polygon_3d[:, 0] if reach_polygon_3d is not None else [],
        [comWF[0]]
    ])
    all_y = np.hstack([
        contactsWF[:, 1],
        stab_polygon_3d[:, 1] if stab_polygon_3d is not None else [],
        reach_polygon_3d[:, 1] if reach_polygon_3d is not None else [],
        [comWF[1]]
    ])
    all_z = np.hstack([
        contactsWF[:, 2],
        stab_polygon_3d[:, 2] if stab_polygon_3d is not None else [],
        reach_polygon_3d[:, 2] if reach_polygon_3d is not None else [],
        [comWF[2]]
    ])

    if all_x.size == 0:
        all_x = np.array([comWF[0]])
    if all_y.size == 0:
        all_y = np.array([comWF[1]])
    if all_z.size == 0:
        all_z = np.array([comWF[2]])

    x_range = np.ptp(all_x)
    y_range = np.ptp(all_y)
    z_range = np.ptp(all_z)
    max_range = max(x_range, y_range, 0.5)

    x_mid = (np.max(all_x) + np.min(all_x)) / 2.0
    y_mid = (np.max(all_y) + np.min(all_y)) / 2.0
    z_min = np.min(all_z)
    z_max = np.max(all_z)

    ax_3d.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
    ax_3d.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)

    if z_min == z_max:
        z_min = z_min - 0.1
        z_max = z_max + 0.1
    ax_3d.set_zlim(z_min, z_max + 0.1)

    ax_3d.set_xlabel('X axis')
    ax_3d.set_ylabel('Y axis')
    ax_3d.set_zlabel('Z axis')
    # ax_3d.set_title("3D View")

    # Center the legend at the top middle (adjust as needed)
    handles, labels = ax_3d.get_legend_handles_labels()
    if stab_poly_collection:
        stab_poly_collection.set_zorder(2)  # Lower zorder
        handles.append(stab_poly_collection)
    if reach_poly_collection:
        reach_poly_collection.set_zorder(2)  # Upper zorder
        handles.append(reach_poly_collection)
    ax_3d.legend(handles, labels, loc='upper center', bbox_to_anchor=(1.15, 1.25),
                 fancybox=True, shadow=True, ncol=2)

    # 2D TOP VIEW -----------------------------------------------------------
    # ax_2d.set_aspect('equal')
    ax_2d.scatter(
        contactsWF[:, 0],
        contactsWF[:, 1],
        c='k', s=100
    )
    if stab_torque_polygon is not None and stab_torque_polygon.size > 0:
        ax_2d.fill(
            stab_torque_polygon[:, 0],
            stab_torque_polygon[:, 1],
            color='blue', alpha=0.4
        )
    if reach_polygon is not None and reach_polygon.size > 0:
        ax_2d.fill(
            reach_polygon[:, 0],
            reach_polygon[:, 1],
            color='purple', alpha=0.6,
            zorder=2
        )
    ax_2d.scatter(
        comWF[0],
        comWF[1],
        c=color_com,
        s=100,
        zorder=3
    )

    if len(all_x) > 0:
        x_min, x_max = np.min(all_x), np.max(all_x)
        y_min, y_max = np.min(all_y), np.max(all_y)
        margin_x = 0.1 * (x_max - x_min) if (x_max != x_min) else 0.1
        margin_y = 0.1 * (y_max - y_min) if (y_max != y_min) else 0.1

        ax_2d.set_xlim(x_min - margin_x, x_max + margin_x)
        ax_2d.set_ylim(y_min - margin_y, y_max + margin_y)
    else:
        ax_2d.set_xlim(-1, 1)
        ax_2d.set_ylim(-1, 1)

    ax_2d.set_xlabel('X axis')
    ax_2d.set_ylabel('Y axis')
    ax_2d.set_title("2D Top View")

    warning_text = ""
    if stab_torque_polygon is None:
        warning_text += "Stability polygon could not be computed.\n"
    if reach_polygon is None:
        warning_text += "Reachable polygon could not be computed."

    if warning_text:
        ax_2d.text(0.5, 0.95, warning_text, transform=ax_2d.transAxes,
                   fontsize=10, color='red', ha='center', va='top',
                   bbox=dict(facecolor='white', alpha=0.8, edgecolor='red'))

    # ax_2d.relim()
    # ax_2d.autoscale_view()


###############################################################################
# 2. The interactive code with sliders
###############################################################################


def main_interactive():
    """
    Main entry point that sets up:
      - Robot parameters
      - COM, external forces, etc.
      - The figure with 2 subplots (3D + 2D)
      - 12 sliders for foot contact positions
      - 2 sliders for arm_reach and arm_mass in the top-right
      - A callback function that re-computes everything when sliders move
    """
    plt.close('all')  # Start fresh

    # --- Initialize "static" data ---
    robot_name, ng, constraint_mode_IP, mu, stanceFeet = initialize_robot_parameters()
    comWF = np.array([-0.0108847352, 0.0, -0.077828674])
    comBF = comWF.copy()
    rpy_base = np.array([0.0, 0.0, 0.0])

    # We'll define 'arm_reach' and 'arm_mass' initially:
    arm_reach_init =0.475
    arm_mass_init = 20.38

    # Nominal contact points in BF (we will adjust them via sliders)
    contactsBF_nominal = setup_contact_points()

    # Compute fixed axis limits for the feet and arm plot
    hind_contacts = contactsBF_nominal[2:4, :]  # Assuming LH and RH are the last two
    min_hind_x = np.min(hind_contacts[:, 0]) - FEET_MAX_DELTA
    max_arm_reach = arm_reach_init  + ARM_REACH_MAX_DELTA  # Set maximum arm reach based on initial value

    # Determine y-axis limits based on all contact y-values
    y_min = np.min(contactsBF_nominal[:, 1]) - FEET_MAX_DELTA
    y_max = np.max(contactsBF_nominal[:, 1]) + FEET_MAX_DELTA

    # Optionally, add a small buffer to y-axis limits for better visualization
    y_buffer = 0.1 * (y_max - y_min) if y_max > y_min else 0.1
    y_min -= y_buffer
    y_max += y_buffer


    # Create the computational dynamics objects
    try:
        feas_projector = ComputationalDynamics(robot_name)
        reach_projector = nonlinear_projection.NonlinearProjectionBretl(robot_name)
    except Exception as e:
        print(f"Error initializing ComputationalDynamics or NonlinearProjection: {e}")
        return

    # The contact normals (just +Z for each foot in body frame)
    normals = compute_normals()

    # -- FIGURE and SUBPLOTS for live update
    fig = plt.figure(figsize=(14, 12))
    ax_3d = fig.add_axes([0.05, 0.2, 0.4, 0.7], projection='3d')
    ax_2d = fig.add_axes([0.55, 0.3, 0.4, 0.5])
    ax_feet_arm = fig.add_axes([0.05, 0.85, 0.3, 0.1])  # left, bottom, width, height


    # We will store the "current" BF foot positions in a mutable array
    contactsBF_current = contactsBF_nominal.copy()

    # Initialize sliders list before defining update_plot
    sliders = []
    slider_arm_reach = None
    slider_arm_mass = None

    def update_plot(_=None):
        """
        Called by each slider when its value changes.
        Recompute:
          1) transform BF->WF
          2) stability & reach polygons
          3) stable or not
          4) re-draw
        """
        # 1) gather the BF foot positions from the slider values
        for i in range(4):
            contactsBF_current[i, 0] = sliders[i*3 + 0].val  # x
            contactsBF_current[i, 1] = sliders[i*3 + 1].val  # y
            contactsBF_current[i, 2] = sliders[i*3 + 2].val  # z

        # Also gather arm_reach and arm_mass
        current_arm_reach = slider_arm_reach.val
        current_arm_mass = slider_arm_mass.val

        # 2) transform to WF
        contactsWF, W_R_B = transform_contacts_to_world_frame(
            contactsBF_current, comWF, rpy_base
        )

        # 3) set up external force & torque based on arm sliders
        #    (Force is downward, torque is about Y if you imagine the arm out in front)
        extForceW = np.array([0.0, 0.0, -current_arm_mass * 9.81])
        extTorqueW = np.array([
            0.0,
            current_arm_mass * 9.81 * (current_arm_reach / 2.0),
            0.0
        ])

        # 4) set up parameters
        params = setup_parameters(
            robot_name, contactsWF,
            comWF, comBF, rpy_base,
            normals, mu,
            stanceFeet, feas_projector,
            extForceW, extTorqueW
        )

        # 5) compute polygons with error handling
        stab_polygon, reach_polygon = compute_projections(
            feas_projector, reach_projector, params, comWF
        )

        # 6) check stability with error handling
        stable_flag = check_com_stability(feas_projector, params)

        # 7) re-draw the plots
        plot_3d_and_top_view(
            ax_3d, ax_2d,
            contactsWF, normals,
            stable_flag, comWF,
            stab_polygon, reach_polygon
        )

        plot_feet_arm(
        ax_feet_arm, 
        contactsBF_current, 
        current_arm_reach, 
        0.0,             # arm_y position; adjust if needed 
        min_hind_x,      # Fixed min x based on hind contacts 
        max_arm_reach,   # Fixed max x based on arm reach 
        y_min,           # Fixed min y based on contacts 
        y_max            # Fixed max y based on contacts
        ) 

        fig.canvas.draw_idle()

    # Define the foot labels in the order: LF, RF, LH, RH
    foot_labels = ["LF", "RF", "LH", "RH"]
    coord_labels = ["X", "Y", "Z"]  # Ensure this is defined

    slider_configs = []
    for i, foot_label in enumerate(foot_labels):
        for j, coord_label in enumerate(['x', 'y', 'z']):
            init_val = contactsBF_nominal[i, j]
            valmin = init_val - FEET_MAX_DELTA
            valmax = init_val + FEET_MAX_DELTA
            slider_lbl = f"{foot_label}_{coord_label}"
            slider_configs.append((slider_lbl, init_val, valmin, valmax))

    # Layout parameters for foot sliders (bottom rows)
    slider_height = 0.03
    slider_gap_x = 0.09
    slider_gap_y = 0.02
    sliders_per_row = 3
    total_slider_rows = 4
    slider_width = 0.18

    # Starting y position for the bottom row
    start_y = 0.05  # Increased from 0.05 to leave space for row labels

    # We'll invert the row index so row=0 => top row, row=3 => bottom row
    total_width = sliders_per_row * slider_width + (sliders_per_row - 1) * slider_gap_x
    start_x = (1.0 - total_width) / 2  # Center horizontally

    # Create the sliders and add labels
    for idx, (label, init_val, valmin, valmax) in enumerate(slider_configs):
        row = idx // sliders_per_row     # 0..3
        col = idx % sliders_per_row      # 0..2
        row_inverted = (total_slider_rows - 1) - row

        x = start_x + col * (slider_width + slider_gap_x)
        y = start_y + row_inverted * (slider_height + slider_gap_y)
        w = slider_width
        h = slider_height

        # --- ADD THESE LINES TO CREATE TEXT LABELS ---

        # 1) Row label on the left (LF, RF, LH, RH) only once per row, i.e. col == 0
        if col == 0:
            fig.text(
                x - 0.06,                # Shift left from the first slider in the row
                y + h / 2,               # Center vertically relative to the slider
                foot_labels[row],        # "LF", "RF", "LH", or "RH"
                va="center",
                ha="right",
                fontsize=10,
                fontweight="bold",
                color='black'
            )

        # 2) Column label at the top (X, Y, Z) only once per column in the top row, i.e. row==0
        if row == 0:
            fig.text(
                x + w / 2,              # Center horizontally over the slider
                y + h + 0.0,           # Slightly above the top slider in the column
                coord_labels[col].upper(),  # "X", "Y", or "Z" in uppercase
                va="bottom",
                ha="center",
                fontsize=10,
                fontweight="bold",
                color='black'
            )

        # Then create the slider
        ax_rect = [x, y, w, h]
        ax_sli = fig.add_axes(ax_rect)
        s = Slider(
            ax=ax_sli,
            label=label,
            valmin=valmin,
            valmax=valmax,
            valinit=init_val,
            orientation='horizontal'
        )
        s.on_changed(update_plot)
        sliders.append(s)

    # Now create top-right sliders for arm_reach and arm_mass.
    # Let's place them at a fixed location in the top-right.
    # Adjust the x,y positions as desired to make them look nice.
    ax_arm_reach = fig.add_axes([0.72, 0.9, 0.20, 0.03])
    slider_arm_reach = Slider(
        ax=ax_arm_reach,
        label="Arm Reach",
        valmin=ARM_REACH_FOLDED,
        valmax=arm_reach_init + ARM_REACH_MAX_DELTA,
        valinit=arm_reach_init,
        orientation='horizontal'
    )
    slider_arm_reach.on_changed(update_plot)

    ax_arm_mass = fig.add_axes([0.72, 0.85, 0.20, 0.03])
    slider_arm_mass = Slider(
        ax=ax_arm_mass,
        label="Arm Mass",
        valmin=0.0,
        valmax=arm_mass_init + 20.,
        valinit=arm_mass_init,
        orientation='horizontal'
    )
    slider_arm_mass.on_changed(update_plot)

    # Leave space for bottom foot sliders + top legend
    plt.subplots_adjust(
        bottom=0.05,  # Reduced bottom to accommodate labels
        top=0.95,
        left=0.05,
        right=0.95
    )

    # INITIAL PLOT AFTER SLIDERS ARE CREATED
    update_plot()

    # Show everything
    plt.show()



###############################################################################
# 3. Entry point
###############################################################################

if __name__ == "__main__":
    main_interactive()

