import pinocchio as pin
from pinocchio.utils import zero
import numpy as np

# Path to your robot's URDF file
urdf_path = '/home/aabdalla/workspace/jet-leg/resources/urdfs/mantis_belt_star/urdf/mantis_belt_star.urdf'
# Load the robot model
model = pin.buildModelFromUrdf(urdf_path)

# Create a data structure required by Pinocchio
data = model.createData()

# Number of joints
nq = model.nq

# Define the joint configuration
# For example, set all joint angles to zero
q = zero(nq)
joint_names = ['LF_HAA', 'LF_HFE', 'LF_KFE',
               'RF_HAA', 'RF_HFE', 'RF_KFE',
               'LH_HAA', 'LH_HFE', 'LH_KFE',
               'RH_HAA', 'RH_HFE', 'RH_KFE']
joint_angles = [1.00, -1.54, 1.50,
                -1.00, -1.54, 1.50,
                2.27, -1.54, 1.50,
                -2.27, -1.54, 1.50]
for name, angle in zip(joint_names, joint_angles):
    joint_id = model.getJointId(name)
    q[joint_id - 1] = angle  # Pinocchio joint indexing starts at 0

# Compute forward kinematics
pin.forwardKinematics(model, data, q)

# Update the frame placements (positions and orientations)
pin.updateFramePlacements(model, data)

# Compute the Center of Mass position in the world frame and total mass
com_world = pin.centerOfMass(model, data, q, False)
total_mass = sum([inertia.mass for inertia in model.inertias])

# Get the transformation of the base frame in the world frame
base_frame_id = model.getFrameId('trunk')
T_base_world = data.oMf[base_frame_id]

# Transform the CoM to the base frame
com_base = T_base_world.inverse().act(com_world)

print("Total Mass of the Robot:", total_mass)
print("Center of Mass in the world frame:", com_world)
print("Center of Mass in the base frame:", com_base)

# Compute the inertia tensor about the CoM
# Initialize the inertia matrix
I_com = np.zeros((3, 3))

# Iterate over all the links to accumulate their contributions
for joint_id in range(model.njoints):
    # Get the spatial inertia of the link
    link_inertia = model.inertias[joint_id]
    
    # Get the placement of the link's CoM in the world frame
    joint_frame = model.frames[joint_id].parent
    T_link_world = data.oMi[joint_id]
    com_link_world = T_link_world.translation  # Assuming CoM is at the origin of each link's frame

    # Vector from CoM of the robot to CoM of the link
    r = com_link_world - com_world

    # Compute the rotation matrix of the link's frame
    R = T_link_world.rotation

    # Inertia tensor of the link in the world frame
    I_link_world = R.dot(link_inertia.inertia).dot(R.T)

    # Apply the parallel axis theorem to shift inertia to the CoM of the robot
    I_shifted = I_link_world + link_inertia.mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))

    # Accumulate the inertia
    I_com += I_shifted

print("Total Mass:", total_mass)
print("Inertia Tensor about CoM:\n", I_com)
