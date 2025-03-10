
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
import yaml
import math

class robotKinematics():
    def __init__(self, robotName):

        base_path = os.path.dirname(os.path.abspath(__file__))
        if sys.version_info[:2] == (2, 7) or sys.version_info[:2] == (3, 5):
            self.PKG = base_path + '/../../resources/urdfs/{}/'.format(robotName)
            self.URDF = self.PKG + 'urdf/{}.urdf'.format(robotName)
        else:
            self.PKG = f"{base_path}/../../resources/urdfs/{robotName}/"
            self.URDF = f"{self.PKG}urdf/{robotName}.urdf"

        self.FEET = self.PKG + 'robot_data.yaml'
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.model = self.robot.model
        self.data = self.robot.data
        self.feet_jac = None
        self.ik_success = False

        yaml_data = []
        self.urdf_feet_names = []
        self.default_q = []
        ## Can be used to compute q in an feet order similar to feet variables
        # Get feet frame names in a similar order to feet variables (position, etc...)
        with open(self.FEET, 'r') as stream:
            try:
                yaml_data = yaml.safe_load(stream)

            except yaml.YAMLError as exc:
                print(exc)

        self.urdf_feet_names = yaml_data['Feet_names']
        self.default_q = yaml_data['Default_q']
        self.default_q = np.vstack(self.default_q)
        self.q = self.default_q   # Last q computed
        # Get feet frame names in an alphabatical order to match pinocchio kinematics
        self.urdf_feet_names_pinocchio = []
        for frame in self.model.frames:
            if frame.name in self.urdf_feet_names:
                self.urdf_feet_names_pinocchio.append(frame.name)

    def getBlockIndex(self, frame_name):
        for i in range(len(self.urdf_feet_names_pinocchio)):
            if frame_name == self.urdf_feet_names_pinocchio[i]:
                idx = i * 3
                break

        return idx

    def computeFootForwardKinematics(self, q_leg, frame_name):
        q = np.zeros((self.model.nq))
        frame_id = self.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)

        q[blockIdx:blockIdx + 3] = q_leg
        pinocchio.forwardKinematics(self.model, self.data, q)
        pinocchio.framesForwardKinematics(self.model, self.data, q)
        return  self.data.oMf[frame_id].translation

    def computeFootJacobian(self, q_leg, frame_name):
        q = np.zeros((self.model.nq))
        frame_id = self.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        q[blockIdx:blockIdx + 3] = q_leg

        J = pinocchio.computeFrameJacobian(self.model, self.data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J[:3,blockIdx:blockIdx + 3]

    def footInverseKinematicsFixedBaseLineSearch(self, foot_pos_des, frame_name, q0_leg = np.zeros(3), verbose = False):


        # Error initialization
        e_bar = 1
        iter = 0
             # Recursion parameters
        epsilon = 0.0001  # Tolerance
        # alpha = 0.1
        alpha = 1  # Step size
        lambda_ = 0.00001# Damping coefficient for pseudo-inverse
        max_iter = 20 # Maximum number of iterations

        # For line search only
        gamma = 0.5
        beta = 0.5

        # Inverse kinematics with line search
        while True:
            # compute foot position
            foot_pos0 = np.ravel(self.computeFootForwardKinematics(q0_leg, frame_name))
            #get the square matrix jacobian that is smaller
            J_leg = self.computeFootJacobian(q0_leg, frame_name)

            # computed error wrt the des cartesian position
            e_bar = foot_pos_des - foot_pos0
            # print("e_bar:", e_bar)

            if np.linalg.norm(e_bar) < epsilon:
                IKsuccess = True
                if verbose:
                    print("IK Convergence achieved!, norm(error) :", np.linalg.norm(e_bar) )
                    print("Inverse kinematics solved in {} iterations".format(iter))
                break
            if iter >= max_iter:
                if verbose:
                    print(("\n Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is: ", np.linalg.norm(e_bar)))
                IKsuccess = False
                break

            #compute newton step
            dq = J_leg.T.dot(np.linalg.solve(J_leg.dot(J_leg.T) + lambda_ * np.identity(J_leg.shape[1]), e_bar))
            dq = np.ravel(dq)

            while True:
                # Update
                q1_leg = q0_leg + dq*alpha
                foot_pos1 = np.ravel(self.computeFootForwardKinematics(q1_leg, frame_name))

                #Compute error of next step
                e_bar_new = foot_pos_des - foot_pos1

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0 # even more strict: gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta*alpha
                    if verbose:
                        print (" line search: alpha: ", alpha)
                else:
                    q0_leg = q1_leg
                    alpha = 1
                    break
            iter += 1

        # unwrapping prevents from outputs larger than 2pi
        for i in range(len(q0_leg)):
            while q0_leg[i] >= 2 * math.pi:
                q0_leg[i] -= 2 * math.pi
            while q0_leg[i] < -2 * math.pi:
                q0_leg[i] += 2 * math.pi

        return q0_leg, J_leg, IKsuccess

    def fixedBaseInverseKinematics(self, feetPosDes, stance_idx, q0 = None, verbose = False):

        no_of_feet = len(self.urdf_feet_names)
        self.feet_jac = []
        q = []
        leg_ik_success = np.zeros((no_of_feet))

        if (q0 is None):
#            q0 =np.vstack((np.array([-0.2, 0.75, -1.5]),
#                np.array([-0.2, 0.75, -1.5]),
#                np.array([-0.2, -0.75, 1.5]),
#                np.array([-0.2, -0.75, 1.5])))
#             q0 = [-0.1, 0.75, -1.5, -0.1, 0.75, -1.5, -0.1, 0.75, -1.5, -0.1, 0.75, -1.5]
#             q0 = np.vstack(self.default_q)
#             q0 = q0.ravel()

            q0 = self.default_q.ravel()
        for leg in range(no_of_feet):
            '''Compute IK in similar order to feet location variable'''
            f_p_des = np.array(feetPosDes[leg, :]).T
            # q_leg, foot_jac, err, leg_ik_success[leg] = self.footInverseKinematicsFixedBase(f_p_des, self.urdf_feet_names[leg], q0)
            q_leg, foot_jac, leg_ik_success[leg]= self.footInverseKinematicsFixedBaseLineSearch(f_p_des, self.urdf_feet_names[leg], q0[leg*3 : leg*3+3], verbose)
            q = np.hstack([q, q_leg])
            self.feet_jac.append(foot_jac)
            # print("foot_jac:", foot_jac)

        stance_idx = np.array(stance_idx, dtype=int)  # Ensure stance_index is an array of integers
        number_of_stance_feet = np.shape(stance_idx)[0]
        are_stance_feet_in_workspace = np.full((1, number_of_stance_feet), False, dtype=bool)[0]
        previous_flag = True
        for k in np.arange(0, number_of_stance_feet):
            stance_leg_id = stance_idx[k]
            are_stance_feet_in_workspace[k] = leg_ik_success[int(stance_leg_id)]
            previous_flag = are_stance_feet_in_workspace[k] and previous_flag

        are_feet_in_workspace_flag = previous_flag
        #print "are_feet_in_ws_flag" , are_feet_in_workspace_flag
        self.ik_success = are_feet_in_workspace_flag

        if self.ik_success == False and verbose == True:
            print('Warning, IK failed in one of the legs')
        return q, self.ik_success

    def getLegJacobians(self):
        isOutOfWS = not self.ik_success
        return self.feet_jac, isOutOfWS

    def getCurrentQ(self):

        return self.q

    # def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):
    #     # Determine the number of legs to check based on joint_positions
    #     no_of_legs_to_check = int(joint_positions.size / 3)
    #     q = joint_positions.reshape((no_of_legs_to_check, 3))

    #     # Validate that joint_limits_max and joint_limits_min have the correct number of legs
    #     if joint_limits_max.shape[0] != no_of_legs_to_check:
    #         raise ValueError(f"Mismatch: joint_limits_max rows ({joint_limits_max.shape[0]}) != no_of_legs_to_check ({no_of_legs_to_check})")

    #     if joint_limits_min.shape[0] != no_of_legs_to_check:
    #         raise ValueError(f"Mismatch: joint_limits_min rows ({joint_limits_min.shape[0]}) != no_of_legs_to_check ({no_of_legs_to_check})")

    #     print("q: ", q)
    #     print("joint_limits_max", joint_limits_max)
    #     print("leq than max ", np.all(np.less_equal(q, joint_limits_max)))
    #     print("geq than min ", np.all(np.greater_equal(q, joint_limits_min)))

    #     # Return True if out of limits, else False
    #     return not np.all(np.less_equal(q, joint_limits_max)) or not np.all(np.greater_equal(q, joint_limits_min))

    # def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):
    #     print("stance_index FIRSTTT", stance_index)
    #     stance_index = np.array(stance_index, dtype=int)  # Ensure stance_index is an array of integers
    #     print("stance_index", stance_index)

    #     # Perform inverse kinematics
    #     q, ik_success = self.fixedBaseInverseKinematics(contactsBF_check, stance_index)

    #     # Assuming q is a flat array: [leg0_joint1, leg0_joint2, leg0_joint3, leg1_joint1, ...]
    #     # Reshape q to (number_of_legs, 3) for easier indexing
    #     num_total_legs = joint_limits_max.shape[0]  # Assuming joint_limits_max has shape (total_legs, 3)
    #     q_reshaped = q.reshape((num_total_legs, 3))

    #     # Select only the joints for the stance legs
    #     q_to_check = q_reshaped[stance_index].reshape(-1)  # Flatten back if needed

    #     print("joint_limits_max before filtering:", joint_limits_max)

    #     # Correctly filter joint limits based on stance_index
    #     filtered_joint_limits_max = joint_limits_max[stance_index]
    #     filtered_joint_limits_min = joint_limits_min[stance_index]

    #     print("Filtered joint_limits_max:", filtered_joint_limits_max)
    #     print("Filtered joint_limits_min:", filtered_joint_limits_min)

    #     # Check joint limits for the stance legs
    #     out = self.isOutOfJointLims(q_to_check, filtered_joint_limits_max, filtered_joint_limits_min)
    #     if not out:
    #         self.q = q

    #     # Optionally, check joint limits for all legs if needed
    #     return self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        no_of_legs_to_check = int(joint_positions.size/3)
        q = joint_positions.reshape((no_of_legs_to_check, 3))

        # print "q: ", q
        # print "leq than max ", np.all(np.less_equal(q, joint_limits_max))
        # print "geq than min ", np.all(np.greater_equal(q, joint_limits_min))
        return not np.all(np.less_equal(q, joint_limits_max)) \
               or not np.all(np.greater_equal(q, joint_limits_min))

    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):
        q, __ = self.fixedBaseInverseKinematics(contactsBF_check, stance_index)
        out = self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
        if not out:
            self.q = q

        return self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
