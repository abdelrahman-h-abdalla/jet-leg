import numpy as np

class HyqrealModel:
    def __init__(self):

        self.trunkMass = 120
        self.trunkInertia = np.array([[4.91182, -0.115619, -0.961905],
                                              [-0.115619,   19.5025, 0.0136923],
                                              [-0.961905, 0.0136923,   21.2841]])
        self.com_BF = np.array([0.006, -0.002, -0.044]) # Average of trunk CoM and nominal whole-body CoM

        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [173.0, 208.0, 249.0]  # HAA, HFE, KFE
        RF_tau_lim = [173.0, 208.0, 249.0]  # HAA, HFE, KFE
        LH_tau_lim = [173.0, 208.0, 249.0]  # HAA, HFE, KFE
        RH_tau_lim = [173.0, 208.0, 249.0]  # HAA, HFE, KFE

        self.joint_torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
        self.contact_torque_limits = np.array([-1, 1])

        LF_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
        LF_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
        RF_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
        RF_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
        LH_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
        LH_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
        RH_q_lim_max = [0.401, 2.181, -0.770]  # HAA, HFE, KFE
        RH_q_lim_min = [-0.733, 0.262, -2.770]  # HAA, HFE, KFE
        self.joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
        self.joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])

        x_nominal_b = 0.44
        y_nominal_b = 0.34
        z_nominal_b = -0.55
        self.nominal_stance_LF = [x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RF = [x_nominal_b, -y_nominal_b, z_nominal_b]
        self.nominal_stance_LH = [-x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RH = [-x_nominal_b, -y_nominal_b, z_nominal_b]
        self.max_dev_from_nominal = [0.2, 0.15, 0.2]

        # Max external wrench
        self.max_ext_force = [300, 300, 300]
        self.max_ext_torque = [150, 150, 150]
        # self.max_ext_torque = [100, 150, 150]