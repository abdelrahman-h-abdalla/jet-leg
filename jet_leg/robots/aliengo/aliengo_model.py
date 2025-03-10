import numpy as np

class AliengoModel:
    def __init__(self):

        self.trunkMass = 21.525
        self.trunkInertia = np.array([[0.274775, -0.00499095,  0.00217848],
                                              [-0.00499095,     1.44553, -0.00100954],
                                              [0.00217848, -0.00100954,     1.44601]])

        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [44.0, 44.0, 55.0]  # HAA, HFE, KFE
        RF_tau_lim = [44.0, 44.0, 55.0]  # HAA, HFE, KFE
        LH_tau_lim = [44.0, 44.0, 55.0]  # HAA, HFE, KFE
        RH_tau_lim = [44.0, 44.0, 55.0]  # HAA, HFE, KFE

        self.joint_torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
        self.contact_torque_limits = np.array([-1, 1])