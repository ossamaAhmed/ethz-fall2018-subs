import math
import numpy as np


class Controller():

    def __init__(self):

        # Set your parameters here
        # Gains for controller
        self.k_P = 2
        self.k_P_d = 6
        self.k_P_w = 2
        self.k_I_d = 0.1
        self.k_I_w = 0.1
        self.d_I = 0
        self.w_I = 0


    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       forward linear velocity of Duckiebot [m/s]
    #           omega_out   angular velocity of Duckiebot [rad/s]
    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        # Do your calculations here
        self.d_I +=  (d_ref - d_est)*dt_last
        self.w_I +=  (phi_ref - phi_est)*dt_last

        # Calculate the output y
        #ref =   (6 * d_ref + 1 * phi_ref)
        d_phi = phi_ref - phi_est
        d_d = d_ref - d_est

        y_k = (self.k_P_d * d_d + self.k_P_w *d_phi + self.k_I_d*self.d_I + self.k_I_w*self.w_I)

        # Native P-Controller
        C_P = self.k_P * y_k
        omega = C_P

        # Declaring return values
        omega_out = omega
        v_out = v_ref

        return (v_out, omega_out)
