import math
import numpy as np


class Controller():

    def __init__(self):

        # Gains for controller
        self.k_P = 7
        self.k_I = 0.05
        self.k_D = 2.5

        # Initialize integral and derivative terms
        self.C_I = 0
        self.C_D = 0

        # C-matrix values
        self.c_1 = 6
        self.c_2 = 1

        #Initialize error term
        self.error = 0

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

        # Calculate the output y
        ref =   (self.c_1 * d_ref + self.c_2 * phi_ref)
        y =     (self.c_1 * d_est + self.c_2 * phi_est)
        err = ref - y

        # General PID controller
        C_P = self.k_P * err
        omega = C_P + self.C_I + self.C_D

        # Updating Integral and derivative terms
        self.C_I = self.C_I + dt_last * (self.k_I * (err + self.error)/2)
        self.C_D = self.k_D * (err - self.error)/dt_last

        # Update error
        self.error = err

        # Declaring return values
        omega_out = omega
        v_out = v_ref

        return (v_out, omega_out)
