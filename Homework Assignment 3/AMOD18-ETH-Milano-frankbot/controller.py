import math
import numpy as np
import rospy

class Controller():

    def __init__(self):
      # Whether to cross-validate or not
      self.cross_validation_mode = False

      # Gains for controller
      self.k_P = 4
      self.k_I = 5
      self.k_D = 0.01

      # Extremal values for cross validation
      self.k_P_start = 3
      self.k_P_end = 6
      self.k_I_start = 4
      self.k_I_end = 8

      self.total_time = 0
      self.time_for_k_I_step = 20

      # For D
      self.last_err = 0

      # For I
      self.total_err = 0

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
      if self.cross_validation_mode:
        # Perform a rough cross validation on k_P and k_I
        self.total_time += dt_last
        if self.total_time % self.time_for_k_I_step < 0.1:
          if self.k_I == self.k_I_end:
            self.k_P += 1
            self.k_I = self.k_I_start
          else:
            self.k_I += 1
          rospy.loginfo('Using k_P = {0}, k_I = {1}'.format(self.k_P, self.k_I))
        elif 1 < (self.total_time % self.time_for_k_I_step) < 6: # Move duckie!
          self.total_err = 0
          self.last_err = 0
          return (0, 0)

      # Calculate the output y
      ref = (6 * d_ref + 1 * phi_ref)
      y = (6 * d_est + 1 * phi_est)
      err = ref - y
      self.last_err = err

      # PID controller
      C_P = self.k_P * err + 1/self.k_I * self.total_err + self.k_D * (err-self.last_err)/dt_last

      omega = C_P

      self.total_err += err

      # Declaring return values
      omega_out = omega
      v_out = v_ref

      return (v_out, omega_out)
