#!/usr/bin/env python3

"""
Orca3 PID controller in Python
"""

import util


class PID(object):
    def __init__(self, kp_, ki_, kd_, i_max_):
        self.kp = kp_
        self.ki = ki_
        self.kd = kd_
        self.i_max = i_max_
        self.target = 0.
        self.prev_error = 0.
        self.integral = 0.

    def set_target(self, target_):
        if abs(target_ - self.target) > 0.001:
            self.target = target_
            self.prev_error = 0.
            self.integral = 0.

    def calc(self, state, dt):
        error = self.target - state

        if self.ki:
            self.integral += (error * dt)
            if self.i_max:
                self.integral = util.clamp(self.integral, self.i_max/self.ki)

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error)

        self.prev_error = error

        return p_term + i_term + d_term
