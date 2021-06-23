#!/usr/bin/env python3

"""
Orca3 motion model in Python (just linear.z)
"""

import model
import pid
import util

# Parameters from orca_base/base_context.hpp:
z_vel = 0.4
z_accel = 0.2  # Modified from 0.4 in FT12
pid_z_kp = 0.8  # Modified from 0.5 in FT12
pid_z_ki = 0.
pid_z_kd = 0.
pid_z_i_max = 0.1


class UnderwaterMotion(object):
    def __init__(self):
        self.pid = pid.PID(pid_z_kp, pid_z_ki, pid_z_kd, pid_z_i_max)

        self.prev_t = 0.
        self.pose = 0.
        self.vel = 0.
        self.accel_model = 0.
        self.accel_drag = 0.
        self.accel_pid = 0.
        self.force_total = 0.

        # Experimental: limit jerk
        self.jerk_model = 0.

    def update(self, t, cmd_vel, depth):
        if not self.prev_t:
            self.prev_t = t
            return

        dt = t - self.prev_t
        self.prev_t = t

        # Calc pose from previous vel
        self.pose += self.vel * dt
        if self.pose > 0:
            self.pose = 0

        # Calc vel from previous accel
        self.vel = util.clamp(self.vel + self.accel_model * dt, z_vel)

        # Calc accel from cmd_vel
        accel_model_prev = self.accel_model
        self.accel_model = util.clamp((cmd_vel - self.vel) / dt, z_accel)

        # Calc jerk
        self.jerk_model = (self.accel_model - accel_model_prev) / dt

        # Drag
        self.accel_drag = -model.drag_accel_z(self.vel)

        # Assume PID control and hover are always enabled
        self.pid.set_target(self.pose)
        self.accel_pid = self.pid.calc(depth, dt)

        # Experiment: consider added mass for accel_model and accel_pid.
        # Drag and hover (buoyancy) forces do not result in acceleration through the fluid.
        self.force_total = model.accel_to_force_total_mass(self.accel_model) + \
                           model.accel_to_force(self.accel_drag) + \
                           model.accel_to_force(model.hover_accel_z()) + \
                           model.accel_to_force_total_mass(self.accel_pid)
