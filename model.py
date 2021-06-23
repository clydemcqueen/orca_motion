#!/usr/bin/env python3

"""
Orca3 physics model in Python (just linear.z)
"""

# From orca_shared/model.hpp:
GRAVITY = 9.8
ROV_DIM_X = 0.457
ROV_DIM_Y = 0.338
ROV_AREA_TOP = ROV_DIM_X * ROV_DIM_Y
T200_MAX_POS_FORCE = 50
T200_MAX_NEG_FORCE = 40
BOLLARD_FORCE_Z_UP = 2 * T200_MAX_POS_FORCE
BOLLARD_FORCE_Z_DOWN = 2 * T200_MAX_NEG_FORCE
mdl_mass = 11.3
mdl_volume = 0.01115
mdl_fluid_density = 1027.
mdl_thrust_scale = 0.7
mdl_drag_coef_z = 1.2
mdl_thrust_dz_pwm = 35  # Before ft12

# Possible future model.hpp value(s):
mdl_added_mass_coef = 0.5  # Model added mass as a % of displaced mass


# From orca_shared:
def bollard_force_z_up():
    return BOLLARD_FORCE_Z_UP * mdl_thrust_scale


def bollard_force_z_down():
    return BOLLARD_FORCE_Z_DOWN * mdl_thrust_scale


def accel_to_force(a):
    return a * mdl_mass


def force_to_effort(f):
    if f > 0:
        return f / bollard_force_z_up()
    else:
        return f / bollard_force_z_down()


def force_to_accel(f):
    return f / mdl_mass


def displaced_mass():
    return mdl_volume * mdl_fluid_density


def weight_in_water():
    return GRAVITY * (mdl_mass - displaced_mass())


def hover_accel_z():
    return weight_in_water() / mdl_mass


def drag_const_z():
    return 0.5 * mdl_fluid_density * ROV_AREA_TOP * mdl_drag_coef_z


def drag_force_z(v):
    # Motion works in all 4 quadrants, note the use of abs()
    return v * abs(v) * -drag_const_z()


def drag_accel_z(v):
    return force_to_accel(drag_force_z(v))


# New:
def accel_to_force_total_mass(a):
    return a * (mdl_mass + mdl_added_mass_coef * displaced_mass())
