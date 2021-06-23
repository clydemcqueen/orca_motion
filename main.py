#!/usr/bin/env python3

"""
Testbed to plot and analyze orca3 motion in linear.z

z.csv is produced from ft12.pj.csv (Plotjuggler export from field test 12 rosbag), which has 9k msgs (rows) and ~200
fields (columns). Plotjuggler exports 1 message per row.

Trim this to key fields:
cut -d, -f1,21,36,42,48,54,60,66,70,76,88,94,188,189 ft12.pj.csv > z.csv

Result looks like this:
__time,/depth/z,/motion/accel_drag/linear/z,/motion/accel_hover/linear/z,/motion/accel_model/linear/z,/motion/accel_pid/linear/z,/motion/accel_total/linear/z,/motion/cmd_vel/linear/z,/motion/effort/force/z,/motion/force/force/z,/motion/pose/position/z,/motion/vel/linear/z,/odom/twist/covariance[4,4]
1624114603.333,,,,,,,,,,,,,
1624114603.334,,,,,,,,,,,,,
1624114603.334,,,,,,,,,,,,1452.000000,1548.000000
1624114603.334,,,,,,,,,,,,,
1624114603.334,,0.000000,-0.130999,0.000000,-0.044380,-0.175380,0.000000,-0.035389,-1.981788,-0.000003,0.000000,,
1624114603.334,0.088761,,,,,,,,,,,,
...

There are commas in the covariance field names, so the last 2 columns are /thrust/thrust.4 and /thrust/thrust.5
"""

import csv
import matplotlib.pyplot as plt
import underwater_motion


COL_STAMP=0
COL_DEPTH=1
COL_MOTION_ACCEL_DRAG=2
COL_MOTION_ACCEL_HOVER=3
COL_MOTION_ACCEL_MODEL=4
COL_MOTION_ACCEL_PID=5
COL_MOTION_ACCEL_TOTAL=6
COL_CMD_VEL=7
COL_MOTION_EFFORT=8
COL_MOTION_FORCE=9
COL_MOTION_POSE=10
COL_MOTION_VEL=11
COL_THRUST_4=12
COL_THRUST_5=13


class Plotter(object):
    def __init__(self):
        # From depth msg
        self.depth_t = []
        self.depth = []

        # From motion msg
        self.motion_t = []
        self.motion_accel_drag = []
        self.motion_accel_hover = []
        self.motion_accel_model = []
        self.motion_accel_pid = []
        self.motion_accel_total = []
        self.motion_cmd_vel = []
        self.motion_force = []
        self.motion_effort =[]
        self.motion_pose = []
        self.motion_vel = []

        # Calculated, possibly with proposed motion model changes
        self.um_pose = []
        self.um_vel = []
        self.um_force = []
        self.um_accel_pid = []
        self.um_jerk_model = []

        self.thrust_t = []
        self.thrust_4 = []
        self.thrust_5 = []

    def plot(self):
        fig, (pose_plot, vel_plot, accel_plot, jerk_plot, force_plot, thrust_plot) = plt.subplots(6, sharex='all')

        pose_plot.set_title('pose')
        pose_plot.plot(self.depth_t, self.depth, label='depth')
        pose_plot.plot(self.motion_t, self.motion_pose, label='model')
        pose_plot.plot(self.motion_t, self.um_pose, label='calculated model')
        pose_plot.legend()

        vel_plot.set_title('vel')
        vel_plot.plot(self.motion_t, self.motion_vel, label='model')
        vel_plot.plot(self.motion_t, self.motion_cmd_vel, label='cmd_vel')
        vel_plot.plot(self.motion_t, self.um_vel, label='calculated model')
        vel_plot.legend()

        accel_plot.set_title('accel')
        accel_plot.plot(self.motion_t, self.motion_accel_drag, label='drag')
        accel_plot.plot(self.motion_t, self.motion_accel_hover, label='hover')
        accel_plot.plot(self.motion_t, self.motion_accel_model, label='model')
        accel_plot.plot(self.motion_t, self.motion_accel_pid, label='pid')
        accel_plot.plot(self.motion_t, self.motion_accel_total, label='total')
        accel_plot.plot(self.motion_t, self.um_accel_pid, label='calculated pid')
        accel_plot.legend()

        jerk_plot.set_title('jerk')
        jerk_plot.plot(self.motion_t, self.um_jerk_model, label='calculated model')
        jerk_plot.legend()

        force_plot.set_title('force')
        force_plot.plot(self.motion_t, self.motion_force, label='total')
        force_plot.plot(self.motion_t, self.um_force, label='calculated total')
        force_plot.legend()

        thrust_plot.set_title('thrust')
        thrust_plot.plot(self.thrust_t, self.thrust_4, label='thrust 4')
        thrust_plot.plot(self.thrust_t, self.thrust_5, label='thrust 5')
        thrust_plot.legend()

        plt.show()


def main():
    plotter = Plotter()
    motion_model = underwater_motion.UnderwaterMotion()
    cmd_vel = 0

    with open('z.csv') as z_csv:
        reader = csv.reader(z_csv)
        for row in reader:
            if row[COL_DEPTH] != '':
                # Depth message from barometer sensor
                plotter.depth_t.append(float(row[COL_STAMP]))
                plotter.depth.append(float(row[COL_DEPTH]))

                # Re-calculate motion using the orca3 motion model
                # Useful to verify that the C++ and Python models produce the same results, to show non-published
                # data (e.g., jerk), or to test new ideas (e.g., added mass)
                motion_model.update(float(row[COL_STAMP]), cmd_vel, float(row[COL_DEPTH]))
                plotter.um_pose.append(motion_model.pose)
                plotter.um_vel.append(motion_model.vel)
                plotter.um_force.append(motion_model.force_total)
                plotter.um_accel_pid.append(motion_model.accel_pid)
                plotter.um_jerk_model.append(motion_model.jerk_model)

            elif row[COL_MOTION_POSE] != '':
                # Motion message from orca3 motion calculator
                plotter.motion_t.append(float(row[COL_STAMP]))
                plotter.motion_accel_drag.append(float(row[COL_MOTION_ACCEL_DRAG]))
                plotter.motion_accel_hover.append(float(row[COL_MOTION_ACCEL_HOVER]))
                plotter.motion_accel_model.append(float(row[COL_MOTION_ACCEL_MODEL]))
                plotter.motion_accel_pid.append(float(row[COL_MOTION_ACCEL_PID]))
                plotter.motion_accel_total.append(float(row[COL_MOTION_ACCEL_TOTAL]))
                plotter.motion_force.append(float(row[COL_MOTION_FORCE]))
                plotter.motion_cmd_vel.append(float(row[COL_CMD_VEL]))
                plotter.motion_pose.append(float(row[COL_MOTION_POSE]))
                plotter.motion_vel.append(float(row[COL_MOTION_VEL]))

                cmd_vel = float(row[COL_CMD_VEL])

            elif row[COL_THRUST_4] != '':
                # Thrust message
                plotter.thrust_t.append(float(row[COL_STAMP]))
                plotter.thrust_4.append(float(row[COL_THRUST_4]))
                plotter.thrust_5.append(float(row[COL_THRUST_5]))

    plotter.plot()



if __name__ == '__main__':
    main()
