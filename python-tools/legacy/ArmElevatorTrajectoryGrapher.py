#! python

import math

import matplotlib.pyplot as plt
import ntcore
import numpy as np
from matplotlib import gridspec
from matplotlib.widgets import Slider
from networktables.util import NetworkTables
from ntcore import NetworkTableInstance, EventFlags
from wpilib import SmartDashboard
from wpimath.trajectory import Trajectory

ARM_LENGTH = 0.6540246
MIN_ELEVATOR = 0.01
MAX_ELEVATOR = 1.035
CORAL_LENGTH= 0.30
CORAL_WIDTH = 0.11
CORAL_SHIFT = -0.045225
END_EFFECTOR_ANGLE = -math.radians(5)
dt = 0.02


####################################################################################################
def create_limits():
    l_arm = ARM_LENGTH
    h_elev = MAX_ELEVATOR
    sq = 0.3
    l = [(l_arm, h_elev), (l_arm, sq), (l_arm - sq, sq), (l_arm - sq, 0), (0, 0), (-l_arm, 0), (-l_arm, h_elev)]
    for deg in range(0, 180):
        theta = math.radians(180-deg)
        l.append((math.cos(theta) * l_arm, math.sin(theta) * l_arm + h_elev))
    l.append((l_arm, h_elev))
    return l

####################################################################################################
def plot_limits(plot, lmts):
    xs, ys = zip(*lmts)
    plot.plot(xs, ys, color='gray', dashes=[6, 2])

####################################################################################################
def plot_trajectory(plot, x_points: [], y_points: []):
    global current_x, current_y
    plot.plot(x_points, y_points, 'b', linestyle='-')
    plot.plot(current_x, current_y, 'r', linestyle='-')

####################################################################################################
def update_plot(elev:[], arm: [], x:[], y:[]) :
    main_plot(elev, arm, x, y)
    plt.draw()

####################################################################################################
def main_plot(elev: [], arm: [], x: [], y: []):
    global ax, trajectory, time_slider, limits
    # setup chart
    ax.cla()
    ax.axis('equal')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Elevator-Arm Trajectory')
    ax.grid(visible=True, which='major', axis='both', color='gray', linestyle='-', linewidth=1)

    # draw lines
    plot_limits(ax, limits)
    plot_trajectory(ax, x, y)

    # draw elevator-arm stick
    plot_elevator_arm(ax, elev, arm, x, y, time_slider.val)

    plot_elev()
    plot_arm()


def plot_elev():
    global ax_elev, time, elev_vel, current_elev_vel
    ax_elev.cla()
    ax_elev.set_xlabel('time (s)')
    ax_elev.set_ylabel('velocity (m/s)')
    ax_elev.set_title('Elevator Velocity')
    ax_elev.plot(time, elev_vel, color='g')
    ax_elev.plot(time, current_elev_vel, color='r')

def plot_arm():
    global ax_arm, time, arm_vel
    ax_arm.cla()
    ax_arm.set_xlabel('time (s)')
    ax_arm.set_ylabel('velocity (rad/s)')
    ax_arm.set_title('Arm Velocity')
    ax_arm.plot(time, arm_vel, color='g')
    ax_arm.plot(time, current_arm_vel, color='r')

####################################################################################################
def inverse_kinematics(x, y, is_up):
    theta = math.acos(max(-1.0, min(x / ARM_LENGTH, 1.0)))
    if not is_up:
        theta = -theta
    h = y - math.sin(theta) * ARM_LENGTH
    return [h, theta]

####################################################################################################
def forward_kinematics(elev, arm):
    x = ARM_LENGTH * math.cos(arm)
    y = elev + ARM_LENGTH * math.sin(arm)
    return x, y

####################################################################################################
def plot_elevator_arm(ax, elev, arm, xs, ys, percent_time):

    i = int(percent_time / 100.0 * len(xs) + 0.5)
    if i < 0:
        i = 0;
    if i > len(xs)-1:
        i = len(xs) -1

    x_e = xs[i]
    y_e = ys[i]
    h = elev[i]
    theta = arm[i]

    x = [0.0, 0.0, x_e]
    y = [0.0, h, y_e]
    ax.plot([0.0, 0.0], [h, MAX_ELEVATOR], color='black', linewidth=2)
    ax.plot(x, y, color='black', linewidth=5)
    ax.scatter([0.0], [h], color='black', s=100)



    dx, dy = CORAL_WIDTH/2, CORAL_LENGTH/2
    coral_theta = theta + END_EFFECTOR_ANGLE
    s, c = math.sin(coral_theta), math.cos(coral_theta)

    corners = [(-dx, -dy+CORAL_SHIFT), (dx, -dy+CORAL_SHIFT), (dx, dy+CORAL_SHIFT), (-dx, dy+CORAL_SHIFT)]
    # Rotate each corner around the center
    r = [
        ( x_e + x * c - y * s,  y_e + x * s + y * c )
        for x, y in corners
    ]
    x = [ r[0][0], r[1][0], r[2][0], r[3][0], r[0][0]]
    y = [ r[0][1], r[1][1], r[2][1], r[3][1], r[0][1]]
    ax.plot (x, y, color='black', linewidth=1)


####################################################################################################
def slider_redraw(_: float):
    update_plot(elev_pos, arm_pos, x_pos, y_pos)



def valueChanged(key, value, event: ntcore.Event ):
    data = event.data.value.getDoubleArray()

    global time, elev_pos, elev_vel, arm_pos, arm_vel, x_pos, y_pos, current_elev_pos, current_elev_vel, current_arm_pos, current_arm_vel, current_x, current_y

    time = []
    elev_pos = []
    elev_vel = []
    arm_pos = []
    arm_vel = []
    x_pos = []
    y_pos = []
    current_elev_pos = []
    current_elev_vel = []
    current_arm_pos = []
    current_arm_vel = []
    current_x = []
    current_y = []


    for i in range(0, len(data), 9):
        time.append(data[i])
        elev_pos.append(data[i+1])
        elev_vel.append(data[i+2])
        arm_pos.append(data[i+3])
        arm_vel.append(data[i+4])
        current_elev_pos.append(data[i+5])
        current_elev_vel.append(data[i+6])
        current_arm_pos.append(data[i+7])
        current_arm_vel.append(data[i+8])


    for i in range(len(elev_pos)):
        x, y = forward_kinematics(elev_pos[i], arm_pos[i])
        x_pos.append(x)
        y_pos.append(y)
        x, y = forward_kinematics(current_elev_pos[i], current_arm_pos[i])
        current_x.append(x)
        current_y.append(y)


    update_plot(elev_pos, arm_pos, x_pos, y_pos)




####################################################################################################
# main
####################################################################################################

IS_SIMULATION = True

inst = NetworkTableInstance.getDefault()

inst.startClient4("Trajectory Generator")
if not IS_SIMULATION:
    inst.setServerTeam(2046)
else:
    inst.setServer("localhost")

print("NT Version:", inst.getNetworkMode())

sd = inst.getTable("SmartDashboard")
limits = create_limits()
time, elev_pos, elev_vel, arm_pos, arm_vel = [], [], [], [], []
current_elev_pos, current_elev_vel, current_arm_pos, current_arm_vel = [], [], [], []
x_pos, y_pos = [], []
current_x, current_y = [], []

# create chart area
fig = plt.figure(figsize=(10, 10))
gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1], hspace=0.3, wspace=0.3)

ax = fig.add_subplot(gs[:, 0])
ax_elev = fig.add_subplot(gs[0, 1])
ax_arm = fig.add_subplot(gs[1, 1])

fig.subplots_adjust(bottom=0.2)


# add time control slider
axtime = fig.add_axes([0.20, 0.1, 0.65, 0.03])
time_slider = Slider(ax=axtime, label="Time (% full)", valmin = 0.0, valmax = 100.0, valinit=0.0)
time_slider.on_changed(slider_redraw)


sd.addListener("WindmillMoveCommand", EventFlags.kValueAll, valueChanged)
plt.show()