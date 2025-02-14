#! python

import math
import os

import tkinter.filedialog as fd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backend_bases import MouseButton, KeyEvent, MouseEvent
import copy
from matplotlib.widgets import Slider
from shapely.geometry.point import Point
from shapely.geometry.polygon import Polygon
from wpimath.geometry import Translation2d
from wpimath.spline import Spline3
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance

ARM_LENGTH = 0.6540246
MIN_ELEVATOR = 0.0
MAX_ELEVATOR = 1.0
CORAL_LENGTH= 0.30
CORAL_WIDTH = 0.11
CORAL_SHIFT = -0.045225
END_EFFECTOR_ANGLE = -math.radians(5)

####################################################################################################
def sign(x: float) -> int:
    return int(math.copysign(1, x))

####################################################################################################
def create_limits():
    l_arm = ARM_LENGTH
    h_elev = 1.0
    sq = 0.3
    l = [(l_arm, h_elev), (l_arm, sq), (l_arm - sq, sq), (l_arm - sq, 0), (0, 0), (-l_arm, 0), (-l_arm, h_elev)]
    for deg in range(0, 180):
        theta = math.radians(180-deg)
        l.append((math.cos(theta) * l_arm, math.sin(theta) * l_arm + h_elev))
    l.append((l_arm, h_elev))
    return l

####################################################################################################
def create_trajectory(p, t):
    last = len(p)-1
    start = Spline3.ControlVector((p[0][0], t[0][0]), (p[0][1], t[0][1]))
    waypoints = []
    if len(p) > 2:
        for i in range(1, last):
            waypoints.append(Translation2d(p[i][0], p[i][1]))
    end = Spline3.ControlVector((p[last][0], t[last][0]), (p[last][1], t[last][1]))
    t = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config)
    x = ArmElevTrajectory(t, None)
    #print_points()
    return x


####################################################################################################
def plot_trajectory(plot, traj :Trajectory):
    times = np.linspace(0, traj.totalTime(), num=400)
    x_points, y_points = [], []

    for t in times:
        sample = traj.sample(t)
        x_points.append(sample.pose.X())
        y_points.append(sample.pose.Y())

    plot.plot(x_points, y_points, 'b', linestyle='-')

####################################################################################################
def plot_controls(plot, pts, tgs):
    last = len(pts) - 1
    plot.scatter(*zip(*pts), color='red', label="Control Points")
    plot.arrow(pts[0][0], pts[0][1], tgs[0][0], tgs[0][1], head_width=0.02, color='green')
    plot.arrow(pts[last][0], pts[last][1], tgs[last][0], tgs[last][1], head_width=0.02, color='green')

####################################################################################################
def plot_limits(plot, lmts):
    xs, ys = zip(*lmts)
    plot.plot(xs, ys, color='gray', dashes=[6, 2])

####################################################################################################
def update_plot() :
    global trajectory, time_slider
    global ax, points, tangents

    trajectory = create_trajectory(points, tangents)

    main_plot()
    plt.draw()
    send_to_robot()

####################################################################################################
def main_plot():
    global ax, trajectory, time_slider
    # setup chart
    ax.cla()
    ax.axis('equal')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Elevator-Arm Trajectory')
    ax.grid(visible=True, which='major', axis='both', color='gray', linestyle='-', linewidth=1)

    # draw lines
    plot_limits(ax, limits)
    plot_controls(ax, points, tangents)
    plot_trajectory(ax, trajectory.getTrajectory())

    # draw elevator-arm stick
    plot_elevator_arm(ax, trajectory, time_slider.val)

####################################################################################################
def select_control(x, y):
    global selected_point
    global selected_tangent
    global points
    global tangents

    # distances to points
    distances = [np.hypot(px - x, py - y) for px, py in points]

    # distances to end point tangents
    last = len(points)-1
    tangent_distance_1st = np.hypot(points[0][0] + tangents[0][0] - x, points[0][1] + tangents[0][1] - y)
    tangent_distance_2nd = np.hypot(points[last][0] + tangents[last][0] - x, points[last][1] + tangents[last][1] - y)

    # find the closest point or tangent
    if distances:
        min_index = np.argmin(distances)
        min_dist = distances[min_index]
        if tangent_distance_1st > min_dist and tangent_distance_2nd > min_dist:
            selected_point = min_index
        elif tangent_distance_1st < tangent_distance_2nd:
            selected_tangent = 0
        else:
            selected_tangent = last

####################################################################################################
def release_control():
    global selected_point
    global selected_tangent
    selected_point = None
    selected_tangent = None

####################################################################################################
def move_control(x, y):
    global selected_point, selected_tangent
    global points, tangents

    if selected_point is None and selected_tangent is None:
        return

    new_points = copy.deepcopy(points)
    new_tangents = copy.deepcopy(tangents)

    pt = (x,y)
    if selected_point is not None:
        new_points[selected_point] = pt
    elif selected_tangent is not None:
        dx = pt[0] - points[selected_tangent][0]
        dy = pt[1] - points[selected_tangent][1]
        new_tangents[selected_tangent] = (dx, dy)

    if is_new_trajectory_valid(new_points, new_tangents):
        points = new_points
        tangents = new_tangents
        update_plot()

####################################################################################################
def send_to_robot():
    data: list[float] = []

    l = len(points)-1
    data.append(config.maxVelocity())
    data.append(config.maxAcceleration())

    data.append(points[0][0])
    data.append(points[0][1])

    data.append(tangents[0][0])
    data.append(tangents[0][1])

    data.append(points[l][0])
    data.append(points[l][1])

    data.append(tangents[l][0])
    data.append(tangents[l][1])

    for i in range(1, l):
        data.append(points[i][0])
        data.append(points[i][1])

    SmartDashboard.putNumberArray("Trajectory Editor Input", data)


####################################################################################################
def is_new_trajectory_valid(p, t):
    trajectory = create_trajectory(p, t)
    times = np.linspace(0, trajectory.totalTime(), num=400)

    limits_polygon = Polygon(limits)

    for t in times:
        sample_xy = trajectory.getTrajectory().sample(t)
        if not limits_polygon.contains(Point((sample_xy.pose.X(), sample_xy.pose.Y()))):
            return False

        h, theta = trajectory.sample(t)
        if h < MIN_ELEVATOR or h > MAX_ELEVATOR:
            return False

    return True



####################################################################################################
def insert_point(x, y):
    global points, tangents
    distances = [np.hypot((px1 + px2) / 2 - x, (py1 + py2) / 2 - y) for (px1, py1), (px2, py2) in zip(points, points[1:])]
    min_index = np.argmin(distances) + 1
    points.insert(min_index, (x, y))
    tangents.insert(min_index, (0,0))
    update_plot()

####################################################################################################
def delete_point(x: float, y: float):
    global points, tangents
    if len(points) == 2:
        return
    distances = [np.hypot(px - x, py - y) for px, py in points]
    min_index = np.argmin(distances)
    del points[min_index]
    del tangents[min_index]
    update_plot()

####################################################################################################
def on_press(event: MouseEvent):
    if event.xdata is None or event.ydata is None:
        return
    if axtime.contains(event)[0]:
        return False
    if event.button == MouseButton.LEFT:
        select_control(event.xdata, event.ydata)

####################################################################################################
def on_release(event: MouseEvent):
    if event.xdata is None or event.ydata is None:
        return
    if event.button == MouseButton.LEFT:
        release_control()

####################################################################################################
def on_motion(event: MouseEvent):
    if event.xdata is None or event.ydata is None:
        return
    if event.button == MouseButton.LEFT:
        move_control(event.xdata, event.ydata)

####################################################################################################
def on_key(event: KeyEvent):
    if event.xdata and event.ydata:
        if event.key == 'a':
            insert_point(event.xdata, event.ydata)
        elif event.key == 'd':
            delete_point(event.xdata, event.ydata)
    if event.key == 'ctrl+l': save_to_file(None)
    elif event.key == 'ctrl+o': load_from_file(None)


####################################################################################################
def save_to_file(file_name: str | None):
    file_name = file_name or fd.asksaveasfilename(filetypes=[("Trajectories", "*.traj")])
    if not file_name:
        return

    with open(file_name + ".traj", "w") as f:
        data = zip(points, tangents)
        f.write("\n".join([f"{p[0]} {p[1]} {t[0]} {t[1]}" for (p, t) in data]))
        print(f"Saved to {f.name}")

def load_from_file(file_name: str | None):
    global trajectory, points, tangents
    file_name = file_name or fd.askopenfilename(filetypes=[("Trajectories", "*.traj")])
    if not file_name:
        return

    with open(file_name, "r") as f:
        text = f.read()
        data = [map(lambda s: float(s), p.split(" ")) for p in text.splitlines()]
        points.clear()
        tangents.clear()
        for [p1, p2, t1, t2] in data:
            points.append((p1, p2))
            tangents.append((t1, t2))
        update_plot()
        print(f"Opened {f.name}")

####################################################################################################
def inverse_kinematics(x, y, is_up):
    theta = math.acos(max(-1.0, min(x / ARM_LENGTH, 1.0)))
    if not is_up:
        theta = -theta
    h = y - math.sin(theta) * ARM_LENGTH
    return [h, theta]

####################################################################################################
def plot_elevator_arm(ax, trajectory, percent_time):

    traj: Trajectory = trajectory.getTrajectory()

    h, theta = trajectory.sample(0.01 * percent_time * traj.totalTime())
    x_e, y_e = ARM_LENGTH * math.cos(theta), h + ARM_LENGTH * math.sin(theta)
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
class ArmElevTrajectory:
    def __init__(self, trajectory: Trajectory, is_up=False):
        self.trajectory: Trajectory = trajectory
        self.arm_direction = []

        # select initial direction base on current director or default of down
        # unless valid in only one direction
        s: Trajectory.State = trajectory.sample(0.0)
        direction = self.get_direction(s)
        if not direction: # valid in both directions
            direction = [0.0, is_up]
        self.arm_direction.append(direction)

        prev_s = s

        # seed with a high slope of the correct sign
        s = trajectory.sample(dt)
        prev_slope = sign(self.slope(s, prev_s)) * 1e+6

        last_transition = None
        for t in np.arange(dt, trajectory.totalTime(), dt):
            s = trajectory.sample(t)
            direction = self.get_direction(s)
            slope = self.slope(s, prev_s)

            if direction and last_transition:# either up or down
                last_transition[1] = direction[1] # set pending transition to the only valid direction
                last_transition = None # clear transition

            else: # valid in both directions
                if not sign(slope) == sign(prev_slope):
                    last_transition = [t, None] # create pending transition
                    self.arm_direction.append(last_transition) # add to transitions

            prev_slope = slope
            prev_s = s

        if last_transition and last_transition[1] is None:
            self.arm_direction.remove(last_transition)

    def getTrajectory(self):
        return self.trajectory

    def slope(self, s, prev_s):
        dx = s.pose.x - prev_s.pose.x
        dy = s.pose.y - prev_s.pose.y
        dy = sign(dy) * max(abs(dy), 1e-6)  # can't be zero
        return dx/dy


    def get_direction(self, sample: Trajectory.State):
        h, theta = inverse_kinematics(sample.pose.x, sample.pose.y, False)
        good_dn = MIN_ELEVATOR <= h <= MAX_ELEVATOR
        h, theta = inverse_kinematics(sample.pose.x, sample.pose.y, True)
        good_up = MIN_ELEVATOR <= h <= MAX_ELEVATOR
        if good_dn and good_up:
            return None
        if good_dn:
            return [sample.t, False]
        return [sample.t, True]

    def get_transition_direction(self, t):
        for dir in self.arm_direction:
            if t >= dir[0]:
                is_up = dir[1]
            else:
                break
        return is_up

    def sample(self, t):
        s  = self.trajectory.sample(t)
        is_up = self.get_transition_direction(t)
        h, theta = inverse_kinematics(s.pose.x, s.pose.y, is_up)
        return h, theta

    def totalTime(self):
        return self.trajectory.totalTime()

####################################################################################################
def slider_redraw(_: float):
    main_plot()
    plt.draw()

####################################################################################################
# main
####################################################################################################

IS_SIMULATION = False

inst = NetworkTableInstance.getDefault()

inst.startClient4("Trajectory Generator")
if not IS_SIMULATION:
    inst.setServerTeam(2046)
else:
    inst.setServer("localhost")

dt = 0.02
config = TrajectoryConfig(
    maxVelocity=2.0,  # meters per second
    maxAcceleration=2.0  # meters per second squared
)
limits = create_limits()
points =    [(-0.1, 0.01), (0.145, 0.3), (0.5, 0.4), (0.2, 1.1), (-0.5, 1.32)]
tangents =  [(-0.1, 0.0),  (0, 0),       (0, 0),     (0, 0),    (-0.25, -0.01)]

selected_point = None
selected_tangent = None
trajectory: ArmElevTrajectory = create_trajectory(points, tangents)

# create chart area
fig, ax = plt.subplots(figsize=(8, 10))
fig.subplots_adjust(bottom=0.2)

def on_close(_):
    save_to_file("last")

# register events
fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('close_event', on_close)

# add time control slider
axtime = fig.add_axes([0.20, 0.1, 0.65, 0.03])
time_slider = Slider(ax=axtime, label="Time (% full)", valmin = 0.0, valmax = 100.0, valinit=0.0)
time_slider.on_changed(slider_redraw)

main_plot()

if os.path.isfile("last.traj"):
    load_from_file("last.traj")
plt.show()