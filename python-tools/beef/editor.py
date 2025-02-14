import math

from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
from wpimath.spline import Spline3
from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters

from wpilib import SmartDashboard
from ntcore import NetworkTableInstance

import dearpygui.dearpygui as dpg

dpg.create_context()
dpg.setup_dearpygui()

from theming import *
from utils import *

global_theme()

IS_SIMULATION = False

# Robot

# NOTE: MAKE SURE THESE ARE UP-TO-DATE

ARM_LENGTH = 0.6540246

CENTER_X_TO_ELEVATOR = -inchesToMeters(1)
PIVOT_TO_BOTTOM_OF_FIRST_STAGE = -0.1906
PIVOT_TO_GROUND = -0.3868

ELEVATOR_MAX_POSE = 1
ELEVATOR_MIN_POSE = 0.01

END_EFFECTOR_MIN_POSE = -0.21

COLLECTOR_STOWED_MIN_X = 0.23 + CENTER_X_TO_ELEVATOR
COLLECTOR_STOWED_HEIGHT = 0.6531 + PIVOT_TO_GROUND

WHEEL_BASE_FWD = 0.527 / 2 - CENTER_X_TO_ELEVATOR  # between steer pivots
WHEEL_BASE_REV = -0.527 / 2 - CENTER_X_TO_ELEVATOR  # between steer pivots
WHEEL_PIVOT = 0.050 + PIVOT_TO_GROUND
WHEEL_RADIUS = inchesToMeters(3.95) / 2

BUMPER_FWD = 0.426 - CENTER_X_TO_ELEVATOR
BUMPER_REV = -0.426 - CENTER_X_TO_ELEVATOR
BUMPER_MIN_Y = 0.024 + PIVOT_TO_GROUND  # floor to bottom of bumpers
BUMPER_MAX_Y = 0.154 + PIVOT_TO_GROUND  # floor to top of bumpers

ELEVATOR_X_FWD = 0.1270
ELEVATOR_X_REV = -0.1270
ELEVATOR_HEIGHT = 0.8255 + PIVOT_TO_GROUND


# Math


def inverse_kinematics(x: float, y: float, orientation: str) -> tuple[float, float]:
    theta = math.acos(x / ARM_LENGTH)
    if orientation == "down":
        theta = -theta
    h = y - math.sin(theta) * ARM_LENGTH
    return h, theta


def ik_in_limits(h: float, theta: float) -> bool:
    return ELEVATOR_MIN_POSE <= h <= ELEVATOR_MAX_POSE


def end_effector_in_limits(x: float, y: float) -> bool:
    return (
        not (x > COLLECTOR_STOWED_MIN_X and y < COLLECTOR_STOWED_HEIGHT)
        and (-ARM_LENGTH <= x <= ARM_LENGTH)
        and (
            END_EFFECTOR_MIN_POSE
            <= y
            <= (math.tan(math.acos(x / ARM_LENGTH)) * x + ELEVATOR_MAX_POSE)
        )
    )


def can_switch_directions(trajectory: Trajectory, time: float) -> bool:
    s = trajectory.sample(time)
    pt = state_to_xy(s)

    return (
        math.fabs(pt[0]) > ARM_LENGTH - 5e-3
        and math.fabs(s.pose.rotation().sin()) > 0.99985
    )


def verify_trajectory(
    trajectory: Trajectory, initial_direction: str, preferred_direction: str
) -> list[tuple[float, str]] | None:
    pd = initial_direction
    ds = [(0, initial_direction)]
    for t in range(0, int(200 * trajectory.totalTime())):
        t = float(t) / 200
        s = trajectory.sample(t)
        pt = state_to_xy(s)

        # End-effector
        if not end_effector_in_limits(*pt):
            return None

        # Elevator
        directions = directions_for_sample(*pt)
        dir_h_theta = dir_h_theta_from_directions(directions, pd, preferred_direction)
        if not dir_h_theta:
            return None

        if pd != dir_h_theta[0] and can_switch_directions(trajectory, t):
            pd = dir_h_theta[0]
            ds.append([t, pd])

        # Arm
        for t2 in range(0, 100):
            x = lerp(0, pt[0], float(t2) / 100)
            y = lerp(dir_h_theta[1], pt[1], float(t2) / 100)
            if not end_effector_in_limits(x, y):
                return None
    return ds


def get_velocity_from_sample(sample: Trajectory.State) -> tuple[float, float]:
    if sample.t < 0:
        return 0, 0
    return (
        sample.pose.rotation().cos() * sample.velocity,
        sample.pose.rotation().sin() * sample.velocity,
    )


def directions_at_time(
    trajectory: Trajectory, time: float
) -> dict[str, tuple[float, float]]:
    sample = trajectory.sample(time)
    return directions_for_sample(sample.pose.x, sample.pose.y)


def directions_for_sample(x: float, y: float) -> dict[str, tuple[float, float]]:
    up = inverse_kinematics(x, y, "up")
    down = inverse_kinematics(x, y, "down")

    is_valid_up = ik_in_limits(*up)
    is_valid_down = ik_in_limits(*down)

    out = {}
    if is_valid_up:
        out["up"] = up
    if is_valid_down:
        out["down"] = down
    return out


def dir_h_theta_from_directions(
    directions: dict[str, tuple[float, float]],
    previous_direction: str | None,
    preferred_direction: str,
) -> tuple[str, float, float]:
    if not "up" in directions and not "down" in directions:
        return None
    elif not "up" in directions:
        d = "down"
        ht = directions["down"]
    elif not "down" in directions:
        d = "up"
        ht = directions["up"]
    elif preferred_direction == "up":
        d = "up"
        ht = directions["up"]
    else:
        d = "down"
        ht = directions["down"]

    return d, ht[0], ht[1]


# Application


class Editor:
    # State

    point_indices = {}

    points: list[list[list[float, float]]] = []

    config = TrajectoryConfig(0.5, 0.5)  # TODO: Derive from h, theta limits

    trajectory: Trajectory | None = None
    initial_direction = "up"
    prefered_direction = "up"
    directions: list[tuple[float, str]] | None = None

    # Animation

    playing = False
    timer = BoundedTimer(max_time=0.0)

    # Initialization

    debug = False
    save_shown = False
    open_shown = False

    def __init__(self):
        # TODO: Load previous trajectory

        # UI

        # - Window

        self.ui_window = Tag(dpg.add_window(), use_registry=False)
        self.ui_debug_window = Tag(
            dpg.add_window(show=self.debug, width=400), use_registry=False
        )

        # - Dialogs

        self.ui_save_dialog = Tag(
            dpg.add_file_dialog(
                show=False,
                default_path=".",
                default_filename=".traj",
                width=500,
                height=350,
                callback=self.save,
                directory_selector=False,
                cancel_callback=self.toggle_save_dialog,
            )
        )
        self.ui_open_dialog = Tag(
            dpg.add_file_dialog(
                show=False,
                width=500,
                height=350,
                callback=self.open,
                file_count=1000,
                directory_selector=False,
                cancel_callback=self.toggle_open_dialog,
            )
        )

        with self.ui_open_dialog:
            dpg.add_file_extension(
                "Trajectories (*.traj){.traj}", color=get_color("yellow")
            )

        # - Debug

        with self.ui_debug_window:
            self.ui_debug_information = Tag(dpg.add_text())

        # - Plot

        with self.ui_window:
            self.ui_plot = Tag(dpg.add_plot(**self.plot_widget_configuration))
            configure_plot_theme(self.ui_plot)

        with self.ui_plot:
            self.ui_plot_axis_x = Tag(
                dpg.add_plot_axis(
                    axis=dpg.mvXAxis, label="x (meters)", **self.plot_axis_configuration
                )
            )
            self.ui_plot_axis_y = Tag(
                dpg.add_plot_axis(
                    axis=dpg.mvYAxis, label="z (meters)", **self.plot_axis_configuration
                )
            )

        with self.ui_plot_axis_y:
            self.ui_plot_ground = Tag(
                dpg.add_inf_line_series(x=[PIVOT_TO_GROUND], horizontal=True)
            )

        # - Windmill
        with self.ui_plot:
            self.ui_trajectory = Tag(
                dpg.draw_polyline([], thickness=self.plot_line_thickness_meters)
            )
            self.ui_ghost_trajectory = Tag(
                dpg.draw_polyline(
                    [],
                    thickness=self.plot_line_thickness_meters,
                    color=[255, 0, 0, 100],
                )
            )

            self.ui_elevator = Tag(
                dpg.draw_circle(
                    center=[0, 0],
                    radius=0.1016 / 2,
                    thickness=self.plot_line_thickness_meters,
                    color=get_color("blue"),
                )
            )
            self.ui_end_effector = Tag(
                dpg.draw_circle(
                    center=[0, 0],
                    radius=0.03,
                    thickness=self.plot_line_thickness_meters,
                    color=get_color("blue"),
                )
            )
            self.ui_end_effector_velocity = Tag(
                dpg.draw_arrow(
                    [0, 0],
                    [0, 0],
                    size=0.02,
                    thickness=0.005,
                    color=get_color("green"),
                )
            )
            self.ui_end_effector_acceleration = Tag(
                dpg.draw_arrow(
                    [0, 0],
                    [0, 0],
                    size=0.02,
                    thickness=0.005,
                    color=get_color("peach"),
                )
            )
            self.ui_arm = Tag(
                dpg.draw_line(
                    p1=[0, 0],
                    p2=[0, 0],
                    thickness=self.plot_line_thickness_meters,
                    color=get_color("blue"),
                )
            )

        # - Action Bar

        with self.ui_window:
            self.ui_action_bar_wrapper = Tag(
                dpg.add_table(
                    header_row=False,
                    borders_innerH=False,
                    borders_outerH=False,
                    borders_innerV=False,
                    borders_outerV=False,
                )
            )

        with self.ui_action_bar_wrapper:
            dpg.add_table_column(no_resize=True, init_width_or_weight=2)
            dpg.add_table_column(no_resize=True, init_width_or_weight=2)
            dpg.add_table_column(no_resize=True, init_width_or_weight=1)

            self.ui_action_bar = Tag(dpg.add_table_row())

        with self.ui_action_bar:
            self.ui_trajectory_controls = Tag(dpg.add_group(horizontal=True))
            self.ui_animation_controls = Tag(dpg.add_group(horizontal=True))
            self.ui_time_scale_controls = Tag(dpg.add_group(horizontal=True))

        with self.ui_trajectory_controls:
            dpg.add_button(label="Save", callback=self.toggle_save_dialog)
            dpg.add_button(label="Open", callback=self.toggle_open_dialog)
            dpg.add_text("Initial Direction")
            self.ui_initial_direction = dpg.add_combo(
                items=["up", "down"],
                default_value="up",
                width=75,
                callback=self.set_initial_direction,
            )
            dpg.add_text("Preferred Direction")
            self.ui_preferred_direction = dpg.add_combo(
                items=["up", "down"],
                default_value="up",
                width=75,
                callback=self.set_preferred_direction,
            )

        with self.ui_animation_controls:
            dpg.add_text("Time (s)")
            self.ui_play_button = dpg.add_button(
                label="Play", callback=self.toggle_playing
            )
            self.ui_time_slider = dpg.add_slider_float(
                clamped=True,
                min_value=0,
                max_value=0,
                width=-1,
                callback=lambda _, t: self.set_time(t),
            )

        with self.ui_time_scale_controls:
            dpg.add_text("Time Scale")
            self.ui_time_scale_slider = dpg.add_slider_float(
                clamped=True,
                min_value=-1,
                max_value=1,
                default_value=1,
                width=-1,
                callback=lambda _, s: self.set_time_scale(s),
            )

        self.draw_robot()
        self.draw_end_effector_limits()

        # Register event handlers
        self.register_global_handlers()
        self.register_item_handlers()

        # Fill the viewport with the entire editor window
        dpg.set_primary_window(self.ui_window.tag, True)

    def register_global_handlers(self):
        dpg.set_viewport_resize_callback(self.handle_viewport_resize)

        with dpg.handler_registry():
            dpg.add_key_release_handler(
                dpg.mvKey_Spacebar, callback=self.toggle_playing
            )
            dpg.add_key_release_handler(dpg.mvKey_Tilde, callback=self.toggle_debug)

    def register_item_handlers(self):
        with self.ui_plot.handler_context():
            dpg.add_item_clicked_handler(button=0, callback=self.handle_plot_left_click)

    # Drawing

    def draw_trajectory(self):
        if not self.points or len(self.points) < 2:
            self.trajectory = None
            return

        [points, tangents] = zip(*self.points)
        start = Spline3.ControlVector(
            (points[0][0], tangents[0][0]), (points[0][1], tangents[0][1])
        )
        waypoints = []
        if len(points) > 2:
            for i in range(1, len(points) - 1):
                waypoints.append(Translation2d(points[i][0], points[i][1]))
        end = Spline3.ControlVector(
            (points[-1][0], tangents[-1][0]), (points[-1][1], tangents[-1][1])
        )

        trajectory = TrajectoryGenerator.generateTrajectory(
            start, waypoints, end, self.config
        )
        pts = [
            state_to_xy(trajectory.sample(float(t) / 200))
            for t in range(0, int(200 * trajectory.totalTime()))
        ]
        directions = verify_trajectory(
            trajectory, self.initial_direction, self.prefered_direction
        )

        if not directions:
            dpg.configure_item(self.ui_ghost_trajectory.tag, **{"points": pts})
            return
        else:
            dpg.configure_item(self.ui_ghost_trajectory.tag, **{"points": []})

        self.directions = directions
        self.trajectory = trajectory
        self.update_duration()

        self.publish()

        dpg.configure_item(self.ui_trajectory.tag, **{"points": pts})
        self.draw_windmill()

    def draw_windmill(self):
        sample = self.trajectory.sample(self.timer.get())
        pose = sample.pose

        h, _ = self.inverse_kinematics_using_direction(self.timer.get())

        if not self.timer.at_max() and self.debug:
            previous_velocity = get_velocity_from_sample(
                self.trajectory.sample(self.timer.get() - 1e-12)
            )
            velocity = get_velocity_from_sample(sample)
            acceleration = (
                (velocity[0] - previous_velocity[0]) / 1e-12,
                (velocity[1] - previous_velocity[1]) / 1e-12,
            )

            dpg.configure_item(
                self.ui_end_effector_velocity.tag,
                **{
                    "p2": (pose.x, pose.y),
                    "p1": (pose.x + velocity[0], pose.y + velocity[1]),
                },
            )
            dpg.configure_item(
                self.ui_end_effector_acceleration.tag,
                **{
                    "p2": (pose.x, pose.y),
                    "p1": (pose.x + acceleration[0], pose.y + acceleration[1]),
                },
            )

        dpg.configure_item(self.ui_end_effector.tag, **{"center": (pose.x, pose.y)})
        dpg.configure_item(self.ui_elevator.tag, **{"center": (0, h)})
        dpg.configure_item(self.ui_arm.tag, **{"p1": (0, h), "p2": (pose.x, pose.y)})

    # Handlers

    def handle_plot_left_click(self):
        if dpg.is_key_down(dpg.mvKey_ModShift):
            self.add_drag_point(dpg.get_plot_mouse_pos())

    def add_drag_point(self, pos, tan=[0, 0.01]):
        tag = dpg.add_drag_point(
            parent=self.ui_plot.tag,
            color=get_color("lavender"),
            default_value=pos,
            show_label=True,
            label=len(self.points).__str__(),
            callback=self.handle_point_drag,
        )
        self.point_indices[tag] = len(self.points)
        self.points.append([pos, tan])
        self.draw_trajectory()

    def handle_point_drag(self, sender):
        pos = dpg.get_value(sender)
        pos[0] = math.copysign(max(0.0, min(math.fabs(pos[0]), ARM_LENGTH)), pos[0])
        if pos[1] < END_EFFECTOR_MIN_POSE: pos[1] = END_EFFECTOR_MIN_POSE
        self.points[self.point_indices[sender]][0] = pos
        dpg.set_value(sender, pos)
        self.draw_trajectory()

    def handle_viewport_resize(self, _, app_data):
        dpg.configure_item(self.ui_plot.tag, **{"height": app_data[3] - 75})

    # Callbacks

    def set_initial_direction(self, _, direction):
        self.initial_direction = direction
        self.draw_trajectory()

    def set_preferred_direction(self, _, direction):
        self.prefered_direction = direction
        self.draw_trajectory()

    def toggle_debug(self):
        self.debug = not self.debug
        if self.debug:
            dpg.show_item(self.ui_debug_window.tag)
            dpg.show_item(self.ui_end_effector_velocity.tag)
            dpg.show_item(self.ui_end_effector_acceleration.tag)
        else:
            dpg.hide_item(self.ui_debug_window.tag)
            dpg.hide_item(self.ui_end_effector_velocity.tag)
            dpg.hide_item(self.ui_end_effector_acceleration.tag)

    def toggle_playing(self):
        self.set_playing(not self.playing, reset=True)

    def set_playing(self, playing, reset=False):
        if not self.trajectory:
            return

        if reset and self.timer.at_max():
            self.timer.reset()

        self.playing = playing
        self.timer.reset_rt()
        dpg.set_item_label(self.ui_play_button, "Pause" if self.playing else "Play")

    def update_duration(self):
        dur = self.trajectory.totalTime()
        self.timer.set_max(dur)
        dpg.configure_item(self.ui_time_slider, **{"max_value": dur})

    def set_time_scale(self, time_scale):
        self.timer.set_scale(time_scale)

    def set_time(self, time):
        """
        Sets the time of the trajectory simulation.

        - `time` is clamped to the trajectory's duration.

        :param time: The time in seconds
        """

        assert self.trajectory is not None

        self.timer.set(time)
        self.sync_time()

    def sync_time(self):
        if self.trajectory:
            self.draw_windmill()
        dpg.set_value(self.ui_time_slider, self.timer.get())

    def toggle_save_dialog(self):
        if self.save_shown:
            dpg.hide_item(self.ui_save_dialog.tag)
        else:
            dpg.show_item(self.ui_save_dialog.tag)
        self.save_shown = not self.save_shown

    def toggle_open_dialog(self):
        if self.open_shown:
            dpg.hide_item(self.ui_open_dialog.tag)
        else:
            dpg.show_item(self.ui_open_dialog.tag)
        self.open_shown = not self.open_shown

    def save(self, _, data):
        with open(data["file_path_name"] + ".traj", "w") as f:
            f.write(
                "\n".join([f"{p[0]} {p[1]} {t[0]} {t[1]}" for (p, t) in self.points])
            )
            print(f"Saved to {f.name}")
        self.toggle_save_dialog()

    def open(self, _, data):
        with open(data["file_path_name"], "r") as f:
            text = f.read()
            data = [map(lambda s: float(s), p.split(" ")) for p in text.splitlines()]
            self.points.clear()
            for drag in self.point_indices.keys():
                dpg.delete_item(drag)
            for [p1, p2, t1, t2] in data:
                self.add_drag_point((p1, p2), (t1, t2))

        self.draw_trajectory()
        print(f"Opened {f.name}")
        self.toggle_open_dialog()

    def publish(self):
        SmartDashboard.putNumber("BEEF/Config/Max Velocity", self.config.maxVelocity())
        SmartDashboard.putNumber(
            "BEEF/Config/Max Acceleration", self.config.maxAcceleration()
        )

        directions_count = len(self.directions) if self.directions else 0
        SmartDashboard.putNumber("BEEF/Directions/Count", directions_count)
        for i in range(0, directions_count):
            t, d = self.directions[i]
            SmartDashboard.putNumber(f"BEEF/Directions/{i}/Time", t)
            SmartDashboard.putString(f"BEEF/Directions/{i}/Direction", d)

        points_count = len(self.points) if self.points else 0
        SmartDashboard.putNumber("BEEF/Points/Count", points_count)
        for i in range(0, points_count):
            pt, tan = self.points[i]
            SmartDashboard.putNumber(f"BEEF/Points/{i}/Position/x", pt[0])
            SmartDashboard.putNumber(f"BEEF/Points/{i}/Position/y", pt[1])
            SmartDashboard.putNumber(f"BEEF/Points/{i}/Tangent/x", tan[0])
            SmartDashboard.putNumber(f"BEEF/Points/{i}/Tangent/y", tan[1])

        hash = (
            self.config.maxVelocity().__str__()
            + self.config.maxAcceleration().__str__()
            + self.directions.__str__()
            + self.points.__str__()
        )
        SmartDashboard.putString("BEEF/Hash", hash)

    # Update

    def inverse_kinematics_using_direction(self, time: float) -> tuple[float, float]:
        sample = self.trajectory.sample(time)
        direction: str | None = None
        for i in range(0, len(self.directions) + 1):
            if i == len(self.directions) or self.directions[i][0] > time:
                direction = self.directions[i - 1][1]
                break
        return inverse_kinematics(sample.pose.x, sample.pose.y, direction)

    def update(self):
        if not inst.isConnected():
            inst.startClient4("BEEF [Beef's Editor (Extra Fancy!!)]")
            self.publish()

        if self.playing and self.trajectory:
            if self.timer.at_max():
                self.set_playing(False)

            self.timer.advance_rt()
            self.sync_time()

        if self.debug and self.trajectory:  # ~
            t = self.timer.get()
            ef = dpg.get_item_configuration(self.ui_end_effector.tag)["center"]
            dirs = directions_for_sample(*ef)
            h, theta = self.inverse_kinematics_using_direction(t)

            if not self.directions:
                return
            direction: str | None = None
            for i in range(0, len(self.directions) + 1):
                if i == len(self.directions) or self.directions[i][0] > t:
                    direction = self.directions[i - 1][1]
                    break

            dpg.set_value(
                self.ui_debug_information.tag,
                f"Debug:\n"
                + f"\tPoints: {self.points}\n"
                + f"\tDirections: {self.directions}\n"
                + f"\tTime: {math.trunc(t * 1000) / 1000}\n"
                + f"\tEnd Effector: ({math.trunc(ef[0] * 1000) / 1000}, {math.trunc(ef[1] * 1000) / 1000})\n"
                + f"\tHeight / Theta: ({math.trunc(h * 1000) / 1000}, {math.trunc(t * 1000) / 1000})\n"
                + f"\tCan Switch Directions: {can_switch_directions(self.trajectory, t)}\n"
                + f"\tDirection: {direction}\n"
                + f"\tValid Directions: {dirs}",
            )

    # Configuration

    plot_widget_configuration = {
        "label": "Trajectory",
        "equal_aspects": True,
        "no_menus": True,
        "width": -1,
        "height": -1,
    }
    plot_axis_configuration = {"no_side_switch": True, "no_highlight": True}

    plot_line_thickness_meters = 0.002

    # Limits

    def draw_end_effector_limits(self):
        x = []
        y = []

        # Physical Limitations
        x.append(-ARM_LENGTH)
        y.append(END_EFFECTOR_MIN_POSE)

        x.append(COLLECTOR_STOWED_MIN_X)
        y.append(END_EFFECTOR_MIN_POSE)

        x.append(COLLECTOR_STOWED_MIN_X)
        y.append(COLLECTOR_STOWED_HEIGHT)

        x.append(ARM_LENGTH)
        y.append(COLLECTOR_STOWED_HEIGHT)

        x.append(ARM_LENGTH)
        y.append(ELEVATOR_MAX_POSE)

        # Max extension
        for t in range(0, 100):
            t = float(t) / 100 * math.pi
            x.append(math.cos(t) * ARM_LENGTH)
            y.append(math.sin(t) * ARM_LENGTH + ELEVATOR_MAX_POSE)

        x.append(-ARM_LENGTH)
        y.append(ELEVATOR_MAX_POSE)

        x.append(-ARM_LENGTH)
        y.append(END_EFFECTOR_MIN_POSE)

        with self.ui_plot_axis_y:
            set_plot_line_color(Tag(dpg.add_line_series(x, y)), "red", 127)

    def draw_robot(self):
        with self.ui_plot_axis_y:
            # Bumpers
            set_plot_line_color(
                Tag(
                    dpg.add_line_series(
                        [BUMPER_REV, BUMPER_FWD, BUMPER_FWD, BUMPER_REV, BUMPER_REV],
                        [
                            BUMPER_MAX_Y,
                            BUMPER_MAX_Y,
                            BUMPER_MIN_Y,
                            BUMPER_MIN_Y,
                            BUMPER_MAX_Y,
                        ],
                    )
                ),
                "green",
                127,
            )

        with self.ui_plot:
            # Elevator Top
            dpg.draw_circle(
                (0, ELEVATOR_MAX_POSE),
                0.03,
                color=get_color("red", 127),
            )

            # Wheels
            dpg.draw_circle(
                (WHEEL_BASE_FWD, WHEEL_PIVOT),
                WHEEL_RADIUS,
                color=get_color("text", 127),
            )
            dpg.draw_circle(
                (WHEEL_BASE_REV, WHEEL_PIVOT),
                WHEEL_RADIUS,
                color=get_color("text", 127),
            )
            dpg.draw_arrow(
                (0.1 - CENTER_X_TO_ELEVATOR, (BUMPER_MAX_Y + BUMPER_MIN_Y) / 2),
                (-0.1 - CENTER_X_TO_ELEVATOR, (BUMPER_MAX_Y + BUMPER_MIN_Y) / 2),
                size=0.02,
                thickness=0.005,
                color=get_color("text"),
            )

        with self.ui_plot_axis_y:
            # Elevator
            set_plot_line_color(
                Tag(
                    dpg.add_line_series(
                        [
                            ELEVATOR_X_REV,
                            ELEVATOR_X_FWD,
                            ELEVATOR_X_FWD,
                            ELEVATOR_X_REV,
                            ELEVATOR_X_REV,
                        ],
                        [
                            ELEVATOR_HEIGHT,
                            ELEVATOR_HEIGHT,
                            BUMPER_MAX_Y,
                            BUMPER_MAX_Y,
                            ELEVATOR_HEIGHT,
                        ],
                    )
                ),
                "yellow",
                127,
            )


editor = Editor()

# Windowing

dpg.create_viewport(title="BEEF [Beef's Editor (Extra Fancy!)]", width=800, height=800)

# Debug
# dpg.show_metrics()
# dpg.show_style_editor()

# from dearpygui.demo import show_demo
# show_demo()

inst = NetworkTableInstance.getDefault()
if not IS_SIMULATION:
    inst.setServerTeam(2046)
else:
    inst.setServer("localhost")

dpg.show_viewport()
while dpg.is_dearpygui_running():
    editor.update()
    dpg.render_dearpygui_frame()
dpg.destroy_context()
