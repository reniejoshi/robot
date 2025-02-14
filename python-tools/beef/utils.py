# Bounded Timer

from time import time as now

from wpimath.trajectory import Trajectory


class BoundedTimer:
    """A timer whose value saturates at bounds."""

    # Member Variables

    current_time: float = 0.0
    """ The current time. """

    min_time: float = 0.0
    """ The minimum time. """
    max_time: float
    """ The maximum time. """

    last_rt_update_time: float | None = None
    """The last update time in seconds for real-time updates."""

    scale: float = 1

    # Initialization

    def __init__(
        self,
        max_time: float,
        current_time: float | None = None,
        min_time: float | None = None,
    ):
        self.min_time = min_time or 0.0
        self.max_time = max_time

        self.set(current_time or self.min_time)  # Sync the current_time or min_time

    # Setters

    def reset_rt(self):
        self.last_rt_update_time = now()

    def advance_rt(self):
        """Updates the timer by the time since the last call to this method."""

        time = now()
        if self.last_rt_update_time:
            dt = time - self.last_rt_update_time
            self.advance(dt)

        self.last_rt_update_time = time

    def advance(self, dt: float):
        """Advances the timer.

        :param dt: The delta time in seconds
        """

        self.set(self.current_time + dt * self.scale)

    def set(self, time: float):
        """Sets the time of the timer.

        :param time: The time in seconds
        """

        self.current_time = max(self.min_time, min(time, self.max_time))

    def set_max(self, max_time: float):
        self.max_time = max_time

    def set_min(self, min_time: float):
        self.min_time = min_time

    def set_scale(self, scale: float):
        self.scale = scale

    def reset(self):
        self.set(self.min_time)

    # Getters
    #   - No need for epsilons as the value saturates at it's bounds

    def get(self) -> float:
        return self.current_time

    def at_max(self) -> bool:
        return self.current_time == self.max_time

    def at_min(self) -> bool:
        return self.current_time == self.min_time


# Tag

import dearpygui.dearpygui as dpg


def last_tag(use_registry: bool = True):
    return Tag(dpg.last_item(), use_registry)

class Tag:
    """A dearpygui helper class to make working with objects easier."""

    tag: int | str
    """ A unique-id referring to a dearpygui object. """

    handler_registry: int | str | None = None

    # Initialization

    def __init__(self, tag: int | str, use_registry: bool = True):
        self.tag = tag
        if use_registry:
            self.handler_registry = dpg.add_item_handler_registry()
            dpg.bind_item_handler_registry(self.tag, self.handler_registry)

    # Context

    @dpg.contextmanager
    def handler_context(self):
        assert self.handler_registry
        try:
            dpg.push_container_stack(self.handler_registry)
            yield self.handler_registry
        finally:
            dpg.pop_container_stack()

    def __enter__(self):
        dpg.push_container_stack(self.tag)

    def __exit__(self, *_):
        dpg.pop_container_stack()


# Maths

def state_to_xy(state: Trajectory.State) -> list[float]:
    return [state.pose.x, state.pose.y]

def lerp(a, b, t):
    return (1-t) * a + t * b