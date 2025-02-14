import dearpygui.dearpygui as dpg
from catppuccin import PALETTE

from utils import *

from pathlib import Path

PATH_PREFIX = Path(__file__).parent.__str__()

# Constants

FONT_SIZE = 16
FLAVOR = PALETTE.mocha

# Functions


def get_color(color: str, alpha=255) -> tuple[float, ...]:
    """Get a color from the selected Catppuccin flavor."""
    rgb = FLAVOR.colors.__dict__[color].rgb
    return rgb.r, rgb.g, rgb.b, alpha


def configure_global_fonts():
    with dpg.font_registry():
        # Load font and register it as the global font
        default_font = dpg.add_font(
            PATH_PREFIX + "/fonts/0xProto-Regular.ttf", FONT_SIZE
        )
        dpg.bind_font(default_font)


def configure_global_theme():
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, get_color("base"))
            dpg.add_theme_color(dpg.mvThemeCol_MenuBarBg, get_color("base"))
            dpg.add_theme_color(dpg.mvThemeCol_Text, get_color("lavender"))

    # Bind the theme globally
    dpg.bind_theme(theme)


def configure_plot_theme(plot_tag: Tag):
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvPlot):
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, [0, 0, 0, 0])

    # Bind the theme to the plot
    dpg.bind_item_theme(plot_tag.tag, theme)


def set_plot_line_color(tag: Tag, color: str, alpha=255):
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(
                dpg.mvPlotCol_Line,
                get_color(color, alpha),
                category=dpg.mvThemeCat_Plots,
            )
    dpg.bind_item_theme(tag.tag, theme)


def global_theme():
    configure_global_fonts()
    configure_global_theme()

    # Hi-DPI
    import ctypes, sys

    if sys.platform.startswith("win"):
        ctypes.windll.shcore.SetProcessDpiAwareness(4)
