import enum
from typing import Literal, Union


class FrameId(enum.Enum):
    map = "map"
    base_link = "base_link"
    body = "body"
    navigate_target = "navigate_target"
    setpoint = "setpoint"
    main_camera_optical = "main_camera_optical"

    aruco_map = "aruco_map"

    @staticmethod
    def aruco(aruco_id: int):
        return f"aruco_{aruco_id}"


FRAME_IDS = Literal[
    "map",
    "base_link",
    "body",
    "navigate_target",
    "setpoint",
    "main_camera_optical",
    "aruco_map",
]

FRAME_IDS_TYPE = Union[FrameId, FRAME_IDS, str]


class LedEffect(enum.Enum):
    fill = "fill"
    blink = "blink"
    blink_fast = "blink_fast"
    fade = "fade"
    wipe = "wipe"
    flash = "flash"
    rainbow = "rainbow"
    rainbow_fill = "rainbow_fill"


LED_EFFECTS = Literal[
    "fill",
    "blink",
    "blink_fast",
    "fade",
    "wipe",
    "flash",
    "rainbow",
    "rainbow_fill",
]

LED_EFFECTS_TYPE = Union[LedEffect, LED_EFFECTS]