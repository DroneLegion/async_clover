import attr
import enum
from functools import cached_property
from typing import Literal, Union, Optional
from async_clover.ros_wrappers import AsyncService, AsyncSubscriber

from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState


from sensor_msgs.msg import Range


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


class CloverFlightServices:
    get_telemetry: AsyncService[srv.GetTelemetry] = AsyncService("get_telemetry", srv.GetTelemetry)
    navigate: AsyncService[srv.Navigate] = AsyncService("navigate", srv.Navigate)
    navigate_global: AsyncService[srv.NavigateGlobal] = AsyncService("navigate_global", srv.NavigateGlobal)
    set_position: AsyncService[srv.SetPosition] = AsyncService("set_position", srv.SetPosition)
    set_velocity: AsyncService[srv.SetVelocity] = AsyncService("set_velocity", srv.SetVelocity)
    set_attitude: AsyncService[srv.SetAttitude] = AsyncService("set_attitude", srv.SetAttitude)
    set_rates: AsyncService[srv.SetRates] = AsyncService("set_rates", srv.SetRates)

    land: AsyncService[Trigger] = AsyncService("land", Trigger)
    arming: AsyncService[CommandBool] = AsyncService("mavros/cmd/arming", CommandBool)


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


class CloverLedServices:
    set_effect: AsyncService[srv.SetLEDEffect] = AsyncService("led/set_effect", srv.SetLEDEffect)
    set_leds: AsyncService[SetLEDs] = AsyncService("led/set_leds", SetLEDs)


@attr.define()
class Clover:
    node_name: str = attr.field(default="flight")

    tolerance: float = attr.field(default=0.20)
    speed: float = attr.field(default=1.0)

    # rangefinder: None
    # camera_raw: None

    @cached_property
    def rangefinder(self) -> AsyncSubscriber[Range]:
        rangefinder: AsyncSubscriber[Range] = AsyncSubscriber("rangefinder/range", Range)
        return rangefinder

    @cached_property
    def image_raw(self) -> AsyncSubscriber[Range]:
        rangefinder: AsyncSubscriber[Range] = AsyncSubscriber("rangefinder/range", Range)
        return rangefinder

    @cached_property
    def image_raw_throttled(self) -> AsyncSubscriber[Range]:
        rangefinder: AsyncSubscriber[Range] = AsyncSubscriber("rangefinder/range", Range)
        return rangefinder

    async def navigate(
            self,
            x: float = 0,
            y: float = 0,
            z: float = 0,
            yaw: float = float('nan'),
            frame_id: FRAME_IDS_TYPE = FrameId.body,
            speed: Optional[float] = None,
            tolerance: Optional[float] = None,
    ):
        speed = speed or self.speed
        tolerance = tolerance or self.tolerance

    async def takeoff(self, z: float = 1, frame_id: FRAME_IDS_TYPE = FrameId.body):
        pass

    async def land(self):
        pass

    async def arm(self, arming: bool = True):
        pass

    async def disarm(self):
        pass

    async def get_telemetry(self, frame_id: FRAME_IDS_TYPE = FrameId.body):
        pass

    async def get_camera_info(self):
        pass


c = Clover()
c.navigate(frame_id="bo")
