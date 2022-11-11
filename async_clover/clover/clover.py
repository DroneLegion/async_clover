import attr
import logging
from functools import cached_property
from typing import Literal, Union, Optional

from async_clover.ros_wrappers import AsyncSubscriber
from async_clover.clover.enums import FrameId, FRAME_IDS_TYPE
from async_clover.clover.services import CloverFlightServices, CloverLedServices

import rospy
from sensor_msgs.msg import Range

logger = logging.getLogger(__name__)


@attr.define()
class Clover:
    node_name: str = attr.field(default="flight")

    tolerance: float = attr.field(default=0.20)
    speed: float = attr.field(default=1.0)

    async def start(self, anonymous: bool = False):
        logger.info(f"Starting node {self.node_name}")
        rospy.start_node(self.node_name, anonymous=anonymous)
        logger.info(f"Node {self.node_name} started")

        logger.info(f"Waiting for services")
        await CloverFlightServices.get_telemetry.connect()
        await CloverFlightServices.navigate.connect()
        await CloverFlightServices.land.connect()
        await CloverFlightServices.arming.connect()
        logger.info(f"Services connected")

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

        await CloverFlightServices.navigate(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            frame_id=frame_id,
            speed=speed,
        )

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
