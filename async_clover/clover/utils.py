import math
import anyio
from typing import Optional, Union, Type

from . services import CloverFlightServices, CloverLedServices
from . enums import FrameId, FRAME_IDS_TYPE

import rospy


async def wait_arrival(
        tolerance: Optional[float] = None,
        interval: Optional[float] = 0.2,
):
    while not rospy.is_shutdown():
        telemetry = await CloverFlightServices.get_telemetry(frame_id='navigate_target')
        if math.hypot(telemetry.x, telemetry.y, telemetry.z) < tolerance:
            break
        await anyio.sleep(interval)


class CALCULATE_TIMEOUT:
    pass


async def navigate_wait(
        x: float = 0,
        y: float = 0,
        z: float = 0,
        yaw: float = float('nan'),
        frame_id: FRAME_IDS_TYPE = FrameId.body,
        speed: Optional[float] = None,
        auto_arm: bool = False,
        tolerance: Optional[float] = None,
        interval: Optional[float] = 0.2,
        timeout: Union[Optional[float], Type[CALCULATE_TIMEOUT]] = CALCULATE_TIMEOUT,
        stop_on_cancel: bool = True,
):
    await CloverFlightServices.navigate(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        frame_id=frame_id,
        speed=speed,
        auto_arm=auto_arm,
    )

    if timeout is CALCULATE_TIMEOUT:
        telemetry = await CloverFlightServices.get_telemetry(frame_id='navigate_target')
        timeout = (math.hypot(telemetry.x, telemetry.y, telemetry.z) / speed) * 1.15

    try:
        with anyio.fail_after(timeout):
            await wait_arrival(tolerance=tolerance, interval=interval)
    except (TimeoutError, anyio.get_cancelled_exc_class()):
        if stop_on_cancel:
            with anyio.CancelScope(shield=True):
                await CloverFlightServices.navigate(x=0, y=0, z=0, frame_id=FrameId.body, speed=speed)
        raise
