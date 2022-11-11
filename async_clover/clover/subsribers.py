import attr
from typing import Optional, Type
from async_clover.ros_wrappers import AsyncService, AsyncSubscriber

from sensor_msgs.msg import Range


@attr.define()
class RangeSubscriber(AsyncSubscriber[float]):
    name: str = attr.field(default="rangefinder/range")
    message_class: Type[Range] = attr.field(default=Range)
    buffer_size: int = attr.field(default=1)

    min_range = attr.field(default=float('nan'))
    max_range = attr.field(default=float('nan'))

    def _convert(self, data):
        self.max_range = data.max_range
        self.min_range = data.min_range
        return data.range
