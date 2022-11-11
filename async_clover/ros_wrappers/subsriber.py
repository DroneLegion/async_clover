import attr
import rospy
import logging
from functools import partial
from typing import Optional, Generic, TypeVar, Type, Callable, Any

import anyio

from . utils import get_message, Callback

logger = logging.getLogger(__name__)

MSG = TypeVar('MSG')


@attr.define()
class AsyncSubscriber(Generic[MSG]):
    name: str = attr.field()
    message_class: Type[MSG] = attr.field()
    buffer_size: int = attr.field(default=1)
    callback: Optional[Callable[[MSG], Any]] = attr.field(default=None)

    _receive_channel = attr.field(default=None)
    _send_channel = attr.field(default=None)

    _subscriber: rospy.Subscriber = attr.field(default=None)
    _closed: bool = attr.field(default=False)

    _portal: anyio.from_thread.BlockingPortal = attr.field(factory=anyio.from_thread.BlockingPortal)

    def __attrs_post_init__(self):
        self._send_channel, self._receive_channel = anyio.create_memory_object_stream(self.buffer_size)

    def __str__(self):
        return f"{self.__class__.__name__}: {self.name}"

    @property
    def closed(self):
        return self._closed

    async def subscribe(self, timeout: Optional[float] = 10):
        if self._subscriber is not None:
            raise RuntimeError("Already subscribed")

        logger.info(f"Waiting until topic '{self.name}' is available.")
        message = await get_message(self.name, self.message_class, timeout)
        logger.info(f"Topic '{self.name}' is available, subscribing.")

        await self._portal.__aenter__()

        self._subscriber = rospy.Subscriber(
            self.name,
            self.message_class,
            self._callback,
        )
        return message

    def _check_subscription(self):
        if self._subscriber is None:
            raise RuntimeError("Not subscribed yet")

    async def unsubscribe(self):
        self._check_subscription()

        if self._closed:
            raise RuntimeError("Already unsubscribed")

        await anyio.to_thread.run_sync(self._subscriber.unregister)

        await self._portal.__aexit__(None, None, None)
        self._receive_channel.close()
        self._send_channel.close()

        self._closed = True

    async def get(self) -> MSG:
        self._check_subscription()
        return await self._receive_channel.receive()

    def get_nowait(self, suppress_errors=False) -> Optional[MSG]:
        self._check_subscription()

        result = None
        try:
            result = self._receive_channel.receive_nowait()
        except anyio.WouldBlock:
            if not suppress_errors:
                raise

        return result

    def __aiter__(self):
        self._check_subscription()
        return self

    async def __anext__(self) -> MSG:
        try:
            return await self._receive_channel.receive()
        except anyio.ClosedResourceError:
            raise StopAsyncIteration

    def _convert(self, data):
        return data

    def _callback(self, data):
        data = self._portal.call(self._convert, data)
        logger.debug(f"{self}: received data {data}")
        if self.callback is not None:
            self._portal.call(self.callback, data)

        try:
            self._portal.call(self._send_channel.send_nowait, data)
        except anyio.WouldBlock:
            # drop the leftmost (oldest) item and put new stuff at the right
            old_data = self._portal.call(self._receive_channel.receive_nowait)
            logger.debug(f"{self}: receive buffer is full, dropping old data {old_data}")
            self._portal.call(self._send_channel.send_nowait, data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    from sensor_msgs.msg import Range

    async def main():
        rospy.init_node("atest")
        rangefinder: AsyncSubscriber[Range] = AsyncSubscriber("rangefinder/range", Range)
        print("CREATED")
        await rangefinder.subscribe()
        print("SUB")

        # with anyio.fail_after(10):  # wait for up to 10 seconds
        #     print(await rangefinder.get())
        # await anyio.sleep(10)
        # r = await rangefinder.get()
        # print("DATA", r.range)

        # print(rangefinder.get_nowait())
        # print("CYCLE")
        #
        try:
            with anyio.fail_after(10):
                async for range_data in rangefinder:
                    print(range_data.range)
        except TimeoutError:
            pass
        # async with trio.open_nursery() as nursery:
        #     nursery.start_soon(camera)
        #     nursery.start_soon(flight)
        await rangefinder.unsubscribe()
        print("ALL DONE")

    anyio.run(main)
