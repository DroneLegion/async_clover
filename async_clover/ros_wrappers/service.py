import logging
import time
from functools import partial
from typing import Callable, Optional, Generic, TypeVar

import anyio
import attr
import rospy

from . utils import wait_for_service

logger = logging.getLogger(__name__)

MSG = TypeVar('MSG')


@attr.define()
class AsyncService(Generic[MSG]):
    name: str = attr.field()
    service_class: type = attr.field()

    _service_proxy: Callable = attr.field(default=None)

    _lock: anyio.Lock = attr.field(factory=anyio.Lock)

    def __str__(self):
        return f"{self.__class__.__name__}: {self.name}"

    async def connect(self, timeout: Optional[float] = 10):
        await wait_for_service(self.name, timeout)
        logger.info(f"Service '{self.name}' is available, connecting proxy.")
        self._service_proxy = rospy.ServiceProxy(self.name, self.service_class)

    async def call(self, *args, **kwargs) -> MSG:
        return await self.__call__(*args, **kwargs)

    def _call_nowait(self, *args, **kwargs) -> MSG:
        """Unsafe!"""
        logger.debug(f"Calling service {self.name} with args {args}; kwargs {kwargs}")
        result = self._service_proxy(*args, **kwargs)
        logger.debug(f"Service {self.name} returned result {result}")
        return result

    async def __call__(self, *args, **kwargs) -> MSG:
        if self._service_proxy is None:
            logger.warning(
                f"Service {self.name} was not connected previously, connecting now. "
                f"Use AsyncService.connect() to connect service proxy preemptively."
            )
            await self.connect()

        async with self._lock:
            func = partial(self._call_nowait, *args, **kwargs)
            result = await anyio.to_thread.run_sync(func)
            return result

    def get_task(self, *args, **kwargs):
        return partial(self, *args, **kwargs)
