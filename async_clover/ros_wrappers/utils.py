import attr

import logging
from functools import partial
from typing import Callable, Any, Coroutine, Optional, Type, TypeVar

import anyio
import rospy

logger = logging.getLogger(__name__)

MSG = TypeVar('MSG')


async def wait_for_service(name: str, timeout: Optional[float] = 10):
    logger.info(f"Waiting until service '{name}' is available.")
    service_waiter = partial(rospy.wait_for_service, name)
    try:
        with anyio.fail_after(timeout):
            await anyio.to_thread.run_sync(service_waiter, cancellable=True)
    except TimeoutError:
        logger.error(f"Timeout ({timeout}s)! Service '{name}' is not available!")
        raise


async def get_message(topic_name: str, message_class: Type[MSG], timeout: Optional[float] = 10) -> MSG:
    message_waiter = partial(rospy.wait_for_message, topic_name, message_class, timeout=timeout)
    try:
        with anyio.fail_after(timeout):
            message = await anyio.to_thread.run_sync(message_waiter, cancellable=True)
        return message
    except (TimeoutError, rospy.exceptions.ROSException):
        logger.error(f"Timeout ({timeout}s)! Topic '{topic_name}' is not available! Have you initialized a node?")
        raise TimeoutError(f"Timeout ({timeout}s)! Topic '{topic_name}' is not available! Have you initialized a node?") from None


async def start_node(name: str, anonymous: bool = False, timeout: Optional[float] = 10):
    logger.info(f"Waiting until node '{name}' start.")
    node_waiter = partial(rospy.init_node, name, anonymous=anonymous)
    try:
        with anyio.fail_after(timeout):
            await anyio.to_thread.run_sync(node_waiter, cancellable=True)
    except TimeoutError:
        logger.error(f"Timeout ({timeout}s)! Node '{name}' had not started!")
        raise


@attr.define()
class Callback:
    callback: Optional[Callable[..., Coroutine[Any, Any, Any]]] = attr.field(default=None)

    def set(self, callback: Callable[[Any, Any], Coroutine[Any, Any, Any]]):
        self.callback = callback

    async def emit(self, *args, **kwargs):
        if self.callback is not None:
            await self.callback(*args, **kwargs)
