import asyncio
import contextlib
import json
import math
import queue
import time
from typing import Any, Callable, List

import mavsdk
from bell.avr.mqtt.client import MQTTModule
from bell.avr.mqtt.payloads import (
    AvrFcmEventsPayload
)
from bell.avr.utils.decorators import async_try_except, try_except
from bell.avr.utils.timing import rate_limit
from loguru import logger
from mavsdk.action import ActionError
from fcc_mqtt import FCMMQTTModule


class DispatcherBusy(Exception):
    """
    Exception for when the action dispatcher is currently busy
    executing another action
    """


class DispatcherManager(FCMMQTTModule):
    def __init__(self) -> None:
        super().__init__()
        self.currently_running_task = None
        self.timeout = 10

    async def schedule_task(self, task: Callable, payload: Any, name: str) -> None:
        """
        Schedule a task (async func) to be run by the dispatcher with the
        given payload. Task name is also required for printing.
        """
        logger.debug(f"Scheduling a task for '{name}'")
        # if the dispatcher is ok to take on a new task
        if (
            self.currently_running_task is not None
            and self.currently_running_task.done()
        ) or self.currently_running_task is None:
            await self.create_task(task, payload, name)
        else:
            raise DispatcherBusy

    async def create_task(self, task: Callable, payload: dict, name: str) -> None:
        """
        Create a task to be run.
        """
        self.currently_running_task = asyncio.create_task(
            self.task_waiter(task, payload, name)
        )

    async def task_waiter(self, task: Callable, payload: dict, name: str) -> None:
        """
        Execute a task with a timeout.
        """
        try:
            await asyncio.wait_for(task(**payload), timeout=self.timeout)
            self._publish_event(f"request_{name}_completed_event")
            self.currently_running_task = None

        except asyncio.TimeoutError:
            try:
                logger.warning(f"Task '{name}' timed out!")
                self._publish_event("action_timeout_event", name)
                self.currently_running_task = None

            except Exception:
                logger.exception("ERROR IN TIMEOUT HANDLER")

        except Exception:
            logger.exception("ERROR IN TASK WAITER")

class ControlManager(FCMMQTTModule):
    def __init__(self) -> None:
        super().__init__()

        # mavlink stuff
        self.drone = mavsdk.System(sysid=142)

        # queues
        self.action_queue = queue.Queue()

    async def connect(self) -> None:
        """
        Connect the Drone object.
        """
        logger.debug("Control: Connecting to the FCC")

        # un-comment to show mavsdk server logging
        # import logging
        # logging.basicConfig(level=logging.DEBUG)

        # mavsdk does not support dns
        await self.drone.connect(system_address="udp://127.0.0.1:14541")

        logger.success("Connected to the FCC")

    async def async_queue_action(
        self, queue_: queue.Queue, action: Callable, frequency: int = 10
    ) -> None:
        """
        Creates a while loop that continously tries to pull a dict from a queue
        and do something with it at a set frequency.

        The given function needs to accept a single argument of the protobuf object
        and be async.

        Setting the frequency to 0 will always run the action.
        """
        last_time = time.time()

        # this particular design will constantly get messages from the queue,
        # even if they are not used, just to try and process them as fast
        # as possible to prevent the queue from filling

        while True:
            try:
                # get the next item from the queue
                data = queue_.get_nowait()
                # if the frequency is 0, or the time since our last run is greater
                # than the frequency, run
                if frequency == 0 or time.time() - last_time > (1 / frequency):
                    # call function
                    await action(data)
                    # reset timer
                    last_time = time.time()

            except queue.Empty:
                # if the queue was empty, just wait
                await asyncio.sleep(0.01)

            except Exception:
                logger.exception("Unexpected error in async_queue_action")

    async def run_non_blocking(self) -> asyncio.Future:
        """
        Run the Flight Control Computer module
        """
        # start our MQTT client
        super().run_non_blocking()

        # connect to the fcc
        await self.connect()

        # start the mission api MQTT client
        # self.mission_api.run_non_blocking()

        # start tasks
        return asyncio.gather(
            # uncomment the following lines to enable outside control
            self.action_dispatcher(),
        )

    # region ################## D I S P A T C H E R  ##########################

    @async_try_except()
    async def action_dispatcher(self) -> None:
        logger.debug("action_dispatcher started")

        action_map = {
            "break": self.set_intentional_timeout,
            "connect": self.connect,
            "arm": self.set_arm,
            "disarm": self.set_disarm,
            "kill": self.set_kill,
            "land": self.set_land,
            "reboot": self.set_reboot,
            "takeoff": self.set_takeoff
        }

        dispatcher = DispatcherManager()
        dispatcher.run_non_blocking()

        while True:
            action = {}
            try:
                action = self.action_queue.get_nowait()

                if action["payload"] == "":
                    action["payload"] = "{}"

                if action["name"] in action_map:
                    payload = json.loads(action["payload"])
                    await dispatcher.schedule_task(
                        action_map[action["name"]], payload, action["name"]
                    )
                else:
                    logger.warning(f"Unknown action: {action['name']}")

            except DispatcherBusy:
                logger.info("I'm busy running another task, try again later")
                self._publish_event("fcc_busy_event", payload=action["name"])

            except queue.Empty:
                await asyncio.sleep(0.1)

            except Exception:
                logger.exception("ERROR IN MAIN LOOP")

    async def simple_action_executor(
        self,
        action_fn: Callable,
        action_text: str,
    ) -> None:
        """
        Executes a given async action function, and publishes a success or failed
        state machine event given whether or not an `ActionError` was raised.
        """
        try:
            await action_fn()
            full_success_str = f"{action_text}_success_event"
            logger.info(f"Sending {full_success_str}")
            self._publish_event(full_success_str)

        except ActionError as e:
            full_fail_str = f"{action_text}_failed_event"
            logger.info(f"Sending {full_fail_str}")
            self._publish_event(full_fail_str)

            if e._result.result_str == "CONNECTION_ERROR":
                asyncio.create_task(self.connect())

            raise e from e

    # endregion ###############################################################

    # region #####################  A C T I O N S #############################

    @async_try_except()
    async def set_intentional_timeout(self, **kwargs) -> None:
        """
        Sets a 20 second timeout.
        """
        with contextlib.suppress(asyncio.CancelledError):
            await asyncio.sleep(20)

    @async_try_except(reraise=True)
    async def set_arm(self, **kwargs) -> None:
        """
        Sets the drone to an armed state.
        """
        logger.info("Sending arm command")
        await self.simple_action_executor(self.drone.action.arm, "arm")

    @async_try_except(reraise=True)
    async def set_disarm(self, **kwargs) -> None:
        """
        Sets the drone to a disarmed state.
        """
        logger.info("Sending disarm command")
        await self.simple_action_executor(self.drone.action.disarm, "disarm")

    @async_try_except(reraise=True)
    async def set_kill(self, **kwargs) -> None:
        """
        Sets the drone to a kill state. This will forcefully shut off the drone
        regardless of being in the air or not.
        """
        logger.warning("Sending kill command")
        await self.simple_action_executor(self.drone.action.kill, "kill")

    @async_try_except(reraise=True)
    async def set_land(self, **kwargs) -> None:
        """
        Commands the drone to land at the current position.
        """
        logger.info("Sending land command")
        await self.simple_action_executor(self.drone.action.land, "land_cmd")

    @async_try_except(reraise=True)
    async def set_reboot(self, **kwargs) -> None:
        """
        Commands the drone computer to reboot.
        """
        logger.warning("Sending reboot command")
        await self.simple_action_executor(self.drone.action.reboot, "reboot")

    @async_try_except(reraise=True)
    async def set_takeoff(self, takeoff_alt: float, **kwargs) -> None:
        """
        Commands the drone to takeoff to the given altitude.
        Will arm the drone if it is not already.
        """
        logger.info(f"Setting takeoff altitude to {takeoff_alt}")
        await self.drone.action.set_takeoff_altitude(takeoff_alt)
        await self.set_arm()
        logger.info("Sending takeoff command")
        await self.simple_action_executor(self.drone.action.takeoff, "takeoff")

    # endregion ###############################################################