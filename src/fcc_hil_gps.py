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
    AvrFcmAttitudeEulerPayload,
    AvrFcmBatteryPayload,
    AvrFcmEventsPayload,
    AvrFcmGpsInfoPayload,
    AvrFcmHilGpsStatsPayload,
    AvrFcmLocationGlobalPayload,
    AvrFcmLocationHomePayload,
    AvrFcmLocationLocalPayload,
    AvrFcmStatusPayload,
    AvrFcmVelocityPayload,
    AvrFusionHilGpsPayload,
)
from bell.avr.utils.decorators import async_try_except, try_except
from bell.avr.utils.timing import rate_limit
from loguru import logger
from pymavlink import mavutil
from fcc_mqtt import FCMMQTTModule

class HILGPSManager(FCMMQTTModule):
    def __init__(self) -> None:
        super().__init__()

        self.topic_map = {
            "avr/fusion/hil_gps": self.hilgps_msg_handler,
        }

        self.num_frames = 0

    @try_except()
    def run_non_blocking(self) -> None:
        """
        Set up a mavlink connection and kick off any tasks
        """

        # this NEEDS to be using UDP, TCP proved extremely unreliable
        self.mavcon = mavutil.mavlink_connection(
            "udpout:127.0.0.1:14541", source_system=144, dialect="bell"
        )

        logger.debug("Waiting for Mavlink heartbeat")
        self.mavcon.wait_heartbeat()
        logger.success("Mavlink heartbeat received")

        super().run_non_blocking()

    @try_except(reraise=True)
    def hilgps_msg_handler(self, payload: AvrFusionHilGpsPayload) -> None:
        """
        Handle a HIL_GPS message.
        """
        msg = self.mavcon.mav.hil_gps_heading_encode(  # type: ignore
            payload["time_usec"],
            payload["fix_type"],
            payload["lat"],
            payload["lon"],
            payload["alt"],
            payload["eph"],
            payload["epv"],
            payload["vel"],
            payload["vn"],
            payload["ve"],
            payload["vd"],
            payload["cog"],
            payload["satellites_visible"],
            payload["heading"],
        )
        # logger.debug(msg)
        self.mavcon.mav.send(msg)  # type: ignore
        self.num_frames += 1

        # publish stats every second
        rate_limit(
            lambda: self.send_message( 
                "avr/fcm/hil_gps_stats",
                AvrFcmHilGpsStatsPayload(num_frames=self.num_frames),
            ), #type: ignore
            frequency=1,
        )