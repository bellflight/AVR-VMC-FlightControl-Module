import asyncio
import contextlib
import json
import math
import queue
import time
from typing import Any, Callable, List

import mavsdk

from mavsdk.geofence import Point, Polygon
from mavsdk.mission_raw import MissionItem, MissionRawError

from fcc_mqtt import FCMMQTTModule

from bell.avr.utils.decorators import async_try_except, try_except
from pymavlink import mavutil

from loguru import logger

class MissionManager(FCMMQTTModule):
    def __init__(self) -> None:
        super().__init__()

        # mavlink stuff
        self.drone = mavsdk.System(sysid=143)

    async def connect(self) -> None:
        """
        Connect the Drone object.
        """
        logger.debug("Connecting to the FCC")

        # un-comment to show mavsdk server logging
        # import logging
        # logging.basicConfig(level=logging.DEBUG)

        # mavsdk does not support dns
        await self.drone.connect(system_address="udp://127.0.0.1:14543")

        logger.success("Mission: Connected to the FCC")

    @async_try_except(reraise=True)
    async def set_geofence(
        self, min_lat: float, min_lon: float, max_lat: float, max_lon: float
    ) -> None:
        """
        Creates and uploads an inclusive geofence given min/max lat/lon.
        """
        logger.info(
            f"Uploading geofence of ({min_lat}, {min_lon}), ({max_lat}, {max_lon})"
        )

        # need to create a rectangle, PX4 isn't quite smart enough
        # to recognize only two corners
        tl_point = Point(max_lat, min_lon)
        tr_point = Point(max_lat, max_lon)
        bl_point = Point(min_lat, min_lon)
        br_point = Point(min_lat, max_lon)

        fence = [
            Polygon(
                [tl_point, tr_point, bl_point, br_point], Polygon.FenceType.INCLUSION
            )
        ]
        await self.drone.geofence.upload_geofence(fence)

    @async_try_except(reraise=True)
    async def build(self, waypoints: List[dict]) -> List[MissionItem]:
        # sourcery skip: hoist-statement-from-loop, switch, use-assigned-variable
        """
        Convert a list of waypoints (dict) to a list of MissionItems.
        """
        mission_items = []

        # if the first waypoint is not a takeoff waypoint, create one
        if waypoints[0]["type"] != "takeoff":
            # use the altitude of the first waypoint
            waypoints.insert(0, {"type": "takeoff", "alt": waypoints[0]["alt"]})

        # now, check if first waypoint has a lat/lon
        # and if not, add lat lon of current position
        waypoint_0 = waypoints[0]
        if "lat" not in waypoints[0] or "lon" not in waypoints[0]:
            # get the next update from the raw gps and use that
            # .position() only updates on new positions
            position = await self.drone.telemetry.raw_gps().__anext__()
            waypoint_0["lat"] = position.latitude_deg
            waypoint_0["lon"] = position.longitude_deg

        # convert the dicts into mission_raw.MissionItems
        for seq, waypoint in enumerate(waypoints):
            waypoint_type = waypoint["type"]

            # https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
            command = None
            param1 = None
            param2 = None
            param3 = None
            param4 = None

            if waypoint_type == "takeoff":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
                command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                param1 = 0  # pitch
                param2 = float("nan")  # empty
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint_type == "goto":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                param1 = 0  # hold time
                param2 = 0  # accepteance radius
                param3 = 0  # pass radius, 0 goes straight through
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint_type == "land":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
                command = mavutil.mavlink.MAV_CMD_NAV_LAND
                param1 = 0  # abort altitude, 0 uses system default
                # https://mavlink.io/en/messages/common.html#PRECISION_LAND_MODE
                # precision landing mode
                param2 = mavutil.mavlink.PRECISION_LAND_MODE_DISABLED
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            # https://mavlink.io/en/messages/common.html#MAV_FRAME
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            current = int(seq == 0)  # boolean
            autocontinue = int(True)
            x = int(float(waypoint["lat"]) * 10000000)
            y = int(float(waypoint["lon"]) * 10000000)
            z = float(waypoint["alt"])
            # https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE
            mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

            mission_items.append(
                MissionItem(
                    seq=seq,
                    frame=frame,
                    command=command,
                    current=current,
                    autocontinue=autocontinue,
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    param4=param4,
                    x=x,
                    y=y,
                    z=z,
                    mission_type=mission_type,
                )
            )

        return mission_items

    @async_try_except(reraise=True)
    async def upload(self, mission_items: List[MissionItem]) -> None:
        """
        Upload a given list of MissionItems to the drone.
        """
        try:
            logger.info("Clearing existing mission on the drone")
            await self.drone.mission_raw.clear_mission()
            logger.info("Uploading mission items to drone")
            await self.drone.mission_raw.upload_mission(mission_items)
            self._publish_event("mission_upload_success_event")
        except MissionRawError as e:
            logger.warning(f"Mission upload failed because: {e._result.result_str}")
            self._publish_event(
                "mission_upload_failed_event", str(e._result.result_str)
            )

    @async_try_except(reraise=True)
    async def build_and_upload(self, waypoints: List[dict]) -> None:
        """
        Upload a list of waypoints (dict) to the done.
        """
        mission_plan = await self.build(waypoints)
        await self.upload(mission_plan)

    @async_try_except(reraise=True)
    async def download(self) -> List[MissionItem]:
        """
        Download the current mission from the drone as a list of MissionItems.
        """
        logger.info("Downloading mission plan from drone")
        return await self.drone.mission_raw.download_mission()

    @async_try_except(reraise=True)
    async def wait_for_finish(self) -> None:
        """
        Async blocking function that waits for the current mission to be finished.
        """
        # self.drone.mission.is_missiion_finished unfortunately does not work,
        # with the mission_raw.MissionItems we've uploaded

        # if called immediately after a mission has been started, will immediately
        # exit as the drone hasn't even started moving to the first waypoint yet
        # give it 5 seconds to get moving first.
        mission_progress = await self.drone.mission_raw.mission_progress().__anext__()
        if mission_progress.current == 0:
            await asyncio.sleep(5)

        async for mission_progress in self.drone.mission_raw.mission_progress():
            if mission_progress.current == 0:
                return

    @async_try_except(reraise=True)
    async def start(self) -> None:
        """
        Commands the drone to start the current mission.
        Drone must already be armed.
        Will raise an exception if the active mission violates a geofence.
        """
        logger.info("Sending start mission command")
        await self.drone.mission_raw.start_mission()

    @async_try_except(reraise=True)
    async def hold(self) -> None:
        """
        Commands the drone to hold the current mission.
        """
        logger.info("Sending pause mission command")
        await self.drone.mission_raw.pause_mission()

    @async_try_except(reraise=True)
    async def pause(self) -> None:
        """
        Commands the drone to pause the current mission.
        """
        logger.info("Sending pause mission command")
        await self.hold()

    @async_try_except(reraise=True)
    async def resume(self) -> None:
        """
        Commands the drone to resume the paused mission.
        """
        logger.info("Sending resume mission command")
        await self.start()
