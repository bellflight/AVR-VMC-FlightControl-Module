import asyncio
import math
import queue
from typing import Coroutine, List, Literal

import mavsdk
import pymap3d
from bell.avr.mqtt.module import MQTTModule
from bell.avr.mqtt.payloads import (
    AVRFCMActionSleep,
    AVRFCMActionTakeoff,
    AVRFCMAirborne,
    AVRFCMArmed,
    AVRFCMAttitudeEulerDegrees,
    AVRFCMBattery,
    AVRFCMFlightMode,
    AVRFCMGoToGlobal,
    AVRFCMGoToLocal,
    AVRFCMGPSInfo,
    AVRFCMHILGPSStats,
    AVRFCMLanded,
    AVRFCMMissionUpload,
    AVRFCMPositionGlobal,
    AVRFCMPositionHome,
    AVRFCMPositionLocal,
    AVRFCMVelocity,
    AVRFusionHILGPSMessage,
)
from bell.avr.utils.decorators import async_try_except, try_except
from bell.avr.utils.env import get_env_int
from bell.avr.utils.timing import rate_limit
from loguru import logger
from mavsdk.mission_raw import MissionItem, MissionRawError
from pymavlink import mavutil

MAVLINK_UDP_1 = get_env_int("MAVLINK_UDP_1", 14541)
MAVLINK_UDP_2 = get_env_int("MAVLINK_UDP_2", 14542)


class FlightControlComputer(MQTTModule):
    def __init__(self) -> None:
        super().__init__()

        self.topic_callbacks = {
            "avr/fcm/action/sleep": self.sleep_action_handler,
            "avr/fcm/action/arm": self.arm_action_handler,
            "avr/fcm/action/disarm": self.disarm_action_handler,
            "avr/fcm/action/kill": self.kill_action_handler,
            "avr/fcm/action/land": self.land_action_handler,
            "avr/fcm/action/reboot": self.reboot_action_handler,
            "avr/fcm/action/takeoff": self.takeoff_action_handler,
            "avr/fcm/action/goto/global": self.goto_position_global_action_handler,
            "avr/fcm/action/goto/local": self.goto_position_local_action_handler,
            "avr/fcm/action/mission/upload": self.upload_mission_action_handler,
            "avr/fcm/action/mission/start": self.start_mission_action_handler,
        }

        # mavlink stuff
        self.drone = mavsdk.System(sysid=141)

        # telemetry persistent variables
        self.heading = 0.0

        # queue to hold action information
        self.action_queue: queue.Queue[Coroutine] = queue.Queue()

    async def connect(self) -> None:
        """
        Connect the Drone object.
        """
        logger.debug("Connecting to the FCC")

        # un-comment to show mavsdk server logging
        # import logging
        # logging.basicConfig(level=logging.DEBUG)

        # mavsdk does not support dns
        await self.drone.connect(system_address=f"udp://0.0.0.0:{MAVLINK_UDP_1}")

        logger.success("Connected to the FCC")

    async def run_non_blocking(self) -> asyncio.Future:
        """
        Run the Flight Control Computer module
        """
        # start our MQTT client
        super().run_non_blocking()

        # connect to the fcc
        await self.connect()

        # start tasks
        return asyncio.gather(self.telemetry_tasks(), self.action_executor())

    async def telemetry_tasks(self) -> asyncio.Future:
        """
        Gathers the telemetry tasks
        """
        return asyncio.gather(
            self.battery_telemetry(),
            self.in_air_telemetry(),
            self.is_armed_telemetry(),
            self.flight_mode_telemetry(),
            self.landed_state_telemetry(),
            self.position_velocity_ned_telemetry(),
            self.position_telemetry(),
            self.home_telemetry(),
            self.attitude_euler_telemetry(),
            self.velocity_ned_telemetry(),
            self.gps_info_telemetry(),
        )

    @async_try_except()
    async def battery_telemetry(self) -> None:
        logger.debug("battery telemetry loop started")

        async for battery in self.drone.telemetry.battery():
            self.send_message(
                "avr/fcm/battery",
                AVRFCMBattery(
                    voltage=battery.voltage_v,
                    state_of_charge=battery.remaining_percent * 100.0,
                ),
            )

    @async_try_except()
    async def in_air_telemetry(self) -> None:
        logger.debug("in_air telemetry loop started")

        async for is_in_air in self.drone.telemetry.in_air():
            self.send_message("avr/fcm/airborne", AVRFCMAirborne(airborne=is_in_air))

    @async_try_except()
    async def is_armed_telemetry(self) -> None:
        logger.debug("armed telemetry loop started")

        async for is_armed in self.drone.telemetry.armed():
            self.send_message("avr/fcm/armed", AVRFCMArmed(armed=is_armed))

    @async_try_except()
    async def landed_state_telemetry(self) -> None:
        logger.debug("landed_state loop started")

        async for landed_state in self.drone.telemetry.landed_state():
            landed_state_str = "UNKNOWN"
            if landed_state is not None:
                landed_state_str = landed_state.name

            self.send_message("avr/fcm/landed", AVRFCMLanded(landed=landed_state_str))

    @async_try_except()
    async def flight_mode_telemetry(self) -> None:
        logger.debug("flight_mode telemetry loop started")

        async for flight_mode in self.drone.telemetry.flight_mode():
            flight_mode_str = "UNKNOWN"
            if flight_mode is not None:
                flight_mode_str = flight_mode.name

            self.send_message(
                "avr/fcm/flight_mode", AVRFCMFlightMode(flight_mode=flight_mode_str)
            )

    @async_try_except()
    async def position_velocity_ned_telemetry(self) -> None:
        logger.debug("position_velocity_ned telemetry loop started")

        async for position_velocity_ned in self.drone.telemetry.position_velocity_ned():
            self.send_message(
                "avr/fcm/position/local",
                AVRFCMPositionLocal(
                    n=position_velocity_ned.position.north_m,
                    e=position_velocity_ned.position.east_m,
                    d=position_velocity_ned.position.down_m,
                ),
            )

    @async_try_except()
    async def position_telemetry(self) -> None:
        logger.debug("position telemetry loop started")

        async for position in self.drone.telemetry.position():
            self.send_message(
                "avr/fcm/position/global",
                AVRFCMPositionGlobal(
                    lat=position.latitude_deg,
                    lon=position.longitude_deg,
                    rel_alt=position.relative_altitude_m,
                    abs_alt=position.absolute_altitude_m,
                    hdg=self.heading,
                ),
            )

    @async_try_except()
    async def home_telemetry(self) -> None:
        logger.debug("home telemetry loop started")

        async for home_position in self.drone.telemetry.home():
            self.send_message(
                "avr/fcm/position/home",
                AVRFCMPositionHome(
                    lat=home_position.latitude_deg,
                    lon=home_position.longitude_deg,
                    rel_alt=home_position.relative_altitude_m,
                    abs_alt=home_position.absolute_altitude_m,
                ),
            )

    @async_try_except()
    async def attitude_euler_telemetry(self) -> None:
        logger.debug("attitude_euler telemetry loop started")

        async for attitude_euler in self.drone.telemetry.attitude_euler():
            phi = attitude_euler.yaw_deg

            # do any necessary wrapping here
            heading = (2 * math.pi) + phi if phi < 0 else phi
            self.heading = math.degrees(heading)

            rate_limit(
                lambda: self.send_message(
                    "avr/fcm/attitude/euler/degrees",
                    AVRFCMAttitudeEulerDegrees(
                        roll=attitude_euler.roll_deg,
                        pitch=attitude_euler.pitch_deg,
                        yaw=attitude_euler.yaw_deg,
                    ),
                ),
                frequency=10,
            )

    @async_try_except()
    async def velocity_ned_telemetry(self) -> None:
        logger.debug("velocity_ned telemetry loop started")

        async for velocity_ned in self.drone.telemetry.velocity_ned():
            self.send_message(
                "avr/fcm/velocity",
                AVRFCMVelocity(
                    Vn=velocity_ned.north_m_s,
                    Ve=velocity_ned.east_m_s,
                    Vd=velocity_ned.down_m_s,
                ),
            )

    @async_try_except()
    async def gps_info_telemetry(self) -> None:
        logger.debug("gps_info telemetry loop started")

        async for gps_info in self.drone.telemetry.gps_info():
            gps_fix_str = "NO_GPS"
            if gps_info.fix_type is not None:
                gps_fix_str: Literal[
                    "NO_GPS",
                    "NO_FIX",
                    "FIX_2D",
                    "FIX_3D",
                    "FIX_DGPS",
                    "RTK_FLOAT",
                    "RTK_FIXED",
                ] = gps_info.fix_type.name  # type: ignore

            self.send_message(
                "avr/fcm/gps/info",
                AVRFCMGPSInfo(
                    visible_satellites=gps_info.num_satellites,
                    fix_type=gps_fix_str,
                ),
            )

    @async_try_except()
    async def action_executor(self) -> None:
        logger.debug("action executor loop started")

        while True:
            try:
                action: Coroutine = self.action_queue.get_nowait()
                await action
            except queue.Empty:
                await asyncio.sleep(0.1)
            except Exception:
                logger.exception("Unexpected error while executing action")

    async def sleep_action(self, seconds: int) -> None:
        logger.info(f"Sleeping for {seconds} seconds")
        await asyncio.sleep(seconds)

    def sleep_action_handler(self, payload: AVRFCMActionSleep) -> None:
        self.action_queue.put(self.sleep_action(payload.seconds))

    async def arm_action(self) -> None:
        logger.warning("Arming drone")
        await self.drone.action.arm()

    def arm_action_handler(self) -> None:
        self.action_queue.put(self.arm_action())

    async def disarm_action(self) -> None:
        logger.info("Disarming drone")
        await self.drone.action.disarm()

    def disarm_action_handler(self) -> None:
        self.action_queue.put(self.disarm_action())

    async def kill_action(self) -> None:
        logger.warning("Killing flight controller")
        await self.drone.action.kill()

    def kill_action_handler(self) -> None:
        self.action_queue.put(self.kill_action())

    async def land_action(self) -> None:
        logger.info("Landing drone")
        await self.drone.action.land()

    def land_action_handler(self) -> None:
        self.action_queue.put(self.land_action())

    async def reboot_action(self) -> None:
        logger.info("Rebooting flight controller")
        await self.drone.action.reboot()

    def reboot_action_handler(self) -> None:
        self.action_queue.put(self.reboot_action())

    async def takeoff_action(self, rel_alt: float) -> None:
        logger.info(f"Taking off drone to {rel_alt} meters")
        await self.drone.action.set_takeoff_altitude(rel_alt)
        await self.arm_action()
        await self.drone.action.takeoff()

    def takeoff_action_handler(self, payload: AVRFCMActionTakeoff) -> None:
        self.action_queue.put(self.takeoff_action(payload.rel_alt))

    async def goto_position_global_action(self, payload: AVRFCMGoToGlobal) -> None:
        logger.info(
            f"Going to {payload.lat}, {payload.lon}, {payload.abs_alt} with heading {payload.hdg}"
        )
        await self.drone.action.goto_location(
            payload.lat, payload.lon, payload.abs_alt, payload.hdg
        )

    def goto_position_global_action_handler(self, payload: AVRFCMGoToGlobal) -> None:
        self.action_queue.put(self.goto_position_global_action(payload))

    async def goto_position_local_action(self, payload: AVRFCMGoToLocal) -> None:
        # start with the home position as the point of reference
        home_position = self.message_cache.get("avr/fcm/position/home")
        if not home_position:
            logger.error(
                "Unable to complete goto request, home position data not initialized"
            )
            return

        # if it's relative, use our current position
        if payload.relative:
            source_position = self.message_cache.get("avr/fcm/position/global")

            if not source_position:
                logger.error(
                    "Unable to complete goto request, drone position data not initialized"
                )
                return

            # add in the absolute alt from home since alt is shown as relative for
            # current position and go to needs absolute
            source_position.abs_alt += home_position.abs_alt
        else:
            source_position = home_position

        logger.info(
            f"Computing absolute position for {source_position.lat}, {source_position.lon}, {source_position.abs_alt}"
        )

        # local to global coordiante conversion
        lat, lon, alt = pymap3d.ned.ned2geodetic(
            payload.n,
            payload.e,
            payload.d,
            source_position.lat,
            source_position.lon,
            source_position.abs_alt,
        )

        # re-use global goto
        await self.goto_position_global_action(
            AVRFCMGoToGlobal(lat=lat, lon=lon, abs_alt=alt, hdg=payload.hdg)
        )

    def goto_position_local_action_handler(self, payload: AVRFCMGoToLocal) -> None:
        self.action_queue.put(self.goto_position_local_action(payload))

    async def uploading_mission_action(self, mission: AVRFCMMissionUpload) -> None:
        logger.info("Uploading mission")

        home_position = self.message_cache.get("avr/fcm/position/home")
        if not home_position:
            logger.error(
                "Unable to complete goto request, home position data not initialized"
            )
            return

        source_position = self.message_cache.get("avr/fcm/position/global")
        if not source_position:
            logger.error(
                "Unable to complete goto request, position data not initialized"
            )
            return

        mission_items: List[MissionItem] = []

        # convert the dicts into mission_raw.MissionItems
        for seq, waypoint in enumerate(mission.waypoints):
            # https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
            command = None
            param1 = None
            param2 = None
            param3 = None
            param4 = None

            if waypoint.waypoint_type == "TAKEOFF":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
                command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                param1 = 0  # pitch
                param2 = float("nan")  # empty
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint.waypoint_type == "GOTO":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                param1 = 0  # hold time
                param2 = 0  # accepteance radius
                param3 = 0  # pass radius, 0 goes straight through / is ignored if hold time > 0
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            elif waypoint.waypoint_type == "LAND":
                # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
                command = mavutil.mavlink.MAV_CMD_NAV_LAND
                param1 = 0  # abort altitude, 0 uses system default
                # https://mavlink.io/en/messages/common.html#PRECISION_LAND_MODE
                # precision landing mode
                param2 = mavutil.mavlink.PRECISION_LAND_MODE_DISABLED
                param3 = float("nan")  # empty
                param4 = float("nan")  # yaw angle. NaN uses current yaw heading mode

            # validation
            if seq != 0 and waypoint.waypoint_type == "TAKEOFF":
                logger.error(
                    "Only the first waypoint in a mission can be a takeoff waypoint"
                )
                return

            if waypoint.waypoint_type == "LAND" and seq != len(mission.waypoints) - 1:
                logger.error(
                    "Only the last waypoint in a mission can be a land waypoint"
                )
                return

            # fill in takeoff coordinates
            if seq == 0 and waypoint.waypoint_type == "TAKEOFF":
                waypoint.lat = source_position.lat
                waypoint.lon = source_position.lon

            # if all 3 absolute coordinates are provided
            if all(
                j is not None for j in [waypoint.lat, waypoint.lon, waypoint.abs_alt]
            ):
                assert waypoint.lat is not None
                assert waypoint.lon is not None
                assert waypoint.abs_alt is not None
                lat, lon, alt = waypoint.lat, waypoint.lon, waypoint.abs_alt

            # if all 3 local coordinates are provided
            elif all(j is not None for j in [waypoint.n, waypoint.e, waypoint.d]):
                assert waypoint.n is not None
                assert waypoint.e is not None
                assert waypoint.d is not None

                lat, lon, alt = pymap3d.ned.ned2geodetic(
                    waypoint.n,
                    waypoint.e,
                    waypoint.d,
                    home_position.lat,
                    home_position.lon,
                    home_position.abs_alt,
                )
                # this is.. weird but sets up the next section to be able to reuse code
                alt = alt - home_position.abs_alt

            # error
            else:
                logger.error(
                    f"X, Y, Z coordinates were not provided in item {seq} of the mission"
                )
                return

            x = int(lat * 10000000)
            y = int(lon * 10000000)
            z = alt + home_position.abs_alt

            mission_items.append(
                MissionItem(
                    seq=seq,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # https://mavlink.io/en/messages/common.html#MAV_FRAME
                    command=command,
                    current=int(seq == 0),  # boolean
                    autocontinue=int(True),
                    param1=param1,
                    param2=param2,
                    param3=param3,
                    param4=param4,
                    x=x,
                    y=y,
                    z=z,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,  # https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE
                )
            )

        try:
            logger.info("Clearing existing mission")
            await self.drone.mission_raw.clear_mission()
            logger.info("Uploading mission items")
            await self.drone.mission_raw.upload_mission(mission_items)
            logger.success("Mission upload sucess")

        except MissionRawError as e:
            logger.exception(f"Mission upload failed because: {e._result.result_str}")

    def upload_mission_action_handler(self, payload: AVRFCMMissionUpload) -> None:
        self.action_queue.put(self.uploading_mission_action(payload))

    async def start_mission_action(self) -> None:
        logger.info("Starting mission")
        await self.drone.mission_raw.start_mission()

    def start_mission_action_handler(self) -> None:
        self.action_queue.put(self.start_mission_action())


class PyMAVLinkAgent(MQTTModule):
    def __init__(self) -> None:
        super().__init__()

        self.topic_callbacks = {
            "avr/fusion/hil_gps/message": self.hilgps_msg_handler,
        }

        self.frames = 0

    @try_except()
    def run_non_blocking(self) -> None:
        """
        Set up a mavlink connection and kick off any tasks
        """

        # this NEEDS to be using UDP, TCP proved extremely unreliable
        self.mavcon = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{MAVLINK_UDP_2}", source_system=142, dialect="bell"
        )

        logger.debug("Waiting for Mavlink heartbeat")
        self.mavcon.wait_heartbeat()
        logger.success("Mavlink heartbeat received")

        super().run_non_blocking()

    @try_except(reraise=True)
    def hilgps_msg_handler(self, payload: AVRFusionHILGPSMessage) -> None:
        """
        Handle a HIL_GPS message.
        """
        msg = self.mavcon.mav.hil_gps_heading_encode(  # type: ignore
            payload.time_usec,
            payload.fix_type,
            payload.lat,
            payload.lon,
            payload.alt,
            payload.eph,
            payload.epv,
            payload.vel,
            payload.vn,
            payload.ve,
            payload.vd,
            payload.cog,
            payload.satellites_visible,
            payload.heading,
        )
        # logger.debug(msg)
        self.mavcon.mav.send(msg)  # type: ignore
        self.frames += 1

        # publish stats every second
        rate_limit(
            lambda: self.send_message(
                "avr/fcm/hil_gps/stats",
                AVRFCMHILGPSStats(frames=self.frames),
            ),
            frequency=1,
        )
