import asyncio
import math

import mavsdk
from bell.avr.mqtt.client import MQTTModule
from bell.avr.mqtt.payloads import (
    AvrFcmAttitudeEulerPayload,
    AvrFcmBatteryPayload,
    AvrFcmGpsInfoPayload,
    AvrFcmHilGpsStatsPayload,
    AvrFcmLocationGlobalPayload,
    AvrFcmLocationHomePayload,
    AvrFcmLocationLocalPayload,
    AvrFcmVelocityPayload,
    AvrFusionHilGpsPayload,
)
from bell.avr.utils.decorators import async_try_except, try_except
from bell.avr.utils.timing import rate_limit
from loguru import logger
from pymavlink import mavutil


class FlightControlComputer(MQTTModule):
    def __init__(self) -> None:
        super().__init__()

        # mavlink stuff
        self.drone = mavsdk.System(sysid=141)

        # telemetry persistent variables
        self.heading = 0.0

    async def connect(self) -> None:
        """
        Connect the Drone object.
        """
        logger.debug("Connecting to the FCC")

        # un-comment to show mavsdk server logging
        # import logging
        # logging.basicConfig(level=logging.DEBUG)

        # mavsdk does not support dns
        await self.drone.connect(system_address="udp://0.0.0.0:14541")

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
        return asyncio.gather(
            self.telemetry_tasks(),
        )

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

            update = AvrFcmBatteryPayload(
                voltage=battery.voltage_v,
                soc=battery.remaining_percent * 100.0,
            )

            self.send_message("avr/fcm/battery", update)

    @async_try_except()
    async def in_air_telemetry(self) -> None:
        logger.debug("in_air telemetry loop started")
        async for is_in_air in self.drone.telemetry.in_air():
            self.send_message("avr/fcm/airborne", {"airborne": is_in_air})

    @async_try_except()
    async def is_armed_telemetry(self) -> None:
        logger.debug("armed telemetry loop started")
        async for is_armed in self.drone.telemetry.armed():
            self.send_message("avr/fcm/armed", {"armed": is_armed})

    @async_try_except()
    async def landed_state_telemetry(self) -> None:
        logger.debug("landed_state loop started")
        async for landed_state in self.drone.telemetry.landed_state():
            self.send_message("avr/fcm/landed", {"landed": str(landed_state)})

    @async_try_except()
    async def flight_mode_telemetry(self) -> None:
        logger.debug("flight_mode telemetry loop started")
        async for flight_mode in self.drone.telemetry.flight_mode():
            self.send_message("avr/fcm/flight_mode", {"flight_mode": str(flight_mode)})

    @async_try_except()
    async def position_velocity_ned_telemetry(self) -> None:
        logger.debug("position_velocity_ned telemetry loop started")
        async for position_velocity_ned in self.drone.telemetry.position_velocity_ned():

            n = position_velocity_ned.position.north_m
            e = position_velocity_ned.position.east_m
            d = position_velocity_ned.position.down_m

            update = AvrFcmLocationLocalPayload(dX=n, dY=e, dZ=d)

            self.send_message("avr/fcm/location/local", update)

    @async_try_except()
    async def position_telemetry(self) -> None:
        """
        Runs the position telemetry loop
        """
        logger.debug("position telemetry loop started")
        async for position in self.drone.telemetry.position():
            update = AvrFcmLocationGlobalPayload(
                lat=position.latitude_deg,
                lon=position.longitude_deg,
                alt=position.relative_altitude_m,
                hdg=self.heading,
            )

            self.send_message("avr/fcm/location/global", update)

    @async_try_except()
    async def home_telemetry(self) -> None:
        logger.debug("home telemetry loop started")
        async for home_position in self.drone.telemetry.home():
            update = AvrFcmLocationHomePayload(
                lat=home_position.latitude_deg,
                lon=home_position.longitude_deg,
                alt=home_position.relative_altitude_m,
            )

            self.send_message("avr/fcm/location/home", update)

    @async_try_except()
    async def attitude_euler_telemetry(self) -> None:
        logger.debug("attitude_euler telemetry loop started")
        async for attitude_euler in self.drone.telemetry.attitude_euler():
            psi = attitude_euler.roll_deg
            theta = attitude_euler.pitch_deg
            phi = attitude_euler.yaw_deg

            update = AvrFcmAttitudeEulerPayload(
                roll=psi,
                pitch=theta,
                yaw=phi,
            )

            # do any necessary wrapping here
            heading = (2 * math.pi) + phi if phi < 0 else phi
            self.heading = math.degrees(heading)

            rate_limit(
                lambda: self.send_message("avr/fcm/attitude/euler", update),
                frequency=10,
            )

    @async_try_except()
    async def velocity_ned_telemetry(self) -> None:
        logger.debug("velocity_ned telemetry loop started")
        async for velocity_ned in self.drone.telemetry.velocity_ned():
            update = AvrFcmVelocityPayload(
                vX=velocity_ned.north_m_s,
                vY=velocity_ned.east_m_s,
                vZ=velocity_ned.down_m_s,
            )

            self.send_message("avr/fcm/velocity", update)

    @async_try_except()
    async def gps_info_telemetry(self) -> None:
        logger.debug("gps_info telemetry loop started")
        async for gps_info in self.drone.telemetry.gps_info():
            update = AvrFcmGpsInfoPayload(
                num_satellites=gps_info.num_satellites,
                fix_type=str(gps_info.fix_type),
            )

            self.send_message("avr/fcm/gps_info", update)


class PyMAVLinkAgent(MQTTModule):
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
            "udpin:0.0.0.0:14542", source_system=142, dialect="bell"
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
            ),
            frequency=1,
        )
