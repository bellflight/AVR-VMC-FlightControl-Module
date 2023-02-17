import asyncio

from fcc_control import ControlManager
from fcc_mission import MissionManager
from fcc_telemetry import TelemetryManager
from fcc_hil_gps import HILGPSManager

class FlightControlModule:
    def __init__(self) -> None:
        super().__init__()

        # create the FCC objects
        self.control_manager = ControlManager()
        self.mission_manager = MissionManager()
        self.hil_gps_manager = HILGPSManager()
        self.telemetry_manager = TelemetryManager()


    async def run(self) -> None:
        
        #self.mission_manager.run_non_blocking()
        #self.hil_gps_manager.run_non_blocking()
        await self.telemetry_manager.run_non_blocking()

        asyncio.gather(
            self.telemetry_manager.run_non_blocking(),
            #self.control_manager.run_non_blocking()
        )

        while True:
            await asyncio.sleep(1)


if __name__ == "__main__":
    fcm = FlightControlModule()
    asyncio.run(fcm.run())
