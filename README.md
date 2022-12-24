# AVR-VMC-FlightControl-Module

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Build FlightControl Module](https://github.com/bellflight/AVR-VMC-FlightControl-Module/actions/workflows/build.yml/badge.svg)](https://github.com/bellflight/AVR-VMC-FlightControl-Module/actions/workflows/build.yml)

The Flight Control module (FCM) is responsible for communicating with the
FCC over MAVLink. This module takes telemetry data from the FCC and publishes
it over MQTT. Additionally, it takes fake GPS data from the Fusion module
and feeds it to the FCC.

There also exists functionality to send commands to the FCC, such as arming the
drone, or sending it missions, but this is disabled due to the drone’s current
lack of knowledge of where it is in global coordinates.
