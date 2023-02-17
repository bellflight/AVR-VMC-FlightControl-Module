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

## Development

It's assumed you have a version of Python installed from
[python.org](https://python.org) that is the same or newer as
defined in the [`Dockerfile`](Dockerfile).

First, install [Poetry](https://python-poetry.org/):

```bash
python -m pip install pipx --upgrade
pipx ensurepath
pipx install poetry
# (Optionally) Add pre-commit plugin
poetry self add poetry-pre-commit-plugin
```

Now, you can clone the repo and install dependencies:

```bash
git clone https://github.com/bellflight/AVR-VMC-FlightControl-Module
cd AVR-VMC-FlightControl-Module
poetry install --sync
poetry run pre-commit install --install-hooks
```

Run

```bash
poetry shell
```

to activate the virtual environment.


# MQTT Endpoints
Topic: `/avr/fcm/actions/`

### Arm

Schema: 
```json
{
    "action": "arm",
    "payload": {}
}
```

### Disarm

Schema: 
```json
{
    "action": "disarm",
    "payload": {}
}
```

### Kill 
Schema: 
```json
{
    "action": "kill",
    "payload": {}
}
```
### Land
Schema: 
```json
{
    "action": "land",
    "payload": {}
}
```
### Reboot
Schema: 
```json
{
    "action": "reboot",
    "payload": {}
}
```

### Go To Location
Schema: 
```json
{
    "action": "goto_location",
    "payload": {
        "lat": <decimal_degrees_lat>,
        "lon": <decimal_degrees_lon>,
        "alt": <float_meters_MSL>,
        "heading": <decimal degrees heading>
    }
}
```

### Upload Mission
Schema: 
```json
{
    "action": "upload_mission",
    "payload": {
        "waypoints": [
    {
        "type": "goto",
        "lat": <decimal_degrees_lat>,
        "lon": <decimal_degrees_lon>,
        "alt": <float_meters_relative_takeoff>,
    },
    {
        "type": "goto",
        "lat": <decimal_degrees_lat>,
        "lon": <decimal_degrees_lon>,
        "alt": <float_meters_relative_takeoff>,
    }
]
    }
}
```

### Start Mission
Schema: 
```json
{
    "action": "start_mission",
    "payload": {}
}
```