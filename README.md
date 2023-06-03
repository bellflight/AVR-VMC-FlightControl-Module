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

First, install [Poetry](https://python-poetry.org/) and
[VS Code Task Runner](https://pypi.org/project/vscode-task-runner/):

```bash
python -m pip install pipx --upgrade
pipx ensurepath
pipx install poetry
pipx install vscode-task-runner
# (Optionally) Add pre-commit plugin
poetry self add poetry-pre-commit-plugin
```

Now, you can clone the repo and install dependencies:

```bash
git clone https://github.com/bellflight/AVR-VMC-FlightControl-Module
cd AVR-VMC-FlightControl-Module
vtr install
```

Run

```bash
poetry shell
```

to activate the virtual environment.
