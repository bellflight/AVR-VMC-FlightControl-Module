FROM docker.io/library/python:3.11 AS poetry-exporter

WORKDIR /work

RUN python -m pip install poetry poetry-plugin-export

COPY pyproject.toml pyproject.toml
COPY poetry.lock poetry.lock

RUN poetry export -o requirements.txt

# pymavlink does not work with 3.12 as of Nov 2023
FROM docker.io/library/python:3.11-bullseye

ENV MAVLINK20=1
ENV MAVLINK_DIALECT=bell

WORKDIR /app

RUN apt-get update -y
RUN apt-get install -y \
    gcc \
    g++ \
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

COPY --from=poetry-exporter /work/requirements.txt requirements.txt
RUN python -m pip install pip wheel --upgrade \
 && python -m pip install -r requirements.txt

COPY src .

CMD ["python", "fcm.py"]