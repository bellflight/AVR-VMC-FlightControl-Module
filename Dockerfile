FROM docker.io/library/python:3.11-bullseye

ENV MAVLINK20=1
ENV MAVLINK_DIALECT=bell
# https://github.com/grpc/grpc/issues/21283#issuecomment-624490050
ENV GRPC_PYTHON_BUILD_EXT_COMPILER_JOBS=8

WORKDIR /app

RUN apt-get update -y
RUN apt-get install -y \
    gcc \
    g++ \
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN python -m pip install pip wheel --upgrade && \
    python -m pip install -r requirements.txt

COPY . .

CMD ["python", "fcm.py"]
