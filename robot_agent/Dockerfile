# Robot-side image: the hardware agent and, optionally, the Pi 5 camera stack.
# Built on the robot by setup_robot.sh. No ROS here -- ROS runs on the PC.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Agent dependencies: GPIO, SPI ADCs, I2C IMU, PMIC voltage readout (vcgencmd).
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-pip python3-lgpio python3-spidev python3-smbus \
      i2c-tools gpiod libraspberrypi-bin procps \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --no-cache-dir rplidar-roboticia

# Camera stack build AND runtime dependencies. build_camera_stack.sh compiles
# libcamera/rpicam-apps into /ws/opt/camera (a host bind mount, so it survives
# container recreation) -- but the shared libraries those binaries link against
# must live in the image, or recreating the container breaks rpicam-vid.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake meson ninja-build pkg-config build-essential \
      python3-yaml python3-ply python3-jinja2 \
      libyaml-dev libgnutls28-dev openssl libssl-dev \
      libdrm-dev libjpeg-dev libtiff-dev libpng-dev libexif-dev \
      libevent-dev libboost-program-options-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
CMD ["sleep", "infinity"]
