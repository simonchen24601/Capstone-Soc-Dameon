# Raspberry Pi 5 (SoC) Dameon Service 

Raspberry Pi 5 (SoC) Dameon Service for Capstone Project of the Embedded System Devlopment 

## Functionalities

- Configuration via `config.ini` and runtime logging (console + file).
- Initialize and manage peripherals: 
    - camera
    - MCU (UART or USB).
- Timed, event-driven main loop for periodic sensor reads and tasks.
- Camera capture, periodic image save, and optional async streaming.
- MCU communication over UART (or USB mode) with robust read/write helpers.
- TODO: MQTT publish/subscribe support for telemetry (NMEA -> JSON publishing).
- TODO: HTTPS REST API communicate with the server backend

## Setup dependance

all compiled standalone libs should be put into the `../ThirdPartyApi` dir

### Setup Boost / OpenSSL / OpenCV / CURL

Using system default, as for debian based system:
```bash
apt install libssl-dev libopencv-dev libcurl4-openssl-dev libboost-all-dev
```

### Setup spdlog

```bash
git clone git@github.com:gabime/spdlog.git
cd spdlog
git checkout v1.16.0
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=../../ThirdPartyApi/spdlog ..
make -j 8 all install
```

### Setup Eclipse Paho MQTT C++

Need to install OpenSSL
`apt install  libssl-dev`
Building the documentation requires doxygen and optionally graphviz to be installed
`sudo apt-get install doxygen graphviz`

```bash
git clone git@github.com:eclipse-paho/paho.mqtt.cpp.git
cd paho.mqtt.cpp
git submodule init
git submodule update
git checkout v1.5.3
mkdir build && cd build

cmake -DPAHO_WITH_MQTT_C=ON -DPAHO_BUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:PATH=../../ThirdPartyApi/mqtt ..
make -j 8 all install
```

## Hardware Debug

### Camera does not working?

1. Check device node, ownership and permissions

```bash
ls -l /dev/video2
# Ensure the user running the daemon is in 'video' group:
groups $USER
# If not:
sudo usermod -aG video $USER
# then log out/in or reboot for group to take effect
```

2. See kernel messages for the camera
```bash
# after plugging device:
dmesg | tail -n 120
```

3. List V4L2 formats/resolutions the device exposes
```
v4l2-ctl --device=/dev/video2 --list-formats-ext
```
