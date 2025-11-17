# Raspberry Pi 5 (SoC) Dameon Service 

Raspberry Pi 5 (SoC) Dameon Service for Capstone Project of the Embedded System Devlopment 

## setup dependance

required third party libraries:
- spdlog
- Eclipse Paho MQTT C++
- OpenCV
- gRPC C++

all compiled standalone libs should be put into the `../ThirdPartyApi` dir

### setup spdlog

```bash
git clone git@github.com:gabime/spdlog.git
cd spdlog
git checkout v1.16.0
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=../ThirdPartyApi/spdlog ..
make -j 8 all install
```

### setup Eclipse Paho MQTT C++

need install OpenSSL
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

cmake -DPAHO_WITH_MQTT_C=ON -DPAHO_BUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:PATH=../ThirdPartyApi/mqtt ..
make -j 8 all install
```
