#include "interface.h"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
//#include <opencv2/opencv.hpp>
//#include <mosquitto.h>

/**** MCUInterface starts ****/

MCUInterface::MCUInterface() 
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , dev_fd_{-1}
{

}

MCUInterface::~MCUInterface()
{
    if(dev_fd_ >= 0) {
        close(dev_fd_);
    }
}

int MCUInterface::init()
{
    dev_fd_ = open_uart(HARDWARE_PATH_, BAUDRATE_);
}

int MCUInterface::open_uart(const std::string& dev, int baudrate)
{
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        logger_->error("open_uart: %s", strerror(errno));
        return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        logger_->error("tcgetattr: %s", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0; 
    tty.c_oflag = 0; 
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        logger_->error("tcsetattr: %s", strerror(errno));
        return -1;
    }

    return fd;
}

/**** MCUInterface ends ****/

/**** CameraMicrophoneInterface starts ****/

CameraMicrophoneInterface::CameraMicrophoneInterface()
{

}

CameraMicrophoneInterface::~CameraMicrophoneInterface()
{

}



/**** CameraMicrophoneInterface ends ****/

/**** PeripheralBroker starts ****/

PeripheralBroker::PeripheralBroker()
{
    logger_ = LoggerFactory::get_instance()->get_logger(LOGGER_NAME_);
    logger_->info("Initializing all peripherals...");
    init_all();
}

int PeripheralBroker::init_all()
{
    int ret;
    int all_okay = PERIPHERAL_STATUS_OK;

    ret = init_camera();
    if (ret != PERIPHERAL_STATUS_OK) {
        logger_->error("camera initialization failed");
        all_okay = PERIPHERAL_STATUS_ERROR;
    }

    ret = init_mcu();
    if (ret != PERIPHERAL_STATUS_OK) {
        logger_->error("MQTT over UART initialization failed");
        all_okay = PERIPHERAL_STATUS_ERROR;
    }

    if (all_okay == PERIPHERAL_STATUS_OK) {
        logger_->info("all peripherals successfully initialized");
    }
    else {
        logger_->warn("some peripherals failed to initialize");
    }

    return all_okay;
}

int PeripheralBroker::init_camera()
{
    logger_->info("Initializing camera");
    dev_camera_ = std::make_shared<CameraMicrophoneInterface>();
    return get_camera_status();
}

int PeripheralBroker::get_camera_status()
{
    if(!dev_camera_) {
        return PERIPHERAL_STATUS_NOT_SUPPORTED;
    }
    return PERIPHERAL_STATUS_OK;
}

int PeripheralBroker::init_mcu()
{
    return get_mcu_status();
}

int PeripheralBroker::get_mcu_status()
{
    if(!dev_mcu_) {
        return PERIPHERAL_STATUS_NOT_SUPPORTED;
    }
    return PERIPHERAL_STATUS_OK;
}

int PeripheralBroker::init_motor_driver()
{
    return get_motor_driver_status();
}

int PeripheralBroker::get_motor_driver_status()
{
    if(!dev_motor_driver_) {
        return PERIPHERAL_STATUS_NOT_SUPPORTED;
    }
    return PERIPHERAL_STATUS_OK;
}

/**** PeripheralBroker ends ****/