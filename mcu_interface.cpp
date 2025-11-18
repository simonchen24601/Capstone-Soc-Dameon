#include "mcu_interface.h"
#include "configservice.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

MCUInterface::MCUInterface()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , dev_fd_{-1}
{
}

MCUInterface::~MCUInterface()
{
    if (dev_fd_ >= 0) close(dev_fd_);
}

static inline int baudrate_to_constant(int baud)
{
    switch(baud) {
#ifdef B0
    case 0: return B0;
#endif
#ifdef B50
    case 50: return B50;
#endif
#ifdef B75
    case 75: return B75;
#endif
#ifdef B110
    case 110: return B110;
#endif
#ifdef B134
    case 134: return B134;
#endif
#ifdef B150
    case 150: return B150;
#endif
#ifdef B200
    case 200: return B200;
#endif
#ifdef B300
    case 300: return B300;
#endif
#ifdef B600
    case 600: return B600;
#endif
#ifdef B1200
    case 1200: return B1200;
#endif
#ifdef B1800
    case 1800: return B1800;
#endif
#ifdef B2400
    case 2400: return B2400;
#endif
#ifdef B4800
    case 4800: return B4800;
#endif
#ifdef B9600
    case 9600: return B9600;
#endif
#ifdef B19200
    case 19200: return B19200;
#endif
#ifdef B38400
    case 38400: return B38400;
#endif
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
    default:
        return B115200;
    }
}

int MCUInterface::init()
{
    const ConfigService* cfg = ConfigService::get_instance();
    std::string dev = HARDWARE_PATH_DEFAULT_;
    int baud_const = BAUDRATE_DEFAULT_;

    if (cfg && cfg->enable_mcu_) {
        // if configured for USB mode, we don't open UART
        std::string mode = cfg->mcu_mode_;
        for (auto &c : mode) c = static_cast<char>(toupper(c));
        if (mode == "USB") {
            logger_->info("MCU init: configured in USB mode, skipping UART open");
            dev_fd_ = -1;
            return 0;
        }

        if (!cfg->mcu_device_.empty()) dev = cfg->mcu_device_;
        if (cfg->mcu_baudrate_ > 0) {
            baud_const = baudrate_to_constant(cfg->mcu_baudrate_);
        }
    }

    dev_fd_ = open_uart(dev, baud_const);
    return (dev_fd_ >= 0) ? 0 : -1;
}

ssize_t MCUInterface::write_bytes(const void* data, size_t len)
{
    if (dev_fd_ < 0) {
        logger_->error("write_bytes: device not opened");
        return -1;
    }

    const uint8_t* buf = reinterpret_cast<const uint8_t*>(data);
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t w = write(dev_fd_, buf, remaining);
        if (w < 0) {
            if (errno == EINTR) continue;
            logger_->error("write_bytes: %s", strerror(errno));
            return -1;
        }
        buf += w;
        remaining -= static_cast<size_t>(w);
    }
    return static_cast<ssize_t>(len);
}

ssize_t MCUInterface::read_bytes(void* data, size_t len)
{
    if (dev_fd_ < 0) {
        logger_->error("read_bytes: device not opened");
        return -1;
    }

    uint8_t* buf = reinterpret_cast<uint8_t*>(data);
    size_t received = 0;
    while (received < len) {
        ssize_t r = read(dev_fd_, buf + received, len - received);
        if (r < 0) {
            if (errno == EINTR) continue;
            logger_->error("read_bytes: %s", strerror(errno));
            return -1;
        }
        if (r == 0) break;
        received += static_cast<size_t>(r);
    }
    return static_cast<ssize_t>(received);
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
        close(fd);
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
        close(fd);
        return -1;
    }

    return fd;
}
