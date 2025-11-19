#include "mcu_interface.h"
#include "configservice.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <chrono>

MCUInterface::MCUInterface()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , dev_fd_{-1}
{}

MCUInterface::~MCUInterface()
{
    stop_background();
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

int MCUInterface::init(const std::string& device, int baudrate)
{
    // std::string mode = cfg->mcu_mode_;
    // for (auto &c : mode) c = static_cast<char>(toupper(c));
    // if (mode == "USB") {
    //     logger_->info("MCU init: configured in USB mode, skipping UART open");
    //     dev_fd_ = -1;
    //     return 0;
    // }

    dev_fd_ = open_uart(device, baudrate_to_constant(baudrate));
    if (dev_fd_ < 0) return -1;

    // start background threads for read/write
    running_.store(true);

    // write thread
    write_thread_ = std::thread([this]() {
        while (running_.load()) {
            std::vector<uint8_t> pkt;
            {
                std::unique_lock<std::mutex> lk(write_mutex_);
                write_cv_.wait(lk, [this]{ return !write_queue_.empty() || !running_.load(); });
                if (!running_.load() && write_queue_.empty()) break;
                pkt = std::move(write_queue_.front());
                write_queue_.pop();
            }
            if (!pkt.empty()) {
                ssize_t w = write_bytes(pkt.data(), pkt.size());
                if (w < 0) {
                    logger_->error("async write failed");
                }
            }
        }
    });

    // read thread
    read_thread_ = std::thread([this]() {
        const size_t BUF_SZ = 1024;
        std::vector<uint8_t> buf(BUF_SZ);
        while (running_.load()) {
            ssize_t r = read_bytes(buf.data(), buf.size());
            if (r > 0) {
                std::vector<uint8_t> got(buf.begin(), buf.begin() + r);
                if (read_callback_) {
                    try { read_callback_(got); } catch (const std::exception& e) {
                        logger_->error("read callback exception: {}", e.what());
                    } catch (...) {
                        logger_->error("read callback unknown exception");
                    }
                }
            } else {
                // small sleep to avoid tight loop on read failures
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    return 0;
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

int MCUInterface::write_bytes_async(const std::vector<uint8_t>& data)
{
    if (dev_fd_ < 0) return -1;
    if (!running_.load()) return -1;
    {
        std::lock_guard<std::mutex> lk(write_mutex_);
        write_queue_.push(data);
    }
    write_cv_.notify_one();
    return 0;
}

void MCUInterface::set_read_callback(const std::function<void(const std::vector<uint8_t>&)>& cb)
{
    read_callback_ = cb;
}

void MCUInterface::stop_background()
{
    if (!running_.load()) {
        // still ensure write thread will not be blocked
        write_cv_.notify_all();
        return;
    }
    running_.store(false);
    write_cv_.notify_all();
    if (write_thread_.joinable()) write_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
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
        logger_->error("open_uart: {}", strerror(errno));
        return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        logger_->error("tcgetattr: {}", strerror(errno));
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
        logger_->error("tcsetattr: {}", strerror(errno));
        close(fd);
        return -1;
    }

    logger_->info("MCU connected, opened {} at baudrate {}, fd {}", dev, baudrate, fd);
    return fd;
}
