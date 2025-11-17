#include "interface.h"
#include "configservice.h"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <filesystem>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <mqtt/async_client.h>
#include <sstream>
#include <fstream>

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
        if (r == 0) {
            // EOF / no more data available
            break;
        }
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
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , cap_{nullptr}
    , device_{}
{}

CameraMicrophoneInterface::~CameraMicrophoneInterface()
{
    // ensure streaming stopped and thread joined
    stop_stream_async();
    if (cap_ && cap_->isOpened()) cap_->release();
}

int CameraMicrophoneInterface::init(const std::string& device, int camera_width, int camera_height, int camera_framerate)
{
    device_ = device;
    camera_width_ = camera_width;
    camera_height_ = camera_height;
    camera_framerate_ = camera_framerate;
    try {
        if (device.empty()) {
            return -1;
        }

        // If device looks like an integer index, open by index
        bool opened = false;
        if (device.size() == 1 && isdigit(device[0])) {
            int idx = device[0] - '0';
            cap_ = std::make_unique<cv::VideoCapture>(idx, cv::CAP_ANY);
            opened = cap_->isOpened();
        } else {
            // Try to open device path using V4L2 backend
            cap_ = std::make_unique<cv::VideoCapture>(device, cv::CAP_V4L2);
            opened = cap_->isOpened();
            if (!opened) {
                // try generic open
                cap_ = std::make_unique<cv::VideoCapture>(device);
                opened = cap_->isOpened();
            }
        }

        if (!opened) {
            logger_->error("Camera init failed for device {}", device);
            cap_.reset();
            return -1;
        }

    // apply requested resolution and framerate if provided
    if (camera_width_ > 0) cap_->set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    if (camera_height_ > 0) cap_->set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    if (camera_framerate_ > 0) cap_->set(cv::CAP_PROP_FPS, camera_framerate_);

        logger_->info("Camera initialized: {}", device);
        return 0;
    }
    catch (const std::exception& e) {
        logger_->error("Camera init exception: {}", e.what());
        cap_.reset();
        return -1;
    }
}

void CameraMicrophoneInterface::get_image()
{
    if (!cap_ || !cap_->isOpened()) {
        // nothing to do
        return;
    }

    cv::Mat frame;
    if (!cap_->read(frame)) {
        logger_->error("Camera read failed");
        return;
    }

    std::filesystem::create_directories("./logs");
    std::string out = "./logs/capture.jpg";
    try {
        if (!cv::imwrite(out, frame)) {
            logger_->error("Failed to write image to {}", out);
        } else {
            logger_->info("Saved camera image to {}", out);
        }
    } catch (const std::exception& e) {
        logger_->error("Exception writing image: {}", e.what());
    }
}

void CameraMicrophoneInterface::start_stream_async()
{
    if (!cap_ || !cap_->isOpened()) {
        logger_->error("start_stream_async: camera not initialized");
        return;
    }

    if (streaming_.load()) {
        logger_->info("start_stream_async: already streaming");
        return;
    }

    // decide fps: prefer configured camera_framerate_, else fallback to 10
    stream_fps_ = (camera_framerate_ > 0) ? camera_framerate_ : 10;
    streaming_.store(true);

    stream_thread_ = std::thread([this]() {
        const auto frame_period = std::chrono::milliseconds(1000 / std::max(1, stream_fps_));
        size_t frame_idx = 0;
        logger_->info("camera stream thread started (fps={})", stream_fps_);
        while (streaming_.load()) {
            cv::Mat frame;
            bool ok = false;
            try {
                ok = cap_->read(frame);
            } catch (const std::exception& e) {
                logger_->error("stream read exception: {}", e.what());
            }

            if (!ok || frame.empty()) {
                // wait a bit but allow immediate stop
                std::unique_lock<std::mutex> lk(stream_mutex_);
                stream_cv_.wait_for(lk, frame_period, [this]{ return !streaming_.load(); });
                continue;
            }

            if (frame_callback_) {
                try {
                    frame_callback_(frame);
                } catch (const std::exception& e) {
                    logger_->error("frame callback exception: {}", e.what());
                }
            } else {
                // default: save one jpeg per second (every stream_fps_ frames)
                if ((frame_idx % std::max(1, stream_fps_)) == 0) {
                    std::filesystem::create_directories("./logs");
                    std::string out = "./logs/stream_" + std::to_string(frame_idx) + ".jpg";
                    try {
                        if (cv::imwrite(out, frame)) {
                            logger_->debug("stream: saved {}", out);
                        } else {
                            logger_->warn("stream: failed to save {}", out);
                        }
                    } catch (const std::exception& e) {
                        logger_->error("stream: save failed: {}", e.what());
                    }
                }
            }

            frame_idx++;

            // wait until next frame or stop
            std::unique_lock<std::mutex> lk(stream_mutex_);
            stream_cv_.wait_for(lk, frame_period, [this]{ return !streaming_.load(); });
        }
        logger_->info("camera stream thread exiting");
    });
}

void CameraMicrophoneInterface::stop_stream_async()
{
    if (!streaming_.load()) return;
    streaming_.store(false);
    stream_cv_.notify_all();
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }
    logger_->info("stopped camera stream");
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
    auto config = ConfigService::get_instance();
    dev_camera_->init(config->camera_device_, config->camera_width_, config->camera_height_, config->camera_framerate_);
    dev_camera_->get_image();
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

/*** MQTTInterface implementation ***/

class MQTTCallback : public virtual mqtt::callback
{
public:
    MQTTCallback(MQTTInterface* owner) : owner_(owner) {}
    void connection_lost(const std::string& cause) override {
        if (owner_) {
            auto lg = owner_->get_logger();
            if (lg) lg->warn("MQTT connection lost: {}", cause);
        }
    }
    void message_arrived(mqtt::const_message_ptr msg) override {
        if (owner_) owner_->handle_incoming(msg->get_topic(), msg->to_string());
    }
    void delivery_complete(mqtt::delivery_token_ptr tok) override {}

private:
    MQTTInterface* owner_;
};

MQTTInterface::MQTTInterface()
    : logger_{LoggerFactory::get_instance()->get_logger("mqtt")}
{ }

MQTTInterface::~MQTTInterface()
{
    try { disconnect(); } catch(...) {}
}

int MQTTInterface::init(const std::string& broker_uri,
                        const std::string& client_id,
                        const std::string& ca_file,
                        const std::string& cert_file,
                        const std::string& key_file)
{
    try {
        client_ = std::make_unique<mqtt::async_client>(broker_uri, client_id);

        mqtt::ssl_options sslopts;
        sslopts.set_trust_store(ca_file);
        sslopts.set_key_store(cert_file);
        sslopts.set_private_key(key_file);

        conn_opts_.set_clean_session(false);
        conn_opts_.set_ssl(sslopts);

        auto cb = std::make_shared<MQTTCallback>(this);
        client_->set_callback(*cb);

        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT init failed: {}", e.what());
        return -1;
    }
}

int MQTTInterface::connect()
{
    try {
        if (!client_) return -1;
        client_->connect(conn_opts_)->wait();
        connected_.store(true);
        logger_->info("MQTT connected");
        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT connect failed: {}", e.what());
        return -1;
    }
}

int MQTTInterface::disconnect()
{
    try {
        if (client_ && connected_.load()) {
            client_->disconnect()->wait();
            connected_.store(false);
            logger_->info("MQTT disconnected");
        }
        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT disconnect failed: {}", e.what());
        return -1;
    }
}

int MQTTInterface::publish(const std::string& topic, const std::string& payload, int qos)
{
    try {
        if (!client_ || !connected_.load()) return -1;
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        client_->publish(msg)->wait();
        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT publish failed: {}", e.what());
        return -1;
    }
}

int MQTTInterface::subscribe_ack(const std::string& request_topic, const std::string& response_topic)
{
    try {
        if (!client_ || !connected_.load()) return -1;
        response_topic_ = response_topic;
        client_->subscribe(request_topic, 1)->wait();
        logger_->info("Subscribed to {}", request_topic);
        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT subscribe failed: {}", e.what());
        return -1;
    }
}

bool MQTTInterface::parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon)
{
    // naive: support GGA and RMC
    try {
        if (line.size() < 6) return false;
        std::string type = line.substr(3,3);
        std::vector<std::string> f;
        std::istringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ',')) f.push_back(token);

        auto convert = [](const std::string& dm, const std::string& hemi)->std::string{
            if (dm.empty()) return std::string();
            // dm = ddmm.mmmm or dddmm.mmmm
            double val = std::stod(dm);
            int deg = static_cast<int>(val / 100);
            double minutes = val - deg*100;
            double dec = deg + minutes/60.0;
            if (hemi == "S" || hemi == "W") dec = -dec;
            char buf[64];
            snprintf(buf, sizeof(buf), "%.6f", dec);
            return std::string(buf);
        };

        if (type == "GGA") {
            if (f.size() > 5) {
                out_lat = convert(f[2], f[3]);
                out_lon = convert(f[4], f[5]);
                return !out_lat.empty() && !out_lon.empty();
            }
        } else if (type == "RMC") {
            if (f.size() > 6) {
                out_lat = convert(f[3], f[4]);
                out_lon = convert(f[5], f[6]);
                return !out_lat.empty() && !out_lon.empty();
            }
        }
    } catch (...) { }
    return false;
}

void MQTTInterface::handle_incoming(const std::string& topic, const std::string& payload)
{
    logger_->info("[SUB] Received message on {}: {}", topic, payload);
    if (response_topic_.empty()) return;
    try {
        // simple JSON construction without external dependency
        std::ostringstream oss;
        oss << "{\"ack\":\"ACK\",\"timestamp\":" << static_cast<long>(time(nullptr)) << "}";
        publish(response_topic_, oss.str(), 1);
        logger_->info("[PUB] ACK sent to {}", response_topic_);
    } catch (const std::exception& e) {
        logger_->error("Failed to send ACK: {}", e.what());
    }
}

int MQTTInterface::publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec)
{
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        logger_->error("Failed to open GPS file: {}", file_path);
        return -1;
    }
    std::string line;
    int count = 1;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::string lat, lon;
        if (parse_nmea_latlon(line, lat, lon)) {
            // construct JSON manually to avoid external dependency
            std::ostringstream oss;
            oss << "{"
                << "\"count\":" << count << ","
                << "\"app\":\"GROUP2\",";
            oss << "\"timestamp\":" << static_cast<long>(time(nullptr)) << ",";
            oss << "\"latitude\":\"" << lat << "\",";
            oss << "\"longitude\":\"" << lon << "\"";
            oss << "}";
            std::string payload = oss.str();
            publish(topic, payload, 1);
            logger_->info("Published GPS: {}", payload);
            count++;
        } else {
            logger_->warn("Skipped unparsable NMEA: {}", line);
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(publish_period_sec));
    }
    return 0;
}
