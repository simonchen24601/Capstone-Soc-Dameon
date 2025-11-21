#include "networkservice.h"


// /*** MQTTInterface implementation ***/

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

MQTTService::MQTTService()
    : logger_{LoggerFactory::get_instance()->get_logger("MQTTService")}
{ }

MQTTService::~MQTTService()
{
    try { disconnect(); } catch(...) {}
}

int MQTTService::init(const std::string& broker_uri,
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

int MQTTService::connect()
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

int MQTTService::disconnect()
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

int MQTTService::publish(const std::string& topic, const std::string& payload, int qos)
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

int MQTTService::subscribe_ack(const std::string& request_topic, const std::string& response_topic)
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

bool MQTTService::parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon)
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

void MQTTService::handle_incoming(const std::string& topic, const std::string& payload)
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

int MQTTService::publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec)
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

