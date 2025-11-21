#pragma once

#include <sstream>
#include <string>
#include <atomic>
#include <memory>
// #include <fstream>
// #include <mqtt/async_client.h>
#include <curl/curl.h>
#include "logger.h"

class HTTPService {
public:
    HTTPService() {};
    ~HTTPService() {};
};

class MQTTService {
public:
    MQTTService();
    ~MQTTService();

    // Initialize from explicit parameters (endpoint like "ssl://host:8883")
    int init(const std::string& broker_uri,
             const std::string& client_id,
             const std::string& ca_file,
             const std::string& cert_file,
             const std::string& key_file);

    int connect();
    int disconnect();

    // publish a payload (blocking)
    int publish(const std::string& topic, const std::string& payload, int qos = 1);

    // Subscribe to request_topic and automatically publish ACKs to response_topic
    int subscribe_ack(const std::string& request_topic, const std::string& response_topic);

    // Read GPS (NMEA) lines from file and publish parsed JSON messages periodically
    int publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec);
public:
    // expose a minimal getter so callback objects can log via the same logger
    std::shared_ptr<spdlog::logger> get_logger() { return logger_; }

    // message handler called from callback (made public so the callback can invoke it)
    void handle_incoming(const std::string& topic, const std::string& payload);

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::unique_ptr<mqtt::async_client> client_;
    mqtt::connect_options conn_opts_;
    std::string response_topic_;
    std::atomic_bool connected_{false};
    // parse minimal NMEA (GGA or RMC) and return pair lat,lon as strings or empty
    static bool parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon);

};
