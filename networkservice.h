#pragma once

#include <sstream>
#include <string>
#include <atomic>
#include <memory>
#include <functional>
#include <mutex>
#include <optional>
// third-party
// #include <mqtt/async_client.h>
#include <curl/curl.h>
#include "logger.h"
#include "util.hpp"

class HTTPService : public SingletonT<HTTPService> {
    friend class SingletonT<HTTPService>;
public:
    HTTPService();
    ~HTTPService();

    // Initialize the HTTP client (returns 0 on success)
    int init(std::string server_url, std::string api_key, const std::function<void(const std::string&)>& response_callback);
    int send_temperature_data(float temperature_celsius, const std::optional<std::string>& timestamp_iso = std::nullopt);

private:
    // Perform a simple HTTP GET request to `url` and return response body.
    // If `out_http_code` is provided, the HTTP status code will be written.
    // Returns 0 on success, non-zero on error.
    int server_request(const std::string& url, std::string& out_body, long* out_http_code = nullptr);
    // Extended variant allowing callers to configure CURL options (e.g., headers, POST fields)
    // under the same internal lock before the request, and clean up/reset after.
    int server_request(
        const std::string& url,
        std::string& out_body,
        long* out_http_code,
        const std::function<void(CURL*)>& preconfigure,
        const std::function<void(CURL*)>& postcleanup);

private:
    const char* LOGGER_NAME_ = "HTTPService";
    std::string server_url_;
    std::string api_key_;
    // API endpoints
    const char* API_URL_DATA_ = "/iot/data";
    const char* API_URL_LOG_  = "/iot/log";
    const char* API_URL_COMMAND_ = "/iot/command";
    std::shared_ptr<spdlog::logger> logger_;
    CURL* curl_handle_;
    std::mutex curl_mutex_;
    std::function<void(const std::string&)> resp_cal_bck_;
};

// class MQTTService : public SingletonT<MQTTService> {
//     friend class SingletonT<MQTTService>;
// public:
//     MQTTService();
//     ~MQTTService();

//     // Initialize the HTTP client (returns 0 on success).
//     // Optional callback will be invoked for responses: callback(body, http_code).
//     int init(const std::function<void(const std::string&, long)>& callback = nullptr);

//     // Perform a simple HTTP GET request to `url` and return response body.
//     // If `out_http_code` is provided, the HTTP status code will be written.
//     // Returns 0 on success, non-zero on error.
//     int server_request(const std::string& url, std::string& out_body, long* out_http_code = nullptr);
//     int connect();
//     int disconnect();

//     // publish a payload (blocking)
//     int publish(const std::string& topic, const std::string& payload, int qos = 1);

//     // Subscribe to request_topic and automatically publish ACKs to response_topic
//     int subscribe_ack(const std::string& request_topic, const std::string& response_topic);

//     // Read GPS (NMEA) lines from file and publish parsed JSON messages periodically
//     int publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec);
// public:
//     // expose a minimal getter so callback objects can log via the same logger
//     std::shared_ptr<spdlog::logger> get_logger() { return logger_; }

//     // message handler called from callback (made public so the callback can invoke it)
//     void handle_incoming(const std::string& topic, const std::string& payload);

// private:
//     const char* LOGGER_NAME_ = "MQTTService";
//     std::shared_ptr<spdlog::logger> logger_;
//     std::unique_ptr<mqtt::async_client> client_;
//     mqtt::connect_options conn_opts_;
//     std::string response_topic_;
//     std::atomic_bool connected_{false};
//     // parse minimal NMEA (GGA or RMC) and return pair lat,lon as strings or empty
//     static bool parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon);

// };
