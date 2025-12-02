// Network service interfaces: lightweight HTTP and MQTT wrappers.
// Keep implementation details in networkservice.cpp; header stays minimal.
#pragma once

#include <sstream>
#include <string>
#include <atomic>
#include <memory>
#include <functional>
#include <mutex>
#include <optional>
// third-party
#include <mqtt/async_client.h>
#include <curl/curl.h>
#include "logger.h"
#include "util.hpp"

// HTTPService: simple singleton wrapper around libcurl for GET/POST.
class HTTPService : public SingletonT<HTTPService> {
    friend class SingletonT<HTTPService>;
public:
    HTTPService();
    ~HTTPService();

    // Initialize HTTP client (returns 0 on success)
    int init(std::string server_url, std::string api_key, const std::function<void(const std::string&)>& response_callback);
    // POST temperature reading as JSON.
    int send_temperature_data(float temperature_celsius, const std::optional<std::string>& timestamp_iso = std::nullopt);
    // Placeholder for image data upload (not implemented yet).
    int send_camera_image_data(const std::vector<uint8_t>& image_data, const std::optional<std::string>& timestamp_iso = std::nullopt) 
        {return -1;};

private:
    // Basic GET request; writes body and optional status code; 0 on success.
    int server_request(const std::string& url, std::string& out_body, long* out_http_code = nullptr);
    // Extended request with pre/post lambdas to set/reset curl options.
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

// MQTTService: AWS IoT Core style MQTT client singleton.
class MQTTService : public SingletonT<MQTTService> {
    friend class SingletonT<MQTTService>;
public:
    MQTTService();
    ~MQTTService();

    // Configure client (endpoint, certs, callback); returns 0 on success.
    int init(
        const std::string& aws_iot_core_endpoint,
        const std::string& client_id,
        const std::string& cert_path,
        const std::string& key_path,
        const std::function<void(const std::string&, long)>& callback);
    
    // Establish MQTT connection.
    int connect();
    // Graceful disconnect.
    int disconnect();      
    // Blocking publish.
    int publish(const std::string& topic, const std::string& payload, int qos = 1); 
    // Sub + auto ACK.
    int subscribe_ack(const std::string& request_topic, const std::string& response_topic); 
    // Internal callback hook.    
    void handle_incoming(const std::string& topic, const std::string& payload); 

private:
    const char* LOGGER_NAME_ = "MQTTService";
    std::shared_ptr<spdlog::logger> logger_;
    // AWS IoT Core endpoint and creds
    std::string aws_iot_core_endpoint_;
    std::string client_id_;
    std::string cert_path_;
    std::string key_path_;
    //
    std::function<void(const std::string&, long)> resp_cal_bck_;
    std::unique_ptr<mqtt::async_client> client_;
    mqtt::connect_options conn_opts_;
    std::string response_topic_;
    std::atomic_bool connected_{false};
};
