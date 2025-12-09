#include "networkservice.h"
#include <fstream>
#include <iomanip>
#include <ctime>
#include <curl/curl.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

/**** NetworkService starts ****/

HTTPService::HTTPService() 
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , curl_handle_{nullptr} 
{}

HTTPService::~HTTPService() {
    if (curl_handle_) {
        curl_easy_cleanup(curl_handle_);
        curl_handle_ = nullptr;
    }
}

int HTTPService::init(std::string server_url, std::string api_key, const std::function<void(const std::string&)>& response_callback)
{
    server_url_ = std::move(server_url);
    api_key_ = std::move(api_key);
    resp_cal_bck_ = response_callback;

    curl_handle_ = curl_easy_init();
    if (!curl_handle_) return -1;
    // basic defaults can be set here (follow redirects, timeouts, etc.)
    return 0;
}

static size_t http_write_cb(char* ptr, size_t size, size_t nmemb, void* userdata)
{
    auto* out = reinterpret_cast<std::string*>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
}

int HTTPService::server_request(
    const std::string& url,
    std::string& out_body,
    long* out_http_code,
    const std::function<void(CURL*)>& preconfigure,
    const std::function<void(CURL*)>& postcleanup)
{
    if (!curl_handle_) return -1;
    std::lock_guard<std::mutex> lk(curl_mutex_);

    out_body.clear();
    curl_easy_setopt(curl_handle_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, http_write_cb);
    curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, &out_body);
    curl_easy_setopt(curl_handle_, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl_handle_, CURLOPT_TIMEOUT, 10L);

    if (preconfigure) preconfigure(curl_handle_);

    CURLcode rc = curl_easy_perform(curl_handle_);

    if (postcleanup) postcleanup(curl_handle_);

    if (rc != CURLE_OK) {
        return static_cast<int>(rc);
    }
    long code = 0;
    curl_easy_getinfo(curl_handle_, CURLINFO_RESPONSE_CODE, &code);
    if (out_http_code) *out_http_code = code;
    if (resp_cal_bck_ && code >= 200 && code < 300) {
        try { 
            resp_cal_bck_(out_body); 
        } 
        catch (...) 
        { /* swallow callback exceptions */ }
    }
    return 0;
}

int HTTPService::server_request(const std::string& url, std::string& out_body, long* out_http_code)
{
    // Delegate to extended variant without extra configuration
    return server_request(url, out_body, out_http_code, nullptr, nullptr);
}

static std::string now_utc_iso8601()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

int HTTPService::send_temperature_data(float temperature_celsius, float humidity_percent)
{
    if (!curl_handle_) return -1;
    // Use api_key_ as device_id per prior design
    std::string device_id = api_key_;
    while (!device_id.empty() && (device_id.back()=='\n' || device_id.back()=='\r' || device_id.back()==' ')) device_id.pop_back();

    std::string response;
    int rc = post_temperature(device_id,
                              static_cast<double>(temperature_celsius),
                              static_cast<double>(humidity_percent),
                              response);
    if (rc == 0) {
        // Best-effort parse/log
        try {
            std::istringstream iss(response);
            boost::property_tree::ptree resp_tree;
            boost::property_tree::read_json(iss, resp_tree);
            auto id_opt = resp_tree.get_optional<int>("id");
            auto ts_opt = resp_tree.get_optional<std::string>("timestamp");
            if (id_opt || ts_opt) {
                logger_->info("/temperature created id={} ts={}", id_opt.value_or(-1), ts_opt.value_or(""));
            }
        } catch (...) {
            logger_->warn("Failed to parse /temperature response JSON");
        }
    }
    return rc;
}

int HTTPService::post_temperature(const std::string& device_id,
                                  double temperature_celsius,
                                  double humidity_percent,
                                  std::string &out_response_body)
{
    if (!curl_handle_) return -1;
    std::string url = server_url_;
    if (!url.empty() && url.back() == '/') url.pop_back();
    url += std::string(API_URL_TEMPERATURE_);

    boost::property_tree::ptree payload_tree;
    payload_tree.put("device_id", device_id);
    payload_tree.put("temperature_celsius", temperature_celsius);
    payload_tree.put("humidity_percent", humidity_percent);
    std::ostringstream body_oss;
    boost::property_tree::write_json(body_oss, payload_tree, false);
    std::string payload = body_oss.str();
    if (!payload.empty() && (payload.back()=='\n' || payload.back()=='\r')) payload.pop_back();

    struct curl_slist* headers = nullptr;
    std::string resp;
    long http_code = 0;

    auto pre = [&](CURL* h){
        headers = curl_slist_append(headers, "Content-Type: application/json");
        if (!api_key_.empty()) {
            std::string api_header = std::string("X-API-Key: ") + api_key_;
            headers = curl_slist_append(headers, api_header.c_str());
        }
        curl_easy_setopt(h, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(h, CURLOPT_POST, 1L);
        curl_easy_setopt(h, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(h, CURLOPT_POSTFIELDSIZE, payload.size());
        curl_easy_setopt(h, CURLOPT_WRITEDATA, &resp);
    };
    auto post = [&](CURL* h){
        curl_easy_setopt(h, CURLOPT_POST, 0L);
        curl_easy_setopt(h, CURLOPT_POSTFIELDS, nullptr);
        curl_easy_setopt(h, CURLOPT_POSTFIELDSIZE, 0L);
        curl_easy_setopt(h, CURLOPT_HTTPHEADER, nullptr);
        if (headers) { curl_slist_free_all(headers); headers = nullptr; }
    };

    int rc = server_request(url, resp, &http_code, pre, post);
    out_response_body = resp;
    if (rc != 0) return rc;
    return (http_code >= 200 && http_code < 300) ? 0 : static_cast<int>(http_code ? http_code : -1);
}

int HTTPService::post_log(const std::string& device_id,
                          const std::string& message,
                          std::string &out_response_body)
{
    if (!curl_handle_) return -1;
    std::string url = server_url_;
    if (!url.empty() && url.back() == '/') url.pop_back();
    url += std::string(API_URL_LOG_);

    boost::property_tree::ptree payload_tree;
    payload_tree.put("device_id", device_id);
    payload_tree.put("message", message);
    std::ostringstream body_oss;
    boost::property_tree::write_json(body_oss, payload_tree, false);
    std::string payload = body_oss.str();
    if (!payload.empty() && (payload.back()=='\n' || payload.back()=='\r')) payload.pop_back();

    struct curl_slist* headers = nullptr;
    std::string resp;
    long http_code = 0;

    auto pre = [&](CURL* h){
        headers = curl_slist_append(headers, "Content-Type: application/json");
        if (!api_key_.empty()) {
            std::string api_header = std::string("X-API-Key: ") + api_key_;
            headers = curl_slist_append(headers, api_header.c_str());
        }
        curl_easy_setopt(h, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(h, CURLOPT_POST, 1L);
        curl_easy_setopt(h, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(h, CURLOPT_POSTFIELDSIZE, payload.size());
        curl_easy_setopt(h, CURLOPT_WRITEDATA, &resp);
    };
    auto post = [&](CURL* h){
        curl_easy_setopt(h, CURLOPT_POST, 0L);
        curl_easy_setopt(h, CURLOPT_POSTFIELDS, nullptr);
        curl_easy_setopt(h, CURLOPT_POSTFIELDSIZE, 0L);
        curl_easy_setopt(h, CURLOPT_HTTPHEADER, nullptr);
        if (headers) { curl_slist_free_all(headers); headers = nullptr; }
    };

    int rc = server_request(url, resp, &http_code, pre, post);
    out_response_body = resp;
    if (rc != 0) return rc;
    return (http_code >= 200 && http_code < 300) ? 0 : static_cast<int>(http_code ? http_code : -1);
}

/**** NetworkService ends ****/

/**** MQTTService starts ****/

MQTTService::MQTTService()
    : logger_{LoggerFactory::get_instance()->get_logger("MQTTService")}
{}

MQTTService::~MQTTService()
{
    try { disconnect(); } catch(...) {}
}

int MQTTService::init(const std::string& aws_iot_core_endpoint,
        const std::string& client_id,
        const std::string& cert_path,
        const std::string& key_path,
        const std::function<void(const std::string&, long)>& callback)
{
    aws_iot_core_endpoint_ = aws_iot_core_endpoint;
    client_id_ = client_id;
    cert_path_ = cert_path;
    key_path_ = key_path;
    resp_cal_bck_ = callback;
    
    try {
        client_ = std::make_unique<mqtt::async_client>(aws_iot_core_endpoint_, client_id_);

        mqtt::ssl_options sslopts;
        // Treat cert_path_ as both trust store (CA) and client certificate if separate CA not provided.
        sslopts.set_trust_store(cert_path_); // CA/root (if file contains it)
        sslopts.set_key_store(cert_path_);   // client certificate
        sslopts.set_private_key(key_path_);  // client private key
        sslopts.set_enable_server_cert_auth(true);

        conn_opts_.set_clean_session(true);
        conn_opts_.set_automatic_reconnect(true);
        conn_opts_.set_keep_alive_interval(20);
        conn_opts_.set_ssl(sslopts);

        // Minimal callback impl
        class CB : public virtual mqtt::callback {
        public:
            explicit CB(MQTTService* owner) : owner_(owner) {}
            void connection_lost(const std::string& cause) override {
                if (owner_) owner_->logger_->warn("MQTT connection lost: {}", cause);
                owner_->connected_.store(false);
            }
            void message_arrived(mqtt::const_message_ptr msg) override {
                if (owner_) owner_->handle_incoming(msg->get_topic(), msg->to_string());
            }
            void delivery_complete(mqtt::delivery_token_ptr) override {}
        private:
            MQTTService* owner_;
        };
        client_->set_callback(*new CB(this)); // leaking small object intentionally; acceptable for daemon lifetime
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
        auto tok = client_->connect(conn_opts_);
        tok->wait();
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
        logger_->info("MQTT published topic='{}' bytes={} qos={}", topic, payload.size(), qos);
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
        logger_->info("MQTT subscribed to {} (ack topic: {})", request_topic, response_topic_);
        return 0;
    } catch (const std::exception& e) {
        logger_->error("MQTT subscribe failed: {}", e.what());
        return -1;
    }
}

void MQTTService::handle_incoming(const std::string& topic, const std::string& payload)
{
    logger_->info("[SUB] {} -> {}", topic, payload);
    if (response_topic_.empty()) return;
    try {
        // Build simple ACK JSON
        std::ostringstream oss;
        oss << "{\"ack\":\"ACK\",\"timestamp\":" << static_cast<long>(time(nullptr)) << "}";
        publish(response_topic_, oss.str(), 1);
        logger_->info("ACK published to {}", response_topic_);
    } catch (const std::exception& e) {
        logger_->error("Failed to publish ACK: {}", e.what());
    }
}

/**** MQTTService end ****/