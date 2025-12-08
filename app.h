/*
* simonchen24601@gmail.com Simon Chen 2025
*/
#pragma once
#include <queue>
#include <ctime>
#include <functional>
#include <chrono>
#include <mutex>
#include "logger.h"
#include "peripheralbroker.h"

class App {
public:
    App();
    void run();
    void stop();
    static void on_MCU_data(const std::vector<MCUInterface::DecodedMessage>&);    // static wrapper for MCU data callback

private:
    // MCU callbacks
    inline void on_MCU_data_impl(const std::vector<MCUInterface::DecodedMessage>&); // called on MCU read, not thread-safe, message may be partial
    void on_MCU_motion_sensor_triggered(const MCUInterface::DecodedMessage&);
    void on_MCU_proximity_sensor_read(const MCUInterface::DecodedMessage&);
    void on_MCU_temperature_sensor_read(const MCUInterface::DecodedMessage&);

    // event handlers
    void handle_mcu_timeout();
    void handle_mcu_sensor_read();
    void handle_motion_sensor_triggered();
    void handle_proximity_sensor_triggered();
    void handle_server_streaming_begin();
    void handle_server_streaming_end();

private:
    static constexpr int TEMPRATURE_SENSOR_READ_INTERVAL_SEC_ = 5;

    const char* LOGGER_NAME_ = "App";
    std::shared_ptr<spdlog::logger> logger_;

    float mcu_temperature_celsius_{0.0f};
    float mcu_humidity_percent_{0.0f};

    enum event_t {
        EVENT_IDLE,
        EVENT_MCU_HEARTBEAT_TIMEOUT,
        EVENT_MCU_MOTION_SENSOR_TRIGGERED,
        EVENT_MCU_PROXIMITY_SENSOR_TRIGGERED,
        EVENT_MCU_TEMPRATURE_SENSOR_READ,   // temperature sensor read serve as heartbeat
        EVENT_SERVER_STREAMING_BEGIN,
        EVENT_SERVER_STREAMING_END
    };

    struct TimedEvent {
        std::chrono::steady_clock::time_point timestamp;
        event_t event_handler;
    };

    struct TimedEventCompare {
        bool operator()(const TimedEvent& a, const TimedEvent& b) const {
            return a.timestamp > b.timestamp;
        }
    };

    /**** event queues ****/

    std::priority_queue<
        TimedEvent,
        std::vector<TimedEvent>,
        TimedEventCompare
    > timed_event_queue_;
    std::mutex timed_event_mutex_;
    std::queue<event_t> high_priority_event_queue_;
    std::mutex high_priority_event_mutex_;
    std::queue<event_t> low_priority_event_queue_;
    std::mutex low_priority_event_mutex_;

    bool exit_flag_;
};
