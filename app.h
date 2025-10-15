/*
* simonchen24601@gmail.com Simon Chen 2025
*/
#pragma once
#include <queue>
#include <ctime>
#include <functional>
#include <chrono>
#include "logger.h"

class App {
public:
    App();
    void run();
    void stop();

private:
    void on_camera_read() {};
    void on_MCU_msg() {};
    void on_MCU_motion_sensor_triggered() {};
    void on_MCU_proximity_sensor_read() {};
    void on_MCU_temperature_sensor_read() {};
    void on_MCU_humidity_sensor_read() {};

    void read_mcu_sensor();
    void read_camera_imge();

private:
    const char* LOGGER_NAME_ = "App";
    std::shared_ptr<spdlog::logger> logger_;

    enum event_t {
        EVENT_IDLE,
        EVENT_READ_MCU_SENSOR,
        EVENT_READ_CAMERA_IMAGE,
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

    std::priority_queue<
        TimedEvent,
        std::vector<TimedEvent>,
        TimedEventCompare
    > time_event_queue_;
    std::queue<event_t> event_queue_;

    bool exit_flag_;
};
