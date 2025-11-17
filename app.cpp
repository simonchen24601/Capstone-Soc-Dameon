#include "app.h"
#include <chrono>
#include <thread>
#include "peripheralbroker.h"

App::App()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , exit_flag_{false}
{

}

void App::run()
{
    bool busy_flag;
    uint32_t count = 0;

    auto handle_event = [&](int event) {
        switch(event) {
            case EVENT_MCU_HEARTBEAT_TIMEOUT: {
                logger_->critical("MCU heartbeat timeout");
            }; break;
            case EVENT_MCU_MOTION_SENSOR_TRIGGERED: {
                handle_motion_sensor_triggered();
                // time_event_queue_.push({
                //     std::chrono::steady_clock::now() + std::chrono::seconds(10), 
                //     EVENT_MCU_MOTION_SENSOR_TRIGGERED});
            }; break;
            case EVENT_MCU_TEMPRATURE_SENSOR_READ: {
                handle_mcu_sensor_read();
                time_event_queue_.push({
                    std::chrono::steady_clock::now() + std::chrono::seconds(TEMPRATURE_SENSOR_READ_INTERVAL_SEC_), 
                    EVENT_MCU_TEMPRATURE_SENSOR_READ});
            }; break;
            case EVENT_MCU_PROXIMITY_SENSOR_TRIGGERED: {
                handle_proximity_sensor_triggered();
            }; break;
            default:
                break;
        }
    };

    auto pull_event_queue = [&]() {
        while(!event_queue_.empty()) {
            event_t ev = event_queue_.back();
            event_queue_.pop();
            handle_event(ev);
        }
    };

    auto check_time_event = [&]() {
        while (!time_event_queue_.empty()) {
            auto now = std::chrono::steady_clock::now();
            auto next = time_event_queue_.top();
            if (next.timestamp <= now) {
                time_event_queue_.pop();
                handle_event(next.event_handler);
            }
            else {
                break;
            }
        }
    };

    event_queue_.push(EVENT_MCU_TEMPRATURE_SENSOR_READ);
    event_queue_.push(EVENT_MCU_MOTION_SENSOR_TRIGGERED);
    logger_->info("main event loop begin");
    while(!exit_flag_) {
        busy_flag = false;
        switch(count % 8) {
            case 0:
            case 1:
                check_time_event();
            default:
                pull_event_queue();
        }

        if(!busy_flag) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        count++;
    }
}

void App::stop()
{
    exit_flag_ = true;
}

void App::handle_mcu_timeout()
{
    logger_->info("%s not implemented", __func__);
}

void App::handle_mcu_sensor_read()
{
    logger_->info("%s not implemented", __func__);
}

void App::handle_motion_sensor_triggered()
{
    logger_->info("%s not implemented", __func__);
}

void App::handle_proximity_sensor_triggered()
{
    logger_->info("%s not implemented", __func__);
}

void App::handle_server_streaming_begin()
{
    auto peripheral_broker = PeripheralBroker::get_instance();
    int ret = peripheral_broker->start_camera_streaming();
    if(ret != PERIPHERAL_STATUS_OK) {
        logger_->error("failed to start camera streaming");
    }
    else {
        logger_->info("camera streaming started");
    }
}

void App::handle_server_streaming_end()
{
    auto peripheral_broker = PeripheralBroker::get_instance();
    int ret = peripheral_broker->stop_camera_streaming();
    if(ret != PERIPHERAL_STATUS_OK) {
        logger_->error("failed to stop camera streaming");
    }
    else {
        logger_->info("camera streaming stopped");
    }
}
