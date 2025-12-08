#include "app.h"
#include <chrono>
#include <thread>
#include <string_view>
#include "peripheralbroker.h"
#include "networkservice.h"

App::App()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , exit_flag_{false}
{}

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
            }; break;
            case EVENT_MCU_TEMPRATURE_SENSOR_READ: {
                handle_mcu_sensor_read();
            }; break;
            case EVENT_MCU_PROXIMITY_SENSOR_TRIGGERED: {
                handle_proximity_sensor_triggered();
            }; break;
            default:
                break;
        }
    };

    auto pull_event_queue = [&]() {
        bool is_busy = false;
        while(true) {
            event_t ev;
            {
                std::lock_guard<std::mutex> lk(high_priority_event_mutex_);
                if (high_priority_event_queue_.empty()) break;
                ev = high_priority_event_queue_.front();
                high_priority_event_queue_.pop();
            }
            handle_event(ev);
            is_busy = true;
        }

        {
            std::lock_guard<std::mutex> lk(low_priority_event_mutex_);
            if(!low_priority_event_queue_.empty()) {
                event_t ev = low_priority_event_queue_.front();
                low_priority_event_queue_.pop();
                handle_event(ev);
                is_busy = true;
            }
        }
        return is_busy;
    };

    auto check_time_event = [&]() {
        bool is_busy = false;
        while (true) {
            auto now = std::chrono::steady_clock::now();
            TimedEvent next;
            {
                std::lock_guard<std::mutex> lk(timed_event_mutex_);
                if (timed_event_queue_.empty()) break;
                next = timed_event_queue_.top();
                if (next.timestamp <= now) {
                    timed_event_queue_.pop();
                } else {
                    break;
                }
            }
            handle_event(next.event_handler);
            is_busy = true;
        }
        return is_busy;
    };

    // debug only: enqueue some events
    // high_priority_event_queue_.push(EVENT_MCU_TEMPRATURE_SENSOR_READ);
    // high_priority_event_queue_.push(EVENT_MCU_MOTION_SENSOR_TRIGGERED);
    
    logger_->info("main event loop begin");
    while(!exit_flag_) {
        busy_flag = false;
        switch(count % 8) {
            case 0:
            case 1: {
                check_time_event() ? busy_flag = true : busy_flag;
            }
            default: {
                pull_event_queue() ? busy_flag = true : busy_flag;
            }
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

void App::on_MCU_data(const std::vector<MCUInterface::DecodedMessage>& msg)
{
    SingletonT<App>::get_instance()->on_MCU_data_impl(msg);
}

void App::on_MCU_data_impl(const std::vector<MCUInterface::DecodedMessage>& msg)
{
    if (!msg.empty()) {
        std::for_each(
            msg.begin(), 
            msg.end(), 
            [this](const MCUInterface::DecodedMessage& m) {
                std::string_view sv_dev(m.dev);

                logger_->info("Decoded MCU message from device: {}", sv_dev);
                for (const auto& [key, value] : m.fields) {
                    logger_->info("  {}: {}", key, value);
                }

                if(m.dev == "temp_sensor") {
                    this->on_MCU_temperature_sensor_read({m});
                }
                else if(m.dev == "motion_sensor") {
                    this->on_MCU_motion_sensor_triggered({m});
                }
                else if(m.dev == "proximity_sensor") {
                    this->on_MCU_proximity_sensor_read({m});
                }
                else {
                    logger_->warn("Unknown MCU device: [{}]", m.dev);
                }
            }
        );
    } else {
        logger_->info("Received MCU data (0 bytes)");
    }
    // todo: decode the data from the MCU and enqueue corresponding events
}

void App::on_MCU_temperature_sensor_read(const MCUInterface::DecodedMessage& msg)
{
    logger_->debug("Temperature sensor reading received");
    mcu_temperature_celsius_ = msg.fields.count("temperature") ? std::stof(msg.fields.at("temperature")) : -999.0f;
    mcu_humidity_percent_ = msg.fields.count("humidity") ? std::stof(msg.fields.at("humidity")) : -999.0f;
    {
        std::lock_guard<std::mutex> lk(low_priority_event_mutex_);
        low_priority_event_queue_.push(EVENT_MCU_TEMPRATURE_SENSOR_READ);
    }
}

void App::on_MCU_motion_sensor_triggered(const MCUInterface::DecodedMessage& msg)
{
    logger_->debug("Motion sensor triggered");
    if(msg.fields.count("motion_detected") && std::stoi(msg.fields.at("motion_detected")) != 0) {
        std::lock_guard<std::mutex> lk(high_priority_event_mutex_);
        high_priority_event_queue_.push(EVENT_MCU_MOTION_SENSOR_TRIGGERED);
    }
}

void App::on_MCU_proximity_sensor_read(const MCUInterface::DecodedMessage& msg)
{
    logger_->debug("Proximity sensor reading received");
    {
        std::lock_guard<std::mutex> lk(high_priority_event_mutex_);
        high_priority_event_queue_.push(EVENT_MCU_PROXIMITY_SENSOR_TRIGGERED);
    }
}

void App::handle_mcu_timeout()
{
    logger_->info("[{}] not implemented", __func__);
}

void App::handle_mcu_sensor_read()
{
    logger_->info("{}", __func__);
    HTTPService::get_instance()->send_temperature_data(mcu_temperature_celsius_, mcu_humidity_percent_); 
}

void App::handle_motion_sensor_triggered()
{
    auto peripheral_broker = PeripheralBroker::get_instance();
    int ret = peripheral_broker->get_camera_image();
    if(ret != PERIPHERAL_STATUS_OK) {
        logger_->error("[{}] failed to get camera image", __func__);
    }
    else {
        logger_->info("[{}] camera image captured", __func__);
    }
}

void App::handle_proximity_sensor_triggered()
{
    logger_->info("[{}] not implemented", __func__);
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
