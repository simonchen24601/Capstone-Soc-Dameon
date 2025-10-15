#include "app.h"
#include <chrono>
#include <thread>

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
            case EVENT_READ_MCU_SENSOR: {
                read_mcu_sensor();
                time_event_queue_.push({
                    std::chrono::steady_clock::now() + std::chrono::seconds(5), 
                    EVENT_READ_MCU_SENSOR});
            }; break;
            case EVENT_READ_CAMERA_IMAGE: {
                read_camera_imge();
                time_event_queue_.push({
                    std::chrono::steady_clock::now() + std::chrono::seconds(10), 
                    EVENT_READ_CAMERA_IMAGE});
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

    event_queue_.push(EVENT_READ_MCU_SENSOR);
    event_queue_.push(EVENT_READ_CAMERA_IMAGE);
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

void App::read_mcu_sensor()
{
    logger_->info(__func__);
}

void App::read_camera_imge()
{
    logger_->info(__func__);
}
