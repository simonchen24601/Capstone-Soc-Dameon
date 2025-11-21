#pragma once
#include <string>
#include <memory>
#include "logger.h"

class MotorDriverInterface {
public:
    MotorDriverInterface();
    void init();
private:
    const char* LOGGER_NAME_ = "MotorDriver";
    std::shared_ptr<spdlog::logger> logger_;
};
