#pragma once
#include <string>
#include <vector>
#include "util.hpp"
#include "spdlog/spdlog.h"

#include <string>
#include <vector>
#include <memory>

class LoggerFactory : public SingletonT<LoggerFactory>
{
    friend class SingletonT<LoggerFactory>;
public:
    int init();
    std::shared_ptr<spdlog::logger> get_logger(const std::string &name);

private:
    LoggerFactory() = default;
    ~LoggerFactory() = default;

private:
    spdlog::level::level_enum default_level_;
	std::vector<spdlog::sink_ptr> m_sinks;
};
