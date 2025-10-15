#include "logger.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/daily_file_sink.h"
#include <filesystem>

int LoggerFactory::init()
{
	std::filesystem::create_directories("./logs");

    m_sinks.clear();
	m_sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	m_sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_mt>("./logs/soc_deamon.log", 23, 59));

	return 0;
}

std::shared_ptr<spdlog::logger> LoggerFactory::get_logger(const std::string &name)
{
	auto combined_logger = spdlog::get(name);
	if (!combined_logger)
	{
		combined_logger = std::make_shared<spdlog::logger>(name, std::begin(m_sinks), std::end(m_sinks));
		spdlog::register_logger(combined_logger);
        combined_logger->set_level(spdlog::level::debug);
		combined_logger->flush_on(spdlog::level::info);
	}

	return combined_logger;
}
