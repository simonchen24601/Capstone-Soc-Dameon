#include "logger.h"
#include "configservice.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/daily_file_sink.h"
#include <filesystem>
#include <algorithm>
#include <cctype>

static inline spdlog::level::level_enum parse_level(const std::string &lvl)
{
	std::string s = lvl;
	std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
	if (s == "trace") return spdlog::level::trace;
	if (s == "debug") return spdlog::level::debug;
	if (s == "info") return spdlog::level::info;
	if (s == "warn" || s == "warning") return spdlog::level::warn;
	if (s == "err" || s == "error") return spdlog::level::err;
	if (s == "critical") return spdlog::level::critical;
	if (s == "off") return spdlog::level::off;
	return spdlog::level::debug;
}

int LoggerFactory::init()
{
	auto config = ConfigService::get_instance();

	// Determine log directory and filename
	std::string log_dir = "./logs";
	std::string log_file = "soc_deamon.log";
	bool enable_file = true;
	std::string level_str = "debug";

	if (config) {
		enable_file = config->enable_file_logging_;
		log_dir = config->log_directory_.empty() ? log_dir : config->log_directory_;
		log_file = config->log_file_name_.empty() ? log_file : config->log_file_name_;
		level_str = config->log_level_.empty() ? level_str : config->log_level_;
	}

	if (enable_file) {
		std::filesystem::create_directories(log_dir);
	}

	m_sinks.clear();
	// console sink always present
	m_sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

	if (enable_file) {
		std::string fullpath = log_dir;
		if (!fullpath.empty() && fullpath.back() != '/' && fullpath.back() != '\\') fullpath += '/';
		fullpath += log_file;
		m_sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_mt>(fullpath, 23, 59));
	}

	// set global default level by storing parsed level for later use in get_logger
	default_level_ = parse_level(level_str);

	// Optionally set the global level too
	spdlog::set_level(default_level_);

	return 0;
}

std::shared_ptr<spdlog::logger> LoggerFactory::get_logger(const std::string &name)
{
	auto combined_logger = spdlog::get(name);
	if (!combined_logger)
	{
		combined_logger = std::make_shared<spdlog::logger>(name, std::begin(m_sinks), std::end(m_sinks));
		spdlog::register_logger(combined_logger);
		// respect global/default level already set in init()
		combined_logger->flush_on(default_level_);
	}

	return combined_logger;
}
