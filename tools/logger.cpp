#include "logger.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>

namespace
{
constexpr std::size_t kMaxLogFileSize = 10 * 1024 * 1024;
constexpr std::size_t kMaxRotatedLogFiles = 4;
}  // namespace

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{
  std::filesystem::create_directories("logs");
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    "logs/infantry.log", kMaxLogFileSize, kMaxRotatedLogFiles);
  file_sink->set_level(spdlog::level::debug);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
