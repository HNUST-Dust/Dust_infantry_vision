#include "logger.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace
{
constexpr std::size_t kMaxLogFileSize = 10 * 1024 * 1024;
constexpr std::size_t kMaxRotatedLogFiles = 4;
constexpr const char * kLogFilePath = "logs/infantry.log";

// 名字只用于向 spdlog registry 注册：flush_every() 内部是 registry::flush_all()，只遍历
// 已注册的 logger，不注册就是静默空操作；set_error_handler() 同样只作用于已注册的 logger。
// 输出格式由 set_pattern() 固定且不含 %n，所以改名不影响日志文本。
// 不能沿用空名字——registry 构造时已用 "" 注册了默认 logger，重名会抛 already exists。
constexpr const char * kLoggerName = "infantry";

std::atomic<std::size_t> g_sink_error_count{0};

// sink 写失败默认被 SPDLOG_LOGGER_CATCH 吞掉，兜底处理器每秒最多往 stderr 打一行；
// 无终端的 systemd 部署下 stderr 进 journald 基本没人看，这里显式记一次数。
void handle_sink_error(const std::string & msg)
{
  g_sink_error_count.fetch_add(1, std::memory_order_relaxed);
  std::fprintf(stderr, "[logger] sink error: %s\n", msg.c_str());
}

std::shared_ptr<spdlog::logger> make_logger()
{
  std::vector<spdlog::sink_ptr> sinks;

  // 日志目录建不出来（只读 CWD、磁盘满）时退化为仅控制台，绝不抛出去：
  // logger() 会在各处 catch 块里被首次调用，此时抛异常可能直接 terminate。
  try
  {
    const std::filesystem::path log_path(kLogFilePath);
    std::error_code ec;
    std::filesystem::create_directories(log_path.parent_path(), ec);
    if (ec) throw spdlog::spdlog_ex("cannot create log directory: " + ec.message());

    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      kLogFilePath, kMaxLogFileSize, kMaxRotatedLogFiles));
  }
  catch (const std::exception & e)
  {
    std::fprintf(stderr, "[logger] file sink disabled: %s\n", e.what());
  }

  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

  auto logger = std::make_shared<spdlog::logger>(kLoggerName, sinks.begin(), sinks.end());

  logger->set_level(spdlog::level::debug);

  // 沿用原来的外观（"[时间] [级别]" + 消息里的手写 [Gimbal]/[Tracker] 前缀），只在级别后补一个
  // 线程 id：运行时是采集/检测/计划/云台读/相机守护多线程，出问题时要能分辨是哪个线程打的。
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v");

  // 不再每条 info/warn/error 都 flush 两个 sink（165fps 下等于每消息一次系统调用），改为
  // warn 以上立即刷、后台每秒刷一次。原来 flush_on(info) 意味着 debug 行永远不刷，
  // 崩溃时丢掉的恰恰是最想看的那几行。
  logger->flush_on(spdlog::level::warn);

  logger->set_error_handler(handle_sink_error);

  return logger;
}
}  // namespace

namespace tools
{
std::shared_ptr<spdlog::logger> logger()
{
  // C++11 magic static：并发首次调用由编译器保证只构造并注册一次。
  // 原来是 if (!logger_) set_logger(); 作用在 namespace 域的 shared_ptr 上，是数据竞争。
  static const std::shared_ptr<spdlog::logger> instance = [] {
    auto logger = make_logger();
    spdlog::register_logger(logger);
    spdlog::flush_every(std::chrono::seconds(1));
    return logger;
  }();

  return instance;
}

}  // namespace tools
