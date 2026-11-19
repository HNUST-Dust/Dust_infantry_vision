#pragma once

#include <chrono>
#include <cstdint>

namespace auto_aim::runtime
{
enum SendFireBlock : uint32_t
{
  send_no_control = 1U << 0,
  send_planner_no_fire = 1U << 1,
  send_angle_outside = 1U << 2,
  send_no_plan = 1U << 3,
  send_plan_expired = 1U << 4,
  send_feedback_stale = 1U << 5,
  send_no_observation = 1U << 6,
  send_observation_control_stale = 1U << 7,
  send_observation_fire_stale = 1U << 8
};

// Copied as one unit from the send thread; errors remain available even when a gate clears control.
struct SendTelemetrySnapshot
{
  uint64_t sequence = 0, plan_id = 0;
  double t = 0.0;
  int mode = 0;
  uint32_t fire_block_mask = 0;
  bool target_valid = false;
  double target_yaw = 0.0, target_pitch = 0.0, fire_tolerance = 0.0;
  double mode2_time_s = 0.0, total_time_s = 0.0;
};

// Send-thread only. The previous mode owns the interval up to the next send invocation.
class SendTelemetryAccumulator
{
public:
  using Clock = std::chrono::steady_clock;
  explicit SendTelemetryAccumulator(Clock::time_point origin) : origin_(origin) {}

  SendTelemetrySnapshot observe(Clock::time_point sent_at, SendTelemetrySnapshot sample)
  {
    if (sequence_ != 0) {
      const double elapsed = std::chrono::duration<double>(sent_at - last_sent_at_).count();
      total_time_s_ += elapsed;
      if (last_mode_ == 2)
        mode2_time_s_ += elapsed;
    }
    sample.sequence = ++sequence_;
    sample.t = std::chrono::duration<double>(sent_at - origin_).count();
    sample.total_time_s = total_time_s_;
    sample.mode2_time_s = mode2_time_s_;
    last_sent_at_ = sent_at;
    last_mode_ = sample.mode;
    return sample;
  }

private:
  Clock::time_point origin_, last_sent_at_{};
  uint64_t sequence_ = 0;
  int last_mode_ = 0;
  double mode2_time_s_ = 0.0, total_time_s_ = 0.0;
};
}  // namespace auto_aim::runtime
