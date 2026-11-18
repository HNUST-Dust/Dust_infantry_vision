#include "telemetry.hpp"

#include "io/gimbal/gimbal.hpp"
#include "src/planner_diagnostics.hpp"
#include "tasks/auto_aim/detection/mt_detector.hpp"
#include "tools/math/math_tools.hpp"

namespace auto_aim::runtime
{
namespace
{
void add_latency_metrics(nlohmann::json & data,
                         const std::optional<tools::LatencySummary> & latency_summary)
{
  if (!latency_summary)
    return;

  const auto & summary = *latency_summary;
  data["vision_latency_ms"] = summary.latest_ms;
  data["vision_latency_p50_ms"] = summary.p50_ms;
  data["vision_latency_p95_ms"] = summary.p95_ms;
  data["vision_latency_p99_ms"] = summary.p99_ms;
}
}  // namespace

nlohmann::json telemetry_json(const TelemetrySnapshot & snapshot)
{
  const auto & plan = snapshot.plan;
  const auto & feedback = snapshot.feedback;
  const auto & gs = feedback.state;
  nlohmann::json data;
  data["t"] = tools::delta_time(snapshot.sampled_at, snapshot.origin);
  data["tracker_state"] = snapshot.tracker_state;
  data["has_target"] = snapshot.target.has_value() ? 1 : 0;
  data["gimbal_yaw"] = gs.yaw;
  data["gimbal_yaw_vel"] = gs.yaw_vel;
  data["gimbal_pitch"] = gs.pitch;
  data["gimbal_pitch_vel"] = gs.pitch_vel;
  data["target_yaw"] = plan.target_yaw;
  data["target_pitch"] = plan.target_pitch;
  data["plan_control"] = plan.control;
  data["plan_yaw"] = plan.yaw;
  data["plan_yaw_vel"] = plan.yaw_vel;
  data["plan_yaw_acc"] = plan.yaw_acc;
  data["plan_pitch"] = plan.pitch;
  data["plan_pitch_vel"] = plan.pitch_vel;
  data["plan_pitch_acc"] = plan.pitch_acc;
  data["plan_time_ms"] = snapshot.planning_ms;
  data["plan_status"] = to_string(plan.status);
  data["plan_status_code"] = static_cast<int>(plan.status);
  data["aim_mode"] = to_string(plan.aim_debug.mode);
  data["aim_preaim"] = plan.aim_debug.mode == AimMode::preaim;
  data["aim_selection_valid"] = plan.aim_debug.selection_valid;
  data["aim_ballistic_converged"] = plan.aim_debug.ballistic_converged;
  data["aim_display_valid"] = plan.aim_visible_at(snapshot.sampled_at);
  data["aim_plan_sequence"] = snapshot.plan_sequence;
  data["aim_age_ms"] = 1e3 * tools::delta_time(snapshot.sampled_at, plan.planned_at);
  data["mpc_yaw"] = mpc_diagnostics_json(plan.yaw_mpc);
  data["mpc_pitch"] = mpc_diagnostics_json(plan.pitch_mpc);
  const auto & reference = snapshot.plan.control_reference;
  data["yaw_mode"] = to_string(reference.yaw_mode);
  data["pitch_mode"] = to_string(reference.pitch_mode);
  data["pitch_height_group"] = reference.height_group;
  data["yaw_transitioning"] = reference.yaw_transitioning;
  data["pitch_transitioning"] = reference.pitch_transitioning;
  data["fire_reason"] = snapshot.plan.fire_context
                          ? nlohmann::json(to_string(snapshot.plan.fire_result.block))
                          : nlohmann::json(nullptr);
  data["send_status_code"] = snapshot.send_status;
  static constexpr const char * send_names[] = {
    "ready",          "no_control",     "no_plan",          "plan_expired",
    "feedback_stale", "no_observation", "observation_stale"};
  data["send_status"] = send_names[snapshot.send_status];
  const auto & send = snapshot.send;
  data["send_valid"] = send.sequence != 0;
  data["send_mode"] = send.mode;
  data["send_t"] = send.t;
  data["send_sequence"] = send.sequence;
  data["send_plan_id"] = send.plan_id;
  data["send_target_valid"] = send.target_valid;
  data["send_target_yaw"] =
    send.target_valid ? nlohmann::json(send.target_yaw) : nlohmann::json(nullptr);
  data["send_target_pitch"] =
    send.target_valid ? nlohmann::json(send.target_pitch) : nlohmann::json(nullptr);
  data["send_fire_tolerance"] =
    send.target_valid ? nlohmann::json(send.fire_tolerance) : nlohmann::json(nullptr);
  data["send_fire_block_mask"] = send.fire_block_mask;
  data["send_block_no_control"] = (send.fire_block_mask & send_no_control) != 0;
  data["send_block_planner_no_fire"] = (send.fire_block_mask & send_planner_no_fire) != 0;
  data["send_block_angle_outside"] = (send.fire_block_mask & send_angle_outside) != 0;
  data["send_block_no_plan"] = (send.fire_block_mask & send_no_plan) != 0;
  data["send_block_plan_expired"] = (send.fire_block_mask & send_plan_expired) != 0;
  data["send_block_feedback_stale"] = (send.fire_block_mask & send_feedback_stale) != 0;
  data["send_block_no_observation"] = (send.fire_block_mask & send_no_observation) != 0;
  data["send_block_observation_control_stale"] =
    (send.fire_block_mask & send_observation_control_stale) != 0;
  data["send_block_observation_fire_stale"] =
    (send.fire_block_mask & send_observation_fire_stale) != 0;
  data["send_yaw_mode"] = to_string(send.yaw_mode);
  data["send_pitch_mode"] = to_string(send.pitch_mode);
  data["send_height_group"] = send.height_group;
  data["send_yaw_transitioning"] = send.yaw_transitioning;
  data["send_pitch_transitioning"] = send.pitch_transitioning;
  data["send_block_real_plate_rejected"] = (send.fire_block_mask & send_real_plate_rejected) != 0;
  data["send_fire_reason"] =
    send.fire_result ? nlohmann::json(to_string(send.fire_result->block)) : nlohmann::json(nullptr);
  data["send_fire_armor_id"] = send.fire_result && send.fire_result->armor_id >= 0
                                 ? nlohmann::json(send.fire_result->armor_id)
                                 : nlohmann::json(nullptr);
  data["send_fire_yaw_error"] = send.fire_result && send.fire_result->error_valid
                                  ? nlohmann::json(send.fire_result->yaw_error)
                                  : nlohmann::json(nullptr);
  data["send_fire_pitch_error"] = send.fire_result && send.fire_result->error_valid
                                    ? nlohmann::json(send.fire_result->pitch_error)
                                    : nlohmann::json(nullptr);
  data["send_mode2_time_s"] = send.mode2_time_s;
  data["send_total_time_s"] = send.total_time_s;
  data["planner_diagnostic_dropped"] = snapshot.trace.dropped;
  data["planner_trace_written"] = snapshot.trace.written;
  data["planner_trace_enabled"] = snapshot.trace.enabled;
  data["feedback_sequence"] = feedback.sequence;
  data["feedback_valid"] = feedback.valid;
  data["feedback_rates_measured"] = feedback.rates_measured;
  data["mpc_feedback_age_ms"] = plan.feedback_age_ms;
  data["mpc_state_valid"] = plan.initial_state.valid;
  data["mpc_initial_yaw"] = plan.initial_state.yaw;
  data["mpc_initial_yaw_vel"] = plan.initial_state.yaw_vel;
  data["mpc_initial_pitch"] = plan.initial_state.pitch;
  data["mpc_initial_pitch_vel"] = plan.initial_state.pitch_vel;
  data["plan_absolute_yaw"] = plan.absolute_yaw;
  data["plan_absolute_pitch"] = plan.absolute_pitch;
  data["mpc_yaw_solved"] = plan.yaw_mpc.solved;
  data["mpc_pitch_solved"] = plan.pitch_mpc.solved;
  data["mpc_yaw_iterations"] = plan.yaw_mpc.iterations;
  data["mpc_pitch_iterations"] = plan.pitch_mpc.iterations;
  data["mpc_yaw_primal_residual"] = plan.yaw_mpc.primal_residual;
  data["mpc_pitch_primal_residual"] = plan.pitch_mpc.primal_residual;
  data["mpc_yaw_dual_residual"] = plan.yaw_mpc.dual_residual;
  data["mpc_pitch_dual_residual"] = plan.pitch_mpc.dual_residual;
  data["mpc_yaw_input_violation"] = plan.yaw_mpc.input_violation;
  data["mpc_pitch_input_violation"] = plan.pitch_mpc.input_violation;
  data["capture_time_ms"] = snapshot.timing.capture_ms;
  data["detection_submit_ms"] = snapshot.timing.submit_ms;
  data["inference_time_ms"] = snapshot.timing.inference_ms;
  data["inference_wait_ms"] = snapshot.timing.inference_wait_ms;
  data["postprocess_time_ms"] = snapshot.timing.postprocess_ms;
  data["result_delivery_ms"] = snapshot.timing.delivery_ms;
  data["dropped_overwritten_frames"] = snapshot.detector_drops.overwritten;
  data["dropped_result_capacity_frames"] = snapshot.detector_drops.result_capacity;
  data["skipped_empty_frames"] = snapshot.detector_drops.skipped_empty;
  data["skipped_busy_frames"] = snapshot.detector_drops.skipped_busy;
  data["skipped_error_frames"] = snapshot.detector_drops.skipped_error;
  data["fire"] = plan.fire ? 1 : 0;
  data["fired"] = snapshot.fired ? 1 : 0;
  if (snapshot.target) {
    const auto & target = *snapshot.target;
    data["target_z"] = target.z;
    data["target_vz"] = target.vz;
    data["w"] = target.spin_rate;
    data["angle"] = target.angle;
  } else {
    data["w"] = 0.0;
  }
  data["mode"] = plan.mode();
  add_latency_metrics(data, snapshot.latency);
  return data;
}
}  // namespace auto_aim::runtime
