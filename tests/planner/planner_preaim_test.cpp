#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "target_replay.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/math/trajectory.hpp"

using namespace auto_aim;
using namespace std::chrono_literals;
namespace
{
void require(bool value, const std::string & message)
{
  if (!value)
    throw std::runtime_error(message);
}
const auto at = std::chrono::steady_clock::time_point(100s);
PlanningState feedback()
{
  PlanningState s;
  s.timestamp = at;
  s.valid = s.rates_measured = true;
  return s;
}
TargetPredictionState target(double phase, double omega, double phi = -M_PI / 2,
                             bool moving = false)
{
  TargetPredictionState s;
  s.timestamp = at - 7ms;
  s.name = ArmorName::three;
  s.armor_type = ArmorType::small;
  s.armor_num = 4;
  s.jumped = true;
  s.x << 3.5 * std::cos(phi), moving ? .3 : 0, 3.5 * std::sin(phi), moving ? -.2 : 0, .1,
    moving ? .03 : 0, phi + phase, omega, .22, .11, .12;
  return s;
}

void segments()
{
  Trajectory r = Trajectory::Zero();
  ReferenceTags tags{};
  for (int k = 0; k < 8; ++k) {
    const int segment = k / 2;
    tags[k] = {segment == 2 ? 2 : 1, true,
               segment == 0 || segment == 3 ? AimMode::track : AimMode::preaim};
    r(0, k) = 100 * segment + k * DT;
    r(2, k) = -100 * segment + 2 * k * DT;
  }
  tags[8] = {0, true, AimMode::preaim};  // isolated point
  fill_segment_reference_velocity(r, tags);
  for (int k = 0; k < 8; ++k)
    require(std::abs(r(1, k) - 1) < 1e-9 && std::abs(r(3, k) - 2) < 1e-9,
            "No cross-mode/armor derivative; one-sided endpoints");
  require(r(1, 8) == 0 && r(3, 8) == 0 && r(1, 9) == 0, "Isolated/invalid rates zero");
}

// Independent geometric derivative: move the center and rotate the armor, then
// measure the change of their relative angle. Do not repeat the production formula.
double relative_rate_by_motion(const Eigen::VectorXd & x)
{
  constexpr double step = 1e-5;
  const auto angle = [&](double dt) {
    return x[6] + x[7] * dt - std::atan2(x[2] + x[3] * dt, x[0] + x[1] * dt);
  };
  return tools::limit_rad(angle(step) - angle(-step)) / (2 * step);
}

void geometry(const TargetPredictionState & input, const PlanningTrace & trace)
{
  require(trace.reference_available, "Reference available");
  for (int k = 0; k < HORIZON; ++k) {
    const auto & tag = trace.reference_tags[k];
    if (tag.mode == AimMode::fallback)
      continue;
    Target hit = Target::from_prediction_state(input);
    hit.predict(at + 10ms);
    if (k)
      hit.predict(k * DT);
    hit.predict(tag.fly_time);
    const auto x = hit.ekf_x();
    const auto armors = hit.armor_xyza_list();
    const double phi = std::atan2(x[2], x[0]);
    const double rate = relative_rate_by_motion(x);
    const double direction = rate > 0 ? 1 : -1;
    const double boundary = -direction * 35 / 57.3;
    int in_window = 0;
    for (const auto & armor : armors) {
      const double delta = tools::limit_rad(armor[3] - phi);
      in_window += std::abs(delta) <= 35 / 57.3 && direction * delta < 20 / 57.3;
    }
    if (tag.mode == AimMode::track) {
      require(tag.selection_valid && (tag.reference_xyza - armors[tag.armor_id]).norm() < 1e-10,
              "Track uses actual armor");
      const double delta = tools::limit_rad(armors[tag.armor_id][3] - phi);
      if (std::abs(rate) <= 2 + 1e-6)
        require(std::abs(delta) <= 60 / 57.3 + 1e-8, "Slow relative motion uses lock window");
      else
        require(std::abs(delta) <= 35 / 57.3 + 1e-8 && direction * delta < 20 / 57.3,
                "Tracking window follows relative motion, not body spin");
      continue;
    }
    require(std::abs(rate) > 2 - 1e-6, "No preaim for slow relative motion");
    require(!tag.selection_valid && tag.converged && tag.rate_valid && in_window == 0,
            "Preaim converges in empty window without fire eligibility");
    // Locally sweep each armor in the geometrically measured relative direction.
    double earliest = 1e10;
    int incoming = -1;
    for (int id = 0; id < 4; ++id) {
      double angle = armors[id][3];
      const double entry = phi + boundary;
      while (direction * (entry - angle) < 0)
        angle -= direction * 2 * M_PI;
      const double time = (entry - angle) / rate;
      if (time < earliest) {
        earliest = time;
        incoming = id;
      }
    }
    require(incoming == tag.armor_id, "Earliest incoming armor, not nearest/id+1");
    const double radius = x[8] + ((incoming % 2) ? x[9] : 0);
    Eigen::Vector3d expected{x[0] - radius * std::cos(phi + boundary),
                             x[2] - radius * std::sin(phi + boundary),
                             x[4] + ((incoming % 2) ? x[10] : 0)};
    require((tag.reference_xyza.head<3>() - expected).norm() < 1e-9,
            "Boundary uses incoming radius/height and translated center");
    tools::Trajectory ballistic(trace.bullet_speed, expected.head<2>().norm(), expected.z());
    require(std::abs(ballistic.fly_time - tag.fly_time) < .001,
            "Flight iteration uses boundary geometry without additional waiting");
  }
}
void moving_selection(Planner & planner)
{
  // Construct a known hit state, then rewind center and physical orientation to
  // the observation timestamp. These cases fail if either selection entry point
  // continues to use body spin, or if w is replaced in physical propagation.
  struct Scenario
  {
    double omega, los_rate, phase;
    AimMode mode;
    double radial_velocity = 0;
  };
  const std::array<Scenario, 8> scenarios{{
    {6, 5, .7, AimMode::track},         // Cancellation: lock despite large body spin.
    {3, 6, .45, AimMode::track},        // Reversed window admits this plate.
    {3, 6, M_PI / 4, AimMode::preaim},  // Reversed incoming boundary/identity.
    {0, 6, M_PI / 4, AimMode::preaim},  // Translation alone changes facing.
    {6, 0, .2, AimMode::track},         // Stationary center.
    {6, 0, .2, AimMode::track, -2},     // Radial motion away: unchanged direction.
    {6, 0, .2, AimMode::track, 2},      // Radial motion toward us.
    {6, -2, .2, AimMode::track},        // Opposite translation adds relative speed.
  }};
  for (double sign : {-1., 1.}) {
    for (const auto & scenario : scenarios) {
      auto input = target(sign * scenario.phase, sign * scenario.omega);
      const double phi = -M_PI / 2;
      Eigen::VectorXd hit = input.x;
      hit[0] = 0;
      hit[2] = -2;
      hit[1] = sign * scenario.los_rate * 2;
      hit[3] = scenario.radial_velocity;
      const double angle =
        phi + sign * (scenario.mode == AimMode::preaim ? 35 / 57.3 : scenario.phase);
      const Eigen::Vector3d expected{-hit[8] * std::cos(angle), -2 - hit[8] * std::sin(angle),
                                     hit[4]};
      const tools::Trajectory shot(23, expected.head<2>().norm(), expected.z());
      const double elapsed = .017 + shot.fly_time;
      input.x = hit;
      input.x[0] -= hit[1] * elapsed;
      input.x[2] -= hit[3] * elapsed;
      input.x[6] -= hit[7] * elapsed;
      planner.restore_history({});
      PlanningTrace trace;
      const auto plan =
        planner.plan_at(Target::from_prediction_state(input), at, 23, feedback(), &trace);
      require(trace.reference_available, "Moving reference generated");
      const auto & first = trace.reference_tags[0];
      require(first.converged && first.mode == scenario.mode && first.armor_id == 0,
              "Cancellation/reversal selects expected mode and incoming plate");
      require((first.reference_xyza.head<3>() - expected).norm() < .02,
              "Hit point follows physical spin and translated center");
      if (scenario.mode == AimMode::preaim)
        require(!plan.fire, "Reversed preaim cannot fire");
      geometry(input, trace);
    }
  }

  // Recompute relative motion at every horizon column: this transverse passage
  // crosses zero and both branch boundaries although vx and body w are constant.
  for (double sign : {-1., 1.}) {
    auto input = target(.6, sign * 3);
    input.x[0] = 0;
    input.x[2] = -1;
    input.x[1] = sign * 8;
    planner.restore_history({});
    PlanningTrace trace;
    planner.plan_at(Target::from_prediction_state(input), at, 23, feedback(), &trace);
    geometry(input, trace);
    bool negative_fast = false, slow = false, positive_fast = false;
    for (int k = 0; k < HORIZON; ++k) {
      const auto & tag = trace.reference_tags[k];
      if (!tag.converged)
        continue;
      auto hit = Target::from_prediction_state(input);
      hit.predict(at + 10ms);
      hit.predict(k * DT + tag.fly_time);
      const double rate = sign * relative_rate_by_motion(hit.ekf_x());
      negative_fast |= rate < -2;
      slow |= std::abs(rate) < 2;
      positive_fast |= rate > 2;
    }
    require(negative_fast && slow && positive_fast,
            "Future states cover reversed, slow and forward relative motion");
  }

  auto invalid = target(.5, 6);
  invalid.x[0] = invalid.x[2] = 0;
  PlanningTrace trace;
  const auto plan =
    planner.plan_at(Target::from_prediction_state(invalid), at, 23, feedback(), &trace);
  require(!plan.control && !plan.fire && !trace.reference_available,
          "Undefined center line of sight rejects the candidate");
  std::cout << "PASS relative selection: cancellation, reversal, translation, horizon crossings, "
               "degenerate center\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc >= 2, "Usage: planner_preaim_test configs/standard3.yaml [capture.jsonl]");
    segments();
    char pattern[] = "/tmp/dust-preaim-test-XXXXXX";
    const char * created = mkdtemp(pattern);
    require(created, "Temporary directory");
    const std::filesystem::path directory(created);
    struct Cleanup
    {
      std::filesystem::path path;
      ~Cleanup()
      {
        std::filesystem::remove_all(path);
      }
    } cleanup{directory};
    auto yaml = YAML::LoadFile(argv[1]);
    // Geometry is fixed at the proposed window; all MPC costs/limits come from supplied config.
    yaml["comming_angle"] = 35;
    yaml["leaving_angle"] = 20;
    yaml["high_speed_delay_time"] = yaml["low_speed_delay_time"] = .01;
    yaml["auto_fire"] = true;
    yaml["first_tolerance"] = yaml["second_tolerance"] = 180;
    yaml["fire_thresh"] = 10;
    const auto off_path = directory / "off.yaml", on_path = directory / "on.yaml";
    yaml["gap_preaim_enabled"] = false;
    {
      std::ofstream out(off_path);
      out << yaml;
    }
    yaml["gap_preaim_enabled"] = true;
    {
      std::ofstream out(on_path);
      out << yaml;
    }
    Planner off(off_path.string()), on(on_path.string());
    moving_selection(on);
    int cases = 0, preaim = 0, off_fail = 0, on_fail = 0, future_gap_gate = 0, tracks_fire = 0;
    int ids_mask = 0;
    double off_ms = 0, on_ms = 0;
    std::ofstream capture;
    if (argc > 2)
      capture.open(argv[2]);
    for (double omega : {-12., -6., 6., 12.}) {
      for (double phi : {-M_PI + .01, -M_PI / 2, M_PI - .01}) {
        for (int i = 0; i < 72; ++i) {
          const auto input = target(i * 2 * M_PI / 72, omega, phi, i % 2);
          const auto t = Target::from_prediction_state(input);
          PlanningTrace a, b;
          auto measured = feedback();
          measured.yaw = tools::limit_rad(phi + M_PI / 2);
          measured.R_gimbal2world =
            Eigen::AngleAxisd(measured.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          const PlannerHistory history{-1, true, measured.yaw, at - 2ms};
          off.restore_history(history);
          on.restore_history(history);
          auto begin = std::chrono::steady_clock::now();
          const auto pa = off.plan_at(t, at, 23, measured, &a);
          off_ms +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count();
          begin = std::chrono::steady_clock::now();
          const auto pb = on.plan_at(t, at, 23, measured, &b);
          on_ms +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count();
          ++cases;
          off_fail += !pa.control;
          on_fail += !pb.control;
          geometry(input, b);
          for (const auto & tag : b.reference_tags)
            if (tag.mode == AimMode::preaim) {
              ++preaim;
              ids_mask |= 1 << tag.armor_id;
            }
          bool command_valid = true;
          for (int k = 0; k <= COMMAND_STEP; ++k)
            command_valid &= b.reference_tags[k].selection_valid && b.reference_tags[k].converged;
          require(command_valid || !pb.fire, "Any invalid/preaim command column prohibits fire");
          tracks_fire += pb.fire;
          future_gap_gate += b.reference_tags[0].mode == AimMode::track &&
                             b.reference_tags[1].mode == AimMode::preaim && !pb.fire;
          if (pb.control)
            require(std::abs(pb.yaw_acc) <= b.yaw_config.max_acc + .002 &&
                      std::abs(pb.pitch_acc) <= b.pitch_config.max_acc + .002,
                    "Both acceleration limits");
          if (i == 0) {
            const auto j = target_replay::serialize({pb, b});
            target_replay::reproduce(nlohmann::json::parse(j.dump()));
            const auto ab_off = target_replay::run(j, false);
            require((ab_off.trace.reference - a.reference).norm() == 0 &&
                      ab_off.plan.fire == pa.fire,
                    "Target A/B reproduces identical disabled input/history");
          }
          if (capture.is_open()) {
            auto j = target_replay::serialize({pb, b});
            j["plan_id"] = cases;
            capture << j.dump() << '\n';
          }
        }
      }
    }
    // Low speed (including exact threshold), unjumped, non-four-plate, and outpost exclusions.
    for (int scenario = 0; scenario < 6; ++scenario) {
      auto input = target(.8, 12);
      if (scenario == 0)
        input.x[7] = 2;
      if (scenario == 1)
        input.x[7] = -2;
      if (scenario == 2)
        input.jumped = false;
      if (scenario == 3) {
        input.name = ArmorName::outpost;
        input.armor_num = 3;
      }
      if (scenario == 4)
        input.armor_num = 2;
      if (scenario == 5)
        input.name = ArmorName::outpost;
      PlanningTrace trace;
      on.plan_at(Target::from_prediction_state(input), at, 23, feedback(), &trace);
      for (const auto & tag : trace.reference_tags)
        require(tag.mode != AimMode::preaim, "Excluded targets keep original selection");
    }
    // At entry, preaim geometry meets the actual incoming plate (all radius/height IDs).
    for (double omega : {-12., 12.}) {
      for (int id = 0; id < 4; ++id) {
        const double phi = -M_PI / 2, boundary = (omega > 0 ? -1 : 1) * 35 / 57.3;
        const double radius = .22 + (id % 2 ? .11 : 0), z = .1 + (id % 2 ? .12 : 0);
        const Eigen::Vector3d edge{3.5 * std::cos(phi) - radius * std::cos(phi + boundary),
                                   3.5 * std::sin(phi) - radius * std::sin(phi + boundary), z};
        const tools::Trajectory shot(23, edge.head<2>().norm(), z);
        const double phase = boundary - id * M_PI / 2 - omega * (.017 + shot.fly_time);
        const auto sample = [&](double offset) {
          const auto input = target(phase + (omega > 0 ? 1 : -1) * offset, omega);
          PlanningTrace trace;
          on.plan_at(Target::from_prediction_state(input), at, 23, feedback(), &trace);
          return trace.reference_tags[0];
        };
        double left = -.02, right = .02;
        auto pre = sample(left), track = sample(right);
        require(pre.mode == AimMode::preaim && track.mode == AimMode::track && pre.armor_id == id &&
                  track.armor_id == id,
                "Bracket actual incoming transition");
        for (int step = 0; step < 18; ++step) {
          const double middle = (left + right) * .5;
          const auto tag = sample(middle);
          require(tag.armor_id == id, "Incoming identity stable at boundary");
          if (tag.mode == AimMode::preaim) {
            left = middle;
            pre = tag;
          } else if (tag.mode == AimMode::track) {
            right = middle;
            track = tag;
          } else
            break;  // Unconverged boundary candidates are forbidden to fire.
        }
        require((pre.reference_xyza.head<3>() - edge).norm() < 1e-9 &&
                  (pre.reference_xyza.head<3>() - track.reference_xyza.head<3>()).norm() < .004,
                "Incoming position joins boundary within 1ms flight iteration accuracy");
      }
    }
    auto old = nlohmann::json::object();
    bool rejected = false;
    try {
      target_replay::run(old);
    } catch (const std::exception &) {
      rejected = true;
    }
    require(rejected, "Old traces without target state rejected");
    require(preaim > 0 && ids_mask == 15 && future_gap_gate > 0 && tracks_fire > 0,
            "Exercise all incoming IDs, future-column fire gate, and valid tracking fire");
    std::cout << "PASS preaim cases=" << cases << " preaim_columns=" << preaim
              << " future_gap_gate=" << future_gap_gate << " tracking_fire=" << tracks_fire
              << " failures_off/on=" << off_fail << '/' << on_fail
              << " mean_ms_off/on=" << off_ms / cases << '/' << on_ms / cases << '\n';
    require(on_fail <= off_fail, "No additional solve failures in synthetic A/B corpus");
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL preaim: " << e.what() << '\n';
    return 1;
  }
}
