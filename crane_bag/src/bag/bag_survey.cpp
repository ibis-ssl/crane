// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_survey.hpp"

#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
#include <vector>

namespace crane::bag
{

namespace
{

std::string fmt_t(double t_sec)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "t=%.2f", t_sec);
  return buf;
}

std::string fmt_pos(double x, double y)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "(%.3f,%.3f)", x, y);
  return buf;
}

std::string fmt_factors(const std::vector<NamedString> & factors)
{
  if (factors.empty()) return "(empty)";
  std::string s;
  for (size_t i = 0; i < factors.size(); ++i) {
    if (i > 0) s += "; ";
    s += factors[i].name + "=" + factors[i].value;
  }
  return s;
}

std::string section_play_situations(const BagData & data)
{
  std::ostringstream oss;
  oss << "=== PLAY SITUATIONS ===\n";
  for (const auto & tm : data.play_situations) {
    double t = tm.t(data.info.start_time_ns);
    oss << "  " << fmt_t(t) << ": " << tm.msg.command_name << ", reason=" << tm.msg.reason_text
        << "\n";
  }
  return oss.str();
}

std::string section_role_assignments(const BagData & data)
{
  std::ostringstream oss;
  oss << "=== ROLE ASSIGNMENTS (last) ===\n";
  if (!data.robot_select_results.empty()) {
    const auto & tm = data.robot_select_results.back();
    double t = tm.t(data.info.start_time_ns);
    oss << "  " << fmt_t(t) << ":\n";
    for (const auto & r : tm.msg.results) {
      oss << "    " << r.name << ": selected=[";
      for (size_t ri = 0; ri < r.selected_robots.size(); ++ri) {
        if (ri > 0) oss << ", ";
        oss << static_cast<int>(r.selected_robots[ri]);
      }
      oss << "]\n";
    }
  }
  return oss.str();
}

std::string section_world_model(const BagData & data, double interval)
{
  std::ostringstream oss;
  char buf[128];
  const char * team_color =
    (!data.world_models.empty() && data.world_models.front().msg.is_yellow) ? "YELLOW" : "BLUE";
  std::snprintf(
    buf, sizeof(buf), "=== WORLD MODEL (every %.0fs) [OUR_TEAM=%s] ===\n", interval, team_color);
  oss << buf;

  for (const auto * tm : BagData::sample(data.world_models, interval)) {
    double t = tm->t(data.info.start_time_ns);
    const auto & ball = tm->msg.ball_info;
    double bx = ball.position.x, by = ball.position.y;
    double bvx = ball.velocity.x, bvy = ball.velocity.y;
    double bspeed = std::sqrt(bvx * bvx + bvy * bvy);

    std::snprintf(
      buf, sizeof(buf), "  t=%.2f: ball=%s speed=%.2fm/s\n", t, fmt_pos(bx, by).c_str(), bspeed);
    oss << buf;

    for (const auto & r : tm->msg.robot_info_ours) {
      double dist =
        std::sqrt((r.pose.x - bx) * (r.pose.x - bx) + (r.pose.y - by) * (r.pose.y - by));
      std::snprintf(
        buf, sizeof(buf), "    our[%d]: %s dist_ball=%.2fm det=%s\n", static_cast<int>(r.id),
        fmt_pos(r.pose.x, r.pose.y).c_str(), dist, r.available_vision ? "True" : "False");
      oss << buf;
    }
  }
  return oss.str();
}

std::string section_planning_factors(const BagData & data)
{
  std::ostringstream oss;
  oss << "=== CONTROL_TARGETS: UNIQUE PLANNING_FACTORS (after first play_situation) ===\n";

  double start_t =
    data.play_situations.empty() ? 0.0 : data.play_situations.front().t(data.info.start_time_ns);

  // "robot_id:name1=val1;name2=val2" 形式の文字列キーで O(1) 重複検出
  std::unordered_set<std::string> seen;

  for (const auto & tm : data.control_targets) {
    double t = tm.t(data.info.start_time_ns);
    if (t < start_t) continue;
    for (const auto & rc : tm.msg.robot_commands) {
      std::string key = std::to_string(rc.robot_id) + ":" + fmt_factors(rc.planning_factors);
      if (seen.insert(key).second) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "  t=%.2f robot=%d: [", t, static_cast<int>(rc.robot_id));
        oss << buf << fmt_factors(rc.planning_factors) << "]\n";
      }
    }
  }
  return oss.str();
}

std::string section_velocity_status(const BagData & data, double interval = 10.0)
{
  std::ostringstream oss;
  char buf[256];
  std::snprintf(
    buf, sizeof(buf), "=== ROBOT VELOCITY STATUS (every %.0fs, robot_commands) ===\n", interval);
  oss << buf;

  for (const auto * tm : BagData::sample(data.robot_commands, interval)) {
    double t = tm->t(data.info.start_time_ns);
    std::vector<int> zero_robots;
    std::vector<std::pair<int, double>> moving_robots;

    for (const auto & rc : tm->msg.robot_commands) {
      double vr = 0.0;
      if (!rc.polar_velocity_target_mode.empty()) {
        vr = rc.polar_velocity_target_mode[0].target_velocity_r;
      }
      if (std::abs(vr) < 0.01 && !rc.stop_flag) {
        zero_robots.push_back(static_cast<int>(rc.robot_id));
      } else if (std::abs(vr) >= 0.01) {
        moving_robots.emplace_back(static_cast<int>(rc.robot_id), vr);
      }
    }

    std::string zero_str = "[";
    for (size_t i = 0; i < zero_robots.size(); ++i) {
      if (i > 0) zero_str += ", ";
      zero_str += std::to_string(zero_robots[i]);
    }
    zero_str += "]";

    std::string moving_str = "[";
    for (size_t i = 0; i < moving_robots.size(); ++i) {
      if (i > 0) moving_str += ", ";
      char tmp[32];
      std::snprintf(
        tmp, sizeof(tmp), "(%d, %.2f)", moving_robots[i].first, moving_robots[i].second);
      moving_str += tmp;
    }
    moving_str += "]";

    std::snprintf(
      buf, sizeof(buf), "  t=%.2f: zero_v=%s, moving=%s\n", t, zero_str.c_str(),
      moving_str.c_str());
    oss << buf;
  }
  return oss.str();
}

std::string section_game_analysis(const BagData & data, double interval)
{
  std::ostringstream oss;
  char buf[128];
  std::snprintf(buf, sizeof(buf), "=== GAME ANALYSIS (every %.0fs) ===\n", interval);
  oss << buf;

  for (const auto * tm : BagData::sample(data.game_analyses, interval)) {
    double t = tm->t(data.info.start_time_ns);
    std::snprintf(
      buf, sizeof(buf), "  t=%.2f: attacker=%d (score=%.2f), pass_target=%d\n", t,
      tm->msg.recommended_attacker_id, tm->msg.attacker_suitability_score, tm->msg.pass_target_id);
    oss << buf;
  }
  return oss.str();
}

std::string section_rosout(const BagData & data)
{
  std::ostringstream oss;
  oss << "=== ROSOUT (WARN/ERROR, deduplicated) ===\n";

  std::set<std::pair<std::string, std::string>> seen;
  constexpr uint8_t WARN_LEVEL = 30;  // rcl_interfaces::msg::Log: DEBUG=10, INFO=20, WARN=30
  for (const auto & tm : data.rosout) {
    if (tm.msg.level < WARN_LEVEL) continue;
    std::string key_msg = tm.msg.msg.substr(0, 80);
    auto key = std::make_pair(tm.msg.name, key_msg);
    if (seen.insert(key).second) {
      double t = tm.t(data.info.start_time_ns);
      std::string truncated = tm.msg.msg.size() > 200 ? tm.msg.msg.substr(0, 200) : tm.msg.msg;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "  t=%.2f [", t);
      oss << buf << tm.msg.name << "] L" << static_cast<int>(tm.msg.level) << ": " << truncated
          << "\n";
    }
  }
  return oss.str();
}

}  // namespace

std::string run_survey(const BagData & data, double sample_interval)
{
  std::string result;
  result += section_play_situations(data);
  result += "\n";
  result += section_role_assignments(data);
  result += "\n";
  result += section_world_model(data, sample_interval);
  result += "\n";
  result += section_planning_factors(data);
  result += "\n";
  result += section_velocity_status(data);
  result += "\n";
  result += section_game_analysis(data, sample_interval);
  result += "\n";
  result += section_rosout(data);
  return result;
}

}  // namespace crane::bag
