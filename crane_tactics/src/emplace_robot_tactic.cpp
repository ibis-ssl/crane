// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/emplace_robot_tactic.hpp>

#include <cstdlib>
#include <filesystem>

namespace crane
{
void EmplaceRobotTactic::onRobotsChanged() { m_skill_map.clear(); }

auto EmplaceRobotTactic::findAvailableSoundFile() -> std::string
{
  // 複数の候補パスを試す（優先順位順）
  std::vector<std::string> candidates = {
    "/usr/share/sounds/sound-icons/trumpet-12.wav",
    "/usr/share/sounds/freedesktop/stereo/dialog-warning.oga",
    "/usr/share/sounds/freedesktop/stereo/bell.oga",
    "/usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga",
    "/usr/share/sounds/ubuntu/stereo/dialog-warning.ogg",
    "/usr/share/sounds/ubuntu/stereo/bell.ogg",
    "/usr/share/sounds/sound-icons/canary-long.wav"};

  for (const auto & path : candidates) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }

  return "";  // 何も見つからなかった
}

std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
EmplaceRobotTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  // ロボット退場中の警告ビープ
  if (!robots.empty()) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_beep_time_ >= BEEP_INTERVAL) {
      // 初回または音声ファイルパスが未設定の場合、利用可能なファイルを探す
      if (beep_sound_path_.empty()) {
        beep_sound_path_ = findAvailableSoundFile();
      }

      // 警告音をバックグラウンドで再生
      if (!beep_sound_path_.empty()) {
        std::string command = "paplay " + beep_sound_path_ + " &";
        std::system(command.c_str());
      }
      last_beep_time_ = now;
    }
  }

  // GlobalRobotAllocator対応: robotsが変更されたらスキルマップを再生成
  // ロボットの優先順位はgetRobotSuitabilityFunc()で決定される（モーター温度が高いほど優先）
  if (m_skill_map.size() != robots.size()) {
    m_skill_map.clear();

    int select_num = robots.size();
    int selected_robots_index = 0;
    for (const auto & robot_id : robots) {
      m_skill_map.try_emplace(
        robot_id.id, std::make_shared<skills::EmplaceRobot>(robot_id.id, world_model));

      m_skill_map[robot_id.id]->setParameter("total_robot_number", select_num);
      m_skill_map[robot_id.id]->setParameter("current_robot_index", selected_robots_index);

      // どちら側に退場するか
      m_skill_map[robot_id.id]->setParameter(
        "emplace_line_positive", world_model->isEmplacePositiveSide());
      ++selected_robots_index;
    }
  }

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  for (auto & [id, skill] : m_skill_map) {
    skill->run();
    robot_commands.emplace_back(skill->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

bool EmplaceRobotTactic::isHardConstraint() const
{
  // 現在出ているロボット台数が出場可能台数を超えている場合はハード制約
  auto available_robots = world_model->ours().getAvailableRobots();
  auto max_allowed = world_model->getOurMaxAllowedBots();
  return available_robots.size() > max_allowed;
}

auto EmplaceRobotTactic::getRobotSuitabilityFunc() const
  -> std::function<double(const std::shared_ptr<RobotInfo> &)>
{
  auto wm = world_model;
  return [wm](const std::shared_ptr<RobotInfo> & robot) {
    // ゴールキーパーは退場させにくくする
    if (robot->id == wm->getOurGoalieId()) {
      return 10000.0;
    }

    // モーター温度の最大値を計算
    float max_temp = 0.0f;
    if (robot->motor_temperatures.size() >= 4) {
      for (size_t i = 0; i < 4; ++i) {
        max_temp = std::max(max_temp, static_cast<float>(robot->motor_temperatures[i]));
      }
    }

    // 温度が高いほど小さい値を返す（逆数）
    // 温度が0の場合は大きな値を返して退場優先度を下げる
    if (max_temp > 0.0f) {
      return 1000.0 / max_temp;
    }
    return 1000.0;  // 温度情報がない場合は退場させにくい
  };
}

}  // namespace crane
