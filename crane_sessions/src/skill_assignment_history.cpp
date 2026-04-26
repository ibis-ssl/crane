// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <cinttypes>
#include <crane_sessions/skill_assignment_history.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
SkillAssignmentHistory::SkillAssignmentHistory(const std::filesystem::path & file_path)
: file_path_(file_path)
{
  load();
}

SkillAssignmentHistory::Entry SkillAssignmentHistory::get(std::uint8_t robot_id) const
{
  auto it = entries_.find(robot_id);
  if (it == entries_.end()) {
    return Entry{};
  }
  return it->second;
}

bool SkillAssignmentHistory::recordSuccess(std::uint8_t robot_id)
{
  const auto previous_next_seq = next_seq_;
  auto & e = entries_[robot_id];
  const auto previous_entry = e;
  e.success += 1;
  e.last_seq = next_seq_++;
  if (save()) {
    return true;
  }
  e = previous_entry;
  next_seq_ = previous_next_seq;
  return false;
}

bool SkillAssignmentHistory::recordFailure(std::uint8_t robot_id)
{
  const auto previous_next_seq = next_seq_;
  auto & e = entries_[robot_id];
  const auto previous_entry = e;
  e.failure += 1;
  e.last_seq = next_seq_++;
  if (save()) {
    return true;
  }
  e = previous_entry;
  next_seq_ = previous_next_seq;
  return false;
}

void SkillAssignmentHistory::load()
{
  if (!std::filesystem::exists(file_path_)) {
    RCLCPP_INFO(
      rclcpp::get_logger("SkillAssignmentHistory"), "履歴ファイルが存在しないため新規開始: %s",
      file_path_.c_str());
    return;
  }

  try {
    YAML::Node root = YAML::LoadFile(file_path_.string());
    if (root["next_seq"]) {
      next_seq_ = root["next_seq"].as<std::uint64_t>();
    }
    if (root["robots"]) {
      for (const auto & kv : root["robots"]) {
        const auto id = kv.first.as<int>();
        Entry e;
        if (kv.second["success"]) e.success = kv.second["success"].as<int>();
        if (kv.second["failure"]) e.failure = kv.second["failure"].as<int>();
        if (kv.second["last_seq"]) e.last_seq = kv.second["last_seq"].as<std::uint64_t>();
        entries_[static_cast<std::uint8_t>(id)] = e;
      }
    }
    RCLCPP_INFO(
      rclcpp::get_logger("SkillAssignmentHistory"),
      "履歴ファイル読み込み完了: %s (entries=%zu, next_seq=%" PRIu64 ")", file_path_.c_str(),
      entries_.size(), static_cast<uint64_t>(next_seq_));
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("SkillAssignmentHistory"),
      "履歴ファイル読み込み失敗 (空状態で開始): %s, error=%s", file_path_.c_str(), ex.what());
    entries_.clear();
    next_seq_ = 1;
  }
}

bool SkillAssignmentHistory::save() const
{
  try {
    YAML::Node root;
    root["next_seq"] = next_seq_;
    YAML::Node robots;
    for (const auto & [id, e] : entries_) {
      YAML::Node entry;
      entry["success"] = e.success;
      entry["failure"] = e.failure;
      entry["last_seq"] = e.last_seq;
      robots[static_cast<int>(id)] = entry;
    }
    root["robots"] = robots;

    // 親ディレクトリを作成
    std::filesystem::create_directories(file_path_.parent_path());

    // 一時ファイルに書いてから rename することで原子的に更新する
    auto tmp_path = file_path_;
    tmp_path += ".tmp";
    {
      std::ofstream ofs(tmp_path);
      if (!ofs) {
        RCLCPP_ERROR(
          rclcpp::get_logger("SkillAssignmentHistory"), "履歴ファイル書き込み失敗 (open): %s",
          tmp_path.c_str());
        return false;
      }
      ofs << root;
    }
    std::filesystem::rename(tmp_path, file_path_);
    RCLCPP_INFO(
      rclcpp::get_logger("SkillAssignmentHistory"),
      "履歴ファイル書き込み成功: %s (entries=%zu, next_seq=%" PRIu64 ")", file_path_.c_str(),
      entries_.size(), static_cast<uint64_t>(next_seq_));
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SkillAssignmentHistory"), "履歴ファイル書き込み失敗: %s, error=%s",
      file_path_.c_str(), ex.what());
    return false;
  }
}
}  // namespace crane
