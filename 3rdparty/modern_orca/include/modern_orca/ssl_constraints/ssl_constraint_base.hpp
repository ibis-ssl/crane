// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../concepts.hpp"
#include "../constraints/constraint_base.hpp"
#include "../types.hpp"

// SSL-specific includes for world model integration
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane::modern_orca
{

// SSL固有の制約タイプ
enum class SSLConstraintType {
  BALL_AVOIDANCE,
  PENALTY_AREA_AVOIDANCE,
  BALL_PLACEMENT_AVOIDANCE,
  FIELD_BOUNDARY,
  REFEREE_COMMAND,
  ROBOT_COLLISION
};

template <Agent AgentType>
class SSLConstraintBase : public ConstraintBase<AgentType>
{
public:
  using agent_type = AgentType;

  virtual ~SSLConstraintBase() = default;

  // SSL固有の制約インターフェース
  virtual void updateFromWorldModel(const crane::WorldModelWrapper::SharedPtr & world_model) = 0;
  virtual void updateFromRefereeCommand(
    const robocup_ssl_msgs::msg::Referee::_command_type & command) = 0;

  // 制約を動的に有効/無効化
  virtual void setEnabled(bool enabled) { enabled_ = enabled; }
  virtual bool isEnabled() const noexcept { return enabled_; }

  // 有効状態を組み込むためのisActiveをオーバーライド
  bool isActive() const noexcept override { return enabled_ && isConstraintActive(); }

  virtual SSLConstraintType getConstraintType() const noexcept = 0;

protected:
  SSLConstraintBase() : enabled_(true) {}

  // サブクラスはisActiveの代わりにここを実装すべき
  virtual bool isConstraintActive() const noexcept { return true; }

  bool enabled_ = true;
};

// CircularAgent用の便利なテンプレートエイリアス
using SSLConstraint = SSLConstraintBase<class CircularAgent>;

}  // namespace crane::modern_orca
