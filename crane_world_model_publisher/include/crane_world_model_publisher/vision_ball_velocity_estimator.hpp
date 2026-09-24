// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISION_BALL_VELOCITY_ESTIMATOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISION_BALL_VELOCITY_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <optional>

namespace crane
{
// Vision のボール観測がどの検出フレーム由来かを表す。
// UDP 経路では 1 つの検出フレームを 2 回処理するので、同じフレームかどうかをこれで見分ける。
// frame_number を送らない（常に 0 の）シミュレータでも別フレームを取り違えないよう、
// t_capture も比べる。
struct VisionFrameKey
{
  uint32_t camera_id{0};
  uint32_t frame_number{0};
  double t_capture{0.0};

  auto operator==(const VisionFrameKey &) const -> bool = default;
};

// Tracker が無いときのボール速度を、Vision の連続観測の位置差 / 受信時刻差で推定する。
// - 前回と同じフレームの観測は無視し、速度も履歴も変えない（新しい入力が無い統合・UDP の二重処理）
// - 初回と、前回の観測から max_gap_sec を超えて空いた後（検出の途切れ）は速度 0 から始める
// - 時刻差が 0 以下なら速度は前回のまま、観測だけ新しいものに置き換える
// - 非有限は 0、max_speed を超える速度は向きを保って max_speed に縮める
class VisionBallVelocityEstimator
{
public:
  VisionBallVelocityEstimator(double max_gap_sec, double max_speed)
  : max_gap_sec_(max_gap_sec), max_speed_(max_speed)
  {
  }

  auto update(const VisionFrameKey & key, const Eigen::Vector3d & position, double stamp_sec)
    -> Eigen::Vector3d
  {
    if (last_ && last_->key == key) {
      return velocity_;
    }

    if (!last_ || stamp_sec - last_->stamp_sec > max_gap_sec_) {
      velocity_ = Eigen::Vector3d::Zero();
    } else if (const double dt = stamp_sec - last_->stamp_sec; dt > 0.0) {
      velocity_ = (position - last_->position) / dt;
      const double speed = velocity_.norm();
      if (!std::isfinite(speed)) {
        velocity_ = Eigen::Vector3d::Zero();
      } else if (speed > max_speed_) {
        velocity_ *= max_speed_ / speed;
      }
    }

    last_ = Sample{key, position, stamp_sec};
    return velocity_;
  }

private:
  struct Sample
  {
    VisionFrameKey key;
    Eigen::Vector3d position;
    double stamp_sec;
  };

  double max_gap_sec_;
  double max_speed_;
  std::optional<Sample> last_;
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_BALL_VELOCITY_ESTIMATOR_HPP_
