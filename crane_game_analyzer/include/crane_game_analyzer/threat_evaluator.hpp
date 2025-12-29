// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__THREAT_EVALUATOR_HPP_
#define CRANE_GAME_ANALYZER__THREAT_EVALUATOR_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/threat_assignment.hpp>
#include <crane_msgs/msg/threat_info.hpp>
#include <optional>
#include <vector>

namespace crane
{

/**
 * @brief 脅威評価設定
 *
 * Sumatraの DefenseThreatRater を参考に設計
 */
struct ThreatEvaluatorConfig
{
  // 重み付け（4因子）
  double weight_redirect_angle = 0.9;
  double weight_pen_area_border = 1.0;
  double weight_facing_goal = 0.8;  ///< ゴール方向を向いているか
  double weight_ball_access = 1.2;  ///< ボールアクセス（高優先度）

  // 距離減衰
  double danger_dropoff_x = 1.0;  // 脅威減衰開始X座標（相対）

  // リダイレクト角度閾値
  double max_good_redirect_angle_deg = 45.0;
  double max_bad_redirect_angle_deg = 75.0;

  // ボール予測
  double ball_lookahead_sec = 0.1;
  double check_ball_direction_vel_threshold = 1.5;
};

/**
 * @brief 脅威評価の詳細結果
 */
struct ThreatRatingDetail
{
  double total_score = 0.0;
  double score_redirect_angle = 0.0;
  double score_pen_area_border = 0.0;
  double score_facing_goal = 0.0;  ///< ゴール方向を向いている度合い
  double score_ball_access = 0.0;  ///< ボールへのアクセスしやすさ
  double distance_factor = 1.0;
};

/**
 * @brief ボール脅威データ（内部用）
 */
struct BallThreat
{
  enum class SourceType { BALL, GOAL_SHOT, PASS_RECEIVE, BOT_CLOSE_TO_BALL };

  SourceType source_type = SourceType::BALL;
  Point source_position{0, 0};
  Vector2 velocity{0, 0};
  Segment threat_line;
  std::optional<Segment> protection_line;
  std::optional<uint8_t> pass_receiver_id;
};

/**
 * @brief ロボット脅威データ（内部用）
 *
 * RobotInfoへのポインタを保持し、位置・速度・その他の情報に直接アクセス可能
 */
struct RobotThreat
{
  /// RobotInfoへのポインタ（位置・速度等はここから取得）
  std::shared_ptr<RobotInfo> robot;

  /// 脅威ライン（ロボット位置からゴールへの線分）
  Segment threat_line;
  std::optional<Segment> protection_line;
  std::optional<Point> protection_position;
  double threat_rating = 0.0;
  ThreatRatingDetail rating_detail;

  enum class DefenseStrategy { CENTER_BACK, MAN_TO_MAN };
  DefenseStrategy recommended_strategy = DefenseStrategy::CENTER_BACK;
};

/**
 * @brief 脅威評価クラス
 *
 * Sumatraの DefenseThreatRater, DefenseBallThreatCalc, DefenseBotThreatCalc を参考に実装
 */
class ThreatEvaluator
{
public:
  explicit ThreatEvaluator(const ThreatEvaluatorConfig & config = {});

  /**
   * @brief ボール脅威の計算
   */
  auto calculateBallThreat(const WorldModelWrapper & world_model) -> BallThreat;

  /**
   * @brief ロボット脅威の計算（優先度順）
   */
  auto calculateRobotThreats(const WorldModelWrapper & world_model, const BallThreat & ball_threat)
    -> std::vector<RobotThreat>;

  /**
   * @brief 単一ロボットの脅威評価
   */
  auto rateRobotThreat(
    const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot,
    const WorldModelWrapper & world_model) -> ThreatRatingDetail;

  /**
   * @brief 推奨守備者数の計算
   */
  auto calculateRecommendedDefenders(
    const BallThreat & ball_threat, const std::vector<RobotThreat> & robot_threats,
    int available_robots) -> int;

  /**
   * @brief BallThreat -> ThreatInfo メッセージ変換
   */
  auto toThreatInfoMsg(const BallThreat & threat) const -> crane_msgs::msg::ThreatInfo;

  /**
   * @brief RobotThreat -> ThreatInfo メッセージ変換
   */
  auto toThreatInfoMsg(const RobotThreat & threat) const -> crane_msgs::msg::ThreatInfo;

private:
  ThreatEvaluatorConfig config_;

  // ===== 個別スコア計算（4因子） =====

  /**
   * @brief リダイレクト角度スコア計算
   *
   * キッカー位置とロボットの向きを考慮したリダイレクト評価
   */
  auto calcRedirectAngleScore(
    const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot,
    const WorldModelWrapper & wm) const -> double;

  /**
   * @brief ペナルティエリア境界スコア計算
   *
   * ペナルティエリアに近いほど高スコア
   */
  auto calcPenAreaBorderScore(const Point & threat_pos, const WorldModelWrapper & wm) const
    -> double;

  /**
   * @brief ゴール方向を向いているかのスコア計算
   *
   * ロボットがゴール方向を向いているほど高スコア
   */
  auto calcFacingGoalScore(
    const std::shared_ptr<RobotInfo> & robot, const WorldModelWrapper & wm) const -> double;

  /**
   * @brief ボールへのアクセスしやすさスコア計算
   *
   * 台形速度プロファイルによる到達時間を評価
   */
  auto calcBallAccessScore(const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot) const
    -> double;

  /**
   * @brief ゴールまでの距離に基づく係数計算
   */
  auto calcDistanceToGoalFactor(
    const Point & threat_pos, double dropoff_percentage, const WorldModelWrapper & wm) const
    -> double;

  /**
   * @brief ボール脅威ソースの判定
   */
  auto determineBallThreatSource(const WorldModelWrapper & wm)
    -> std::pair<BallThreat::SourceType, Point>;

  /**
   * @brief 防御ライン計算
   */
  auto calculateProtectionLine(
    const Segment & threat_line, double min_distance, const WorldModelWrapper & wm)
    -> std::optional<Segment>;
};

}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__THREAT_EVALUATOR_HPP_
