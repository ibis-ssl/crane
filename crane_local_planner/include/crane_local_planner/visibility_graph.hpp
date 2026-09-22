// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_
#define CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <optional>
#include <vector>

namespace crane::visibility_graph
{

struct Obstacle
{
  enum class Type { CIRCLE, BOX, CAPSULE };

  Type type = Type::CIRCLE;
  Circle circle{Point::Zero(), 0.0};
  Box box{};
  Capsule capsule{};
  // nearestDynamicEscape 専用の形状。経路計画用の capsule は相対速度で自機側へ伸びるため、
  // 静止ロボットへ近づくだけで「食い込み」になり退避が誤発火する。退避判定は相手の実位置と
  // 相手自身の予測移動だけで行う。makeCapsule では capsule と同じ形状になる。
  Capsule escape_capsule{};

  // 移動するロボッか
  bool is_dynamic_robot = false;

  static auto makeCircle(const Point & center, double radius) -> Obstacle;
  static auto makeBox(const Box & box) -> Obstacle;
  static auto makeCapsule(const Capsule & capsule, bool is_dynamic_robot = false) -> Obstacle;
  /**
  * @brief 指定した点が、障害物の境界からどれだけ離れているかを計算する
  *
  * @param point 指定した点
  * @retval <0 点が障害物の内側にある
  * @retval 0 点が障害物の境界上にある
  * @retval >0 点が障害物の外側にある
  */
  [[nodiscard]] auto signedDistance(const Point & point) const -> double;

  /**
   * @brief 指定した点が障害物の内側にあるときに、外側に押し出す
   *
   * @param point 指定点
   * @param clearance クリアランス
   * @return Point 押し出したあとの点
   */
  [[nodiscard]] auto projectOutside(const Point & point, double clearance = 1e-3) const -> Point;
};

class VisibilityGraph
{
public:
  struct Config
  {
    int circle_samples = 12;
    int capsule_end_samples = 6;
    double node_clearance = 1e-3;
  };

  void configure(const Config & config) { config_ = config; }

  /**
   * @brief
   *
   * @param start 経路の始点
   * @param goal 経路の終点
   * @param obstacles 障害物リスト
   * @return std::optional<std::vector<Point>> 生成された経路。経路が生成できなかった場合は std::nullopt
   */
  [[nodiscard]] auto plan(
    const Point & start, const Point & goal, const std::vector<Obstacle> & obstacles) const
    -> std::optional<std::vector<Point>>;

  /**
  * @brief 経路の各線分に障害物との干渉がないかチェックする
  *
  * @param path チェック対象のパス
  * @param obstacles 障害物リスト
  * @retval true 経路が障害物と干渉していない
  * @retval false 経路が障害物と干渉する
  */
  [[nodiscard]] auto isPathVisible(
    const std::vector<Point> & path, const std::vector<Obstacle> & obstacles) const -> bool;

  /**
    * @brief 指定した点が、移動ロボットの障害物に食い込んでいる場合、障害物の外側に押し出す
    *
    * 判定には Obstacle::escape_capsule（相手の実位置 + 相手自身の予測移動）を使う。
    *
    * @param point 指定点
    * @param obstacles 障害物リスト
    * @param inside_margin 「食い込み」と判定する符号付き距離の上限。退避中はこの値を正にして
    *                      ヒステリシスをかけ、境界すれすれで退避と復帰を往復させない
    * @param escape_clearance 退避先を境界からどれだけ外側に置くか（node_clearance に加算）。
    *                         inside_margin よりロボット側の到達許容誤差ぶん以上大きくすること。
    *                         同程度だと機体が解除線の手前で止まり、退避が固着する
    * @return std::optional<Point> 押し出したあとの点
    */
  [[nodiscard]] auto nearestDynamicEscape(
    const Point & point, const std::vector<Obstacle> & obstacles, double inside_margin = 0.0,
    double escape_clearance = 0.0) const -> std::optional<Point>;

private:
  /**
   * @brief 障害物を囲む軸平行矩形境界。辺との重なりが無ければ厳密判定を省ける。
   * 円・カプセルは半径ぶん広げてあるので、この判定は必ず保守的（取りこぼさない）
   *
   */
  struct ObstacleBounds
  {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
  };

  /**
   * @brief 衝突判定の簡易化のために、障害物を囲む矩形を作成する
   *
   * @param obstacles 障害物リスト
   * @return std::vector<ObstacleBounds> 作成した矩形境界
   */
  [[nodiscard]] static auto computeObstacleBounds(const std::vector<Obstacle> & obstacles)
    -> std::vector<ObstacleBounds>;

  /**
  * @brief 矩形境界リストと、障害物リストから、線分が障害物と干渉していないかを判定する
  *
  * @param from 始点
  * @param to 終点
  * @param obstacles 障害物リスト
  * @param bounds 障害物の矩形境界リスト
  * @retval true 経路が障害物と干渉していない
  * @retval false 経路が障害物と干渉する
  */
  [[nodiscard]] auto isEdgeVisible(
    const Point & from, const Point & to, const std::vector<Obstacle> & obstacles,
    const std::vector<ObstacleBounds> & bounds) const -> bool;

  /**
   * @brief 障害物の周りに回避するための座標を生成する
   *
   * @param obstacles 障害物
   * @return std::vector<Point> 回避するために生成した座標
   */
  [[nodiscard]] auto generateNodes(const std::vector<Obstacle> & obstacles) const
    -> std::vector<Point>;

  Config config_;
};

/**
 * @brief 経路再計画の戦略
 */
enum class ReplanAction {
  USE_DIRECT_PATH,      ///< 直接経路を使用する
  REUSE_RETAINED_PATH,  ///< 保持している経路を再利用する
  RUN_FULL_REPLAN       ///< 経路を完全に再計画する
};

/**
 * @brief どの経路を使うか判定する
 *
 * @param has_safe_retained_path 既存経路が安全か
 * @param direct_path_visible 直線経路が安全か
 * @param full_replan_due 経路再生成をする時刻になったか
 * @retval USE_DIRECT_PATH: 直線経路が安全なのでそれを使う
 * @retval REUSE_RETAINED_PATH: 保持経路が安全なのでそれを使う
 * @retval RUN_FULL_REPLAN: 全グラフ再計算を行う
 */
[[nodiscard]] auto decideReplanAction(
  bool has_safe_retained_path, bool direct_path_visible, bool full_replan_due) -> ReplanAction;

/**
 * @brief 経路の長さを計算する
 *
 * @param path 経路
 * @return double 経路の長さ
 */
[[nodiscard]] auto pathLength(const std::vector<Point> & path) -> double;

/**
 * @brief firstWaypointBeyond の結果
 */
struct WaypointChoice
{
  size_t index = 0;         ///< path 上の添字（path.size() >= 2 なら 1 以上）
  double arc_length = 0.0;  ///< path[0] から path[index] までの弧長
};

/**
 * @brief 現在位置（path[0]）から十分離れた最初の中継点を選ぶ
 *
 * trimPathFromCurrent の射影点や plan() の押し出し始点は現在位置から 1mm 程度のことがあり、
 * そのまま目標にすると「今いる場所へ行け、ただし終端速度は大きく」という指令になり、
 * 終端速度ベクトルの向きだけがフレームごとに反転する。見つからなければ終点を返す。
 *
 * @param path 経路（2 点以上）
 * @param min_distance 中継点までの最小弧長 [m]
 * @return WaypointChoice 選んだ中継点の添字と弧長
 */
[[nodiscard]] auto firstWaypointBeyond(const std::vector<Point> & path, double min_distance)
  -> WaypointChoice;

/**
 * @brief 既に通過した経路を切り捨てる
 *
 * @param current 現在位置
 * @param path 経路
 * @return std::vector<Point> 切り詰めた経路（先頭は current、経路が 2 点未満なら空）
 */
[[nodiscard]] auto trimPathFromCurrent(const Point & current, const std::vector<Point> & path)
  -> std::vector<Point>;

/**
 * @brief 経路にそって移動量分移動した先の位置を求める
 *
 * @param path 複数ポイントからなる経路
 * @param distance 移動量
 * @return Point 移動量分移動した先の位置（経路長を超える場合は終点）
 */
[[nodiscard]] auto pointAtDistance(const std::vector<Point> & path, double distance) -> Point;

/**
 * @brief サブゴールの選び方
 */
enum class SubgoalMode {
  LOOKAHEAD,      ///< 経路上 lookahead_distance 先の点
  WAYPOINT,       ///< 先読み点が不可視で、最初の中継点 path[1] を採用
  WAYPOINT_SKIP,  ///< path[1] が現在位置に近すぎるので、より先の中継点を採用
  PATH_POINT,     ///< 読み飛ばした中継点へ直接向かえないので、経路上 min_subgoal_distance 先の点
};

[[nodiscard]] auto toString(SubgoalMode mode) -> const char *;

/**
 * @brief selectSubgoal の結果
 */
struct SubgoalChoice
{
  Point point = Point::Zero();
  double arc_length = 0.0;  ///< 現在位置(path[0]) から point までの弧長
  SubgoalMode mode = SubgoalMode::LOOKAHEAD;
};

/**
 * @brief 経路からサブゴールを選ぶ
 *
 * 基本は経路上 lookahead_distance 先の点。そこへ現在位置から直接向かえないときは中継点へ
 * フォールバックするが、現在位置から min_subgoal_distance 未満の中継点は読み飛ばす。
 * 読み飛ばした先の中継点へ現在位置から直接向かうと障害物の角をかすめる場合は、
 * 経路に沿って min_subgoal_distance だけ進んだ点を採用し、退化した path[1] には戻さない。
 *
 * @param graph 可視性判定に使うグラフ
 * @param current 現在位置（path[0] と一致している前提）
 * @param path 選択済みの経路（2 点以上）
 * @param obstacles 障害物
 * @param lookahead_distance 先読み距離 [m]
 * @param min_subgoal_distance 中継点までの最小弧長 [m]
 * @return SubgoalChoice
 */
[[nodiscard]] auto selectSubgoal(
  const VisibilityGraph & graph, const Point & current, const std::vector<Point> & path,
  const std::vector<Obstacle> & obstacles, double lookahead_distance, double min_subgoal_distance)
  -> SubgoalChoice;

/**
 * @brief 予測されたロボットの障害物を作成する
 *
 * @param ego_velocity 自分の速度
 * @param ego_radius 自分の半径
 * @param other_position 他のロボットの位置
 * @param other_velocity 他のロボットの速度
 * @param other_radius 他のロボットの半径
 * @param prediction_horizon 予測時間
 * @param safety_margin セーフティマージン
 * @return Obstacle 作成された障害物
 */
[[nodiscard]] auto makePredictedRobotObstacle(
  const Vector2 & ego_velocity, double ego_radius, const Point & other_position,
  const Vector2 & other_velocity, double other_radius, double prediction_horizon,
  double safety_margin) -> Obstacle;

}  // namespace crane::visibility_graph

#endif  // CRANE_LOCAL_PLANNER__VISIBILITY_GRAPH_HPP_
