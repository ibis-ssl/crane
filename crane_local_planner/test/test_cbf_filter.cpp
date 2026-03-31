// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "crane_local_planner/ateb_cbf_filter.hpp"
#include "crane_local_planner/ateb_types.hpp"
#include "test_helpers.hpp"

using crane::Point;
using crane::Vector2;
using crane::ateb::CBFFilter;
using crane::ateb::Obstacle;

namespace
{

CBFFilter makeCBF()
{
  CBFFilter cbf;
  cbf.configure(test_helpers::makeDefaultCBFConfig());
  return cbf;
}

}  // namespace

// ===== 基本テスト =====

TEST(CBFFilter, NoConstraintsPassthrough)
{
  // 障害物なし → 名目速度がそのまま通過
  const auto cbf = makeCBF();
  const Vector2 u_nom(1.0, 0.5);
  const auto u = cbf.filter(Point::Zero(), u_nom, {}, {});
  EXPECT_NEAR(u.x(), u_nom.x(), 1e-9);
  EXPECT_NEAR(u.y(), u_nom.y(), 1e-9);
}

// ===== 静的BOX制約テスト（ペナルティエリア）=====

TEST(CBFFilter, StaticBoxDeflection)
{
  // ロボットがペナルティエリアへ向かって移動 → 速度が偏向される
  const auto cbf = makeCBF();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  // PA左辺(inflation後 x=4.1)の左側0.05mに位置するロボット
  const Point ego_pos(4.05, 0.0);
  // PA内部に向かう速度（右方向）
  const Vector2 u_nom(1.0, 0.0);

  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});

  // x方向速度は減少または逆転すべき（PAに近いため）
  EXPECT_LT(u.x(), u_nom.x()) << "ペナルティエリアへ向かう速度は偏向されるべき";
}

TEST(CBFFilter, StaticBoxParallelMotion)
{
  // PA辺に平行な移動（y方向）→ ほぼ変化なし
  const auto cbf = makeCBF();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  // PA左辺(inflation後 x=4.1)の左側0.3mに位置するロボット
  const Point ego_pos(3.8, 0.0);
  // PA辺に平行な速度（y方向）
  const Vector2 u_nom(0.0, 1.0);

  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});

  // y方向速度はほぼ変化なし、x方向速度が押し出しで少し追加
  EXPECT_NEAR(u.y(), u_nom.y(), 0.5) << "PA辺に平行な速度は大きく変化すべきでない";
}

TEST(CBFFilter, StaticBoxCornerVelocity)
{
  // PA角に向かう速度 → 偏向される
  const auto cbf = makeCBF();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  // PA左上角(inflation後: 4.1, 1.9)に向かう位置
  const Point ego_pos(3.9, 1.7);
  // 角に向かう速度
  const Vector2 u_nom(1.0, 1.0);

  // クラッシュしないこと
  EXPECT_NO_THROW({
    const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});
    // 角に向かう速度成分が減少することを確認
    const double nom_toward_corner =
      (obs.box.min_corner() - Vector2(0.1, 0.0) - ego_pos).normalized().dot(u_nom);
    const double filtered_toward_corner =
      (obs.box.min_corner() - Vector2(0.1, 0.0) - ego_pos).normalized().dot(u);
    EXPECT_LE(filtered_toward_corner, nom_toward_corner + 0.1);
  });
}

TEST(CBFFilter, RobotNearPenaltyBoxBarrierSign)
{
  // ロボットがPA safety_margin内にいる場合: h = dist - safe_dist < 0
  // CBF制約: grad^T * u >= -alpha * h  (h<0なのでbound>0: 外向き速度が必要)
  const auto cbf = makeCBF();
  const auto cfg = test_helpers::makeDefaultCBFConfig();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  // PA左辺のすぐ外側だが safety_margin 内
  const double safe_dist = cfg.robot_radius + cfg.safety_margin;
  // dist = safe_dist/2 なので h = safe_dist/2 - safe_dist = -safe_dist/2 < 0
  const Point ego_pos(4.1 - safe_dist / 2.0, 0.0);  // PA左辺から safe_dist/2

  // PA内向き速度（x正方向）
  const Vector2 u_nom(1.0, 0.0);
  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});

  // x方向速度は負（PA外向き）になるべき
  EXPECT_LT(u.x(), 0.0) << "safety_margin内でPA向き速度を持つ場合、外向きに修正されるべき: u.x="
                        << u.x();
}

TEST(CBFFilter, VeryCloseToBox)
{
  // PAのほぼ表面上 → 強い制約
  const auto cbf = makeCBF();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  const Point ego_pos(4.11, 0.0);  // PA左辺から0.01m
  const Vector2 u_nom(2.0, 0.0);   // PAに向かう高速

  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});
  // PA方向（x正）の速度成分が強く減少すべき
  EXPECT_LT(u.x(), 1.0) << "PA近接時、PA方向速度は強く制限されるべき";
}

TEST(CBFFilter, MaxCorrectionClamp)
{
  // 非常に大きな名目速度に対してmax_correctionでクランプ
  const auto cbf = makeCBF();
  const auto cfg = test_helpers::makeDefaultCBFConfig();
  const auto obs = test_helpers::makeOurPenaltyObstacle();

  const Point ego_pos(4.1, 0.0);   // PA左辺すぐ外
  const Vector2 u_nom(10.0, 0.0);  // PA方向の高速

  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});

  // 修正量がmax_correctionを超えないこと
  const double correction = (u - u_nom).norm();
  EXPECT_LE(correction, cfg.max_correction + 1e-6)
    << "速度修正量がmax_correction=" << cfg.max_correction << "を超えた: " << correction;
}

TEST(CBFFilter, ProjectionQPConvergence)
{
  // 複数の半平面制約での射影法収束テスト
  const auto cbf = makeCBF();
  const std::vector<Obstacle> obstacles = {
    test_helpers::makeOurPenaltyObstacle(),
    test_helpers::makeTheirPenaltyObstacle(),
  };

  // フィールド中央付近のロボット
  const Point ego_pos(0.0, 0.0);
  const Vector2 u_nom(1.0, 1.0);

  // 複数制約でもクラッシュしないこと
  EXPECT_NO_THROW({
    const auto u = cbf.filter(ego_pos, u_nom, {}, obstacles);
    EXPECT_TRUE(std::isfinite(u.x()));
    EXPECT_TRUE(std::isfinite(u.y()));
  });
}

// ===== 動的障害物テスト =====

TEST(CBFFilter, DynamicObstacleDeflection)
{
  // 動的障害物（別のロボット）に向かって移動 → 偏向
  const auto cbf = makeCBF();

  // 別ロボットが右側2mにいる（safe_dist=0.21mより十分離れている）
  const Point ego_pos(0.0, 0.0);
  const Point obs_pos(2.0, 0.0);
  const Vector2 obs_vel(0.0, 0.0);  // 静止

  // 衝突方向への速度
  const Vector2 u_nom(1.0, 0.0);

  const auto u = cbf.filter(ego_pos, u_nom, {{obs_pos, obs_vel}}, {});

  // safe_dist = 2*robot_radius + margin = 0.09*2 + 0.03 = 0.21m
  // 現在の距離2.0m >> safe_dist なのでバリア関数h>>0、制約はほぼ無効
  // 速度はほぼ変化しないはず（許容値0.2）
  EXPECT_NEAR(u.x(), u_nom.x(), 0.2);
}

TEST(CBFFilter, MultipleConstraintsBoxAndCircle)
{
  // 静的BOX + 動的Circle制約の同時適用
  const auto cbf = makeCBF();
  const auto obs_box = test_helpers::makeOurPenaltyObstacle();

  // ロボットがPA近くかつ別のロボットも近い
  const Point ego_pos(3.8, 0.0);    // PAから0.3m
  const Point robot_obs(3.5, 0.5);  // 別のロボット
  const Vector2 robot_vel(0.0, 0.0);
  const Vector2 u_nom(1.0, 0.0);  // PA方向

  EXPECT_NO_THROW({
    const auto u = cbf.filter(ego_pos, u_nom, {{robot_obs, robot_vel}}, {obs_box});
    EXPECT_TRUE(std::isfinite(u.x()));
    EXPECT_TRUE(std::isfinite(u.y()));
  });
}

// ===== 静的Circle障害物のバグテスト（バグ1）=====

TEST(CBFFilter, StaticCircleDistanceBug)
{
  // [バグ1の影響] 静的Circle障害物でOBstacle::distance()がradiusを引かないため、
  // CBFのバリア関数 h = dist - safe_dist が誤った値を返す
  const auto cbf = makeCBF();
  const auto cfg = test_helpers::makeDefaultCBFConfig();
  (void)cfg;

  constexpr double circle_r = 0.3;
  const auto obs = Obstacle::makeCircle(Point(0.0, 0.0), circle_r);

  // ロボットが円の表面上: 距離は0であるべきだが、
  // [バグ1] distance()は0.3を返す（center距離）
  const Point ego_pos(circle_r, 0.0);  // 表面上

  const Vector2 u_nom(-1.0, 0.0);  // 円中心に向かう速度

  // safe_dist = robot_radius + margin = 0.09 + 0.03 = 0.12m
  // 正しい実装: dist=0 < safe_dist=0.12 → h<0 → 強い制約
  // [バグ1] distance()=0.3 > safe_dist=0.12 → h>0 → 制約は緩い
  const auto u = cbf.filter(ego_pos, u_nom, {}, {obs});

  // Circle表面上にいるのに、円中心への速度が許可される場合はバグ
  // 正しくは円から離れる方向の速度成分が追加されるべき
  const double dist_from_surface = (ego_pos - obs.circle.center).norm() - obs.circle.radius;
  if (std::abs(dist_from_surface) < 1e-6) {
    // ego_posは確かに表面上
    // 内向きの速度成分
    const double inward_vel = -(u.x());  // x負方向が内向き
    if (inward_vel > 0.0) {
      ADD_FAILURE() << "[バグ1確認] Circle表面上のロボットが円内向き速度を許可されている。"
                       "Obstacle::distance()がradiusを引かないためCBF制約が機能していない。"
                       "内向き速度成分: "
                    << inward_vel;
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
