// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_physics/kicker_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace crane
{

class KickerModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用設定
    config_.straight_kick_powers = {0.0, 0.5, 1.0};
    config_.straight_kick_speeds = {0.0, 3.0, 6.0};
    config_.chip_kick_powers = {0.0, 0.5, 1.0};
    config_.chip_kick_distances = {0.0, 1.0, 2.0};

    kicker_model_ = std::make_unique<KickerModel>(config_);
  }

  KickerModel::Config config_;
  std::unique_ptr<KickerModel> kicker_model_;
};

// ===== 基本機能テスト =====

TEST_F(KickerModelTest, DefaultConstructor)
{
  KickerModel default_model;
  EXPECT_TRUE(default_model.validateConfig());
}

TEST_F(KickerModelTest, ConfigConstructor)
{
  EXPECT_TRUE(kicker_model_->validateConfig());
  EXPECT_EQ(kicker_model_->getConfig().straight_kick_powers.size(), 3);
}

// ===== ストレートキック予測テスト =====

TEST_F(KickerModelTest, PredictStraightKickSpeed)
{
  // 境界値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(1.0), 6.0);

  // 中間値テスト（線形補間）
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.5), 3.0);

  // 細かい補間テスト
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.25), 1.5);
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.75), 4.5);
}

TEST_F(KickerModelTest, PredictStraightKickSpeedInvalidInput)
{
  EXPECT_THROW(kicker_model_->predictStraightKickSpeed(-0.1), std::runtime_error);
  EXPECT_THROW(kicker_model_->predictStraightKickSpeed(1.1), std::runtime_error);
}

// ===== ストレートキック力計算テスト =====

TEST_F(KickerModelTest, CalculateStraightKickPower)
{
  // 境界値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(6.0), 1.0);

  // 中間値テスト（逆補間）
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(3.0), 0.5);

  // 細かい逆補間テスト
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(1.5), 0.25);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(4.5), 0.75);
}

TEST_F(KickerModelTest, CalculateStraightKickPowerInvalidInput)
{
  EXPECT_THROW(kicker_model_->calculateStraightKickPower(-1.0), std::runtime_error);
}

// ===== チップキック予測テスト =====

TEST_F(KickerModelTest, PredictChipKickDistance)
{
  // 境界値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(1.0), 2.0);

  // 中間値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(0.5), 1.0);
}

TEST_F(KickerModelTest, CalculateChipKickPower)
{
  // 境界値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(2.0), 1.0);

  // 中間値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(1.0), 0.5);
}

// ===== BallPhysicsModel統合テスト =====

TEST_F(KickerModelTest, BallPhysicsModelSetGet)
{
  // 初期状態ではnullptr
  EXPECT_EQ(kicker_model_->getBallPhysicsModel(), nullptr);

  // TODO: PR5でBallPhysicsModel実装後に詳細テストを追加
  // 現在はnullptrチェックのみ

  // BallPhysicsModel統合機能テスト（現在はエラーチェックのみ）
  EXPECT_THROW(kicker_model_->calculateKickPowerForStopDistance(2.0), std::runtime_error);
  EXPECT_THROW(kicker_model_->predictStopDistance(0.5), std::runtime_error);
  EXPECT_THROW(kicker_model_->predictChipKickTotalDistance(0.5), std::runtime_error);
}

// ===== 設定検証テスト =====

TEST_F(KickerModelTest, ValidateConfig)
{
  EXPECT_TRUE(kicker_model_->validateConfig());

  // 無効な設定テスト
  KickerModel::Config invalid_config;
  invalid_config.straight_kick_powers = {1.0, 0.5, 0.0};  // 降順（無効）
  invalid_config.straight_kick_speeds = {0.0, 3.0, 6.0};

  EXPECT_THROW(KickerModel invalid_model(invalid_config), std::runtime_error);
}

TEST_F(KickerModelTest, IsValidKickPower)
{
  EXPECT_TRUE(KickerModel::isValidKickPower(0.0));
  EXPECT_TRUE(KickerModel::isValidKickPower(0.5));
  EXPECT_TRUE(KickerModel::isValidKickPower(1.0));

  EXPECT_FALSE(KickerModel::isValidKickPower(-0.1));
  EXPECT_FALSE(KickerModel::isValidKickPower(1.1));
}

// ===== 設定管理テスト =====

TEST_F(KickerModelTest, SetConfigValid)
{
  KickerModel::Config new_config;
  new_config.straight_kick_powers = {0.0, 0.3, 0.7, 1.0};
  new_config.straight_kick_speeds = {0.0, 2.0, 5.0, 8.0};
  new_config.chip_kick_powers = {0.0, 0.4, 1.0};
  new_config.chip_kick_distances = {0.0, 1.5, 3.0};

  EXPECT_NO_THROW(kicker_model_->setConfig(new_config));
  EXPECT_EQ(kicker_model_->getConfig().straight_kick_powers.size(), 4);
  EXPECT_EQ(kicker_model_->getConfig().chip_kick_powers.size(), 3);
}

TEST_F(KickerModelTest, SetConfigInvalid)
{
  KickerModel::Config invalid_config;
  invalid_config.straight_kick_powers = {};  // 空配列
  invalid_config.straight_kick_speeds = {0.0, 3.0};

  EXPECT_THROW(kicker_model_->setConfig(invalid_config), std::runtime_error);
}

// ===== 線形補間精度テスト =====

TEST_F(KickerModelTest, LinearInterpolationAccuracy)
{
  // より詳細な補間テスト
  KickerModel::Config detailed_config;
  detailed_config.straight_kick_powers = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
  detailed_config.straight_kick_speeds = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};

  KickerModel detailed_model(detailed_config);

  // 中間点の補間精度テスト
  EXPECT_NEAR(detailed_model.predictStraightKickSpeed(0.1), 0.5, 1e-9);
  EXPECT_NEAR(detailed_model.predictStraightKickSpeed(0.3), 1.5, 1e-9);
  EXPECT_NEAR(detailed_model.predictStraightKickSpeed(0.7), 3.5, 1e-9);

  // 逆補間精度テスト
  EXPECT_NEAR(detailed_model.calculateStraightKickPower(0.5), 0.1, 1e-9);
  EXPECT_NEAR(detailed_model.calculateStraightKickPower(2.5), 0.5, 1e-9);
  EXPECT_NEAR(detailed_model.calculateStraightKickPower(4.5), 0.9, 1e-9);
}

// ===== ファクトリー関数テスト =====

TEST_F(KickerModelTest, FactoryFunctions)
{
  // デフォルトKickerModel作成
  auto default_model = createDefaultKickerModel();
  EXPECT_NE(default_model, nullptr);
  EXPECT_TRUE(default_model->validateConfig());

  // YAML読み込み（存在しないファイル指定でもデフォルト設定で作成される）
  auto yaml_model = createKickerModelFromYAML("/nonexistent/path.yaml");
  EXPECT_NE(yaml_model, nullptr);
  EXPECT_TRUE(yaml_model->validateConfig());

  // BallPhysicsModel統合（nullptr指定）
  auto integrated_model = createIntegratedKickerModel("/nonexistent/path.yaml", nullptr);
  EXPECT_NE(integrated_model, nullptr);
  EXPECT_EQ(integrated_model->getBallPhysicsModel(), nullptr);
}

// ===== エッジケーステスト =====

TEST_F(KickerModelTest, EdgeCases)
{
  // 範囲外の値（クランプ動作確認）
  KickerModel::Config edge_config;
  edge_config.straight_kick_powers = {0.2, 0.8};
  edge_config.straight_kick_speeds = {1.0, 4.0};

  KickerModel edge_model(edge_config);

  // 範囲外の値は端点の値を返すことを確認
  EXPECT_DOUBLE_EQ(edge_model.predictStraightKickSpeed(0.0), 1.0);   // 下限クランプ
  EXPECT_DOUBLE_EQ(edge_model.predictStraightKickSpeed(1.0), 4.0);   // 上限クランプ
  EXPECT_DOUBLE_EQ(edge_model.calculateStraightKickPower(0.5), 0.2); // 下限クランプ
  EXPECT_DOUBLE_EQ(edge_model.calculateStraightKickPower(5.0), 0.8); // 上限クランプ
}

}  // namespace crane