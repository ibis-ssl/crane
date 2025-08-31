// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

#include "crane_physics/ball_physics_model.hpp"
#include "crane_physics/kicker_model.hpp"

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

    kicker_model_ = std::make_shared<KickerModel>(config_);
    ball_physics_model_ = std::make_shared<BallPhysicsModel>();
  }

  KickerModel::Config config_;
  std::shared_ptr<KickerModel> kicker_model_;
  std::shared_ptr<BallPhysicsModel> ball_physics_model_;
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
  EXPECT_EQ(kicker_model_->getConfig().straight_kick_speeds.size(), 3);
}

TEST_F(KickerModelTest, InvalidConfig)
{
  KickerModel::Config invalid_config;
  invalid_config.straight_kick_powers = {0.0, 0.5};  // サイズ不一致
  invalid_config.straight_kick_speeds = {0.0, 3.0, 6.0};

  // コンストラクタ呼び出しは一時オブジェクト生成をブロックで明示
  EXPECT_THROW({ KickerModel dummy(invalid_config); }, std::runtime_error);
}

// ===== キック力予測テスト =====

TEST_F(KickerModelTest, PredictStraightKickSpeed)
{
  // 境界値テスト
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(1.0), 6.0);

  // 中間値テスト（線形補間）
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(0.5), 3.0);

  // 範囲外値はクランプされる
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(-0.1), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictStraightKickSpeed(1.1), 6.0);
}

TEST_F(KickerModelTest, PredictChipKickDistance)
{
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(1.0), 2.0);
  EXPECT_DOUBLE_EQ(kicker_model_->predictChipKickDistance(0.5), 1.0);
}

TEST_F(KickerModelTest, InvalidKickPower)
{
  EXPECT_THROW(kicker_model_->predictStraightKickSpeed(-0.5), std::runtime_error);
  EXPECT_THROW(kicker_model_->predictStraightKickSpeed(1.5), std::runtime_error);
}

// ===== キック力計算テスト =====

TEST_F(KickerModelTest, CalculateStraightKickPower)
{
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(6.0), 1.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateStraightKickPower(3.0), 0.5);
}

TEST_F(KickerModelTest, CalculateChipKickPower)
{
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(0.0), 0.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(2.0), 1.0);
  EXPECT_DOUBLE_EQ(kicker_model_->calculateChipKickPower(1.0), 0.5);
}

TEST_F(KickerModelTest, InvalidTargetValues)
{
  EXPECT_THROW(kicker_model_->calculateStraightKickPower(-1.0), std::runtime_error);
  EXPECT_THROW(kicker_model_->calculateChipKickPower(-0.5), std::runtime_error);
}

// ===== 双方向変換テスト =====

TEST_F(KickerModelTest, BidirectionalConversionStraight)
{
  double original_power = 0.7;
  double predicted_speed = kicker_model_->predictStraightKickSpeed(original_power);
  double calculated_power = kicker_model_->calculateStraightKickPower(predicted_speed);

  EXPECT_NEAR(calculated_power, original_power, 1e-10);
}

TEST_F(KickerModelTest, BidirectionalConversionChip)
{
  double original_power = 0.3;
  double predicted_distance = kicker_model_->predictChipKickDistance(original_power);
  double calculated_power = kicker_model_->calculateChipKickPower(predicted_distance);

  EXPECT_NEAR(calculated_power, original_power, 1e-10);
}

// ===== BallPhysicsModel統合テスト =====

TEST_F(KickerModelTest, BallPhysicsModelIntegration)
{
  kicker_model_->setBallPhysicsModel(ball_physics_model_);
  EXPECT_EQ(kicker_model_->getBallPhysicsModel(), ball_physics_model_);

  // 停止距離計算テスト（BallPhysicsModelが必要）
  double target_distance = 2.0;  // 2m
  EXPECT_NO_THROW({
    double kick_power = kicker_model_->calculateKickPowerForStopDistance(target_distance);
    EXPECT_GE(kick_power, 0.0);
    EXPECT_LE(kick_power, 1.0);
  });
}

TEST_F(KickerModelTest, WithoutBallPhysicsModel)
{
  // BallPhysicsModelが設定されていない場合の例外テスト
  EXPECT_THROW(kicker_model_->calculateKickPowerForStopDistance(2.0), std::runtime_error);
  EXPECT_THROW(kicker_model_->predictStopDistance(0.5), std::runtime_error);
}

// ===== 設定検証テスト =====

TEST_F(KickerModelTest, ValidateConfig)
{
  EXPECT_TRUE(kicker_model_->validateConfig());

  // 無効な設定のテスト
  KickerModel::Config invalid_config;
  invalid_config.straight_kick_powers = {};  // 空の配列
  invalid_config.straight_kick_speeds = {};

  KickerModel invalid_model(invalid_config);
  EXPECT_FALSE(invalid_model.validateConfig());
}

TEST_F(KickerModelTest, IsValidKickPower)
{
  EXPECT_TRUE(KickerModel::isValidKickPower(0.0));
  EXPECT_TRUE(KickerModel::isValidKickPower(0.5));
  EXPECT_TRUE(KickerModel::isValidKickPower(1.0));

  EXPECT_FALSE(KickerModel::isValidKickPower(-0.1));
  EXPECT_FALSE(KickerModel::isValidKickPower(1.1));
}

// ===== YAML読み込みテスト =====

TEST_F(KickerModelTest, YAMLLoadConfig)
{
  // テスト用YAML文字列
  std::string yaml_content = R"(
kicker_model:
  straight_kick_powers: [0.0, 0.3, 0.7, 1.0]
  straight_kick_speeds: [0.0, 1.5, 4.5, 7.0]
  chip_kick_powers: [0.0, 0.4, 0.8]
  chip_kick_distances: [0.0, 0.8, 1.8]
)";

  // 一時ファイルに書き込み
  std::string temp_file = "/tmp/test_kicker_config.yaml";
  std::ofstream file(temp_file);
  file << yaml_content;
  file.close();

  // YAML読み込みテスト
  EXPECT_NO_THROW({
    auto config = KickerModel::loadConfigFromYAML(temp_file);
    EXPECT_EQ(config.straight_kick_powers.size(), 4);
    EXPECT_EQ(config.straight_kick_speeds.size(), 4);
    EXPECT_EQ(config.chip_kick_powers.size(), 3);
    EXPECT_EQ(config.chip_kick_distances.size(), 3);

    EXPECT_DOUBLE_EQ(config.straight_kick_powers[1], 0.3);
    EXPECT_DOUBLE_EQ(config.straight_kick_speeds[3], 7.0);
  });

  // ファイル削除
  std::remove(temp_file.c_str());
}

TEST_F(KickerModelTest, YAMLFileNotFound)
{
  EXPECT_THROW(KickerModel::loadConfigFromYAML("/nonexistent/file.yaml"), std::runtime_error);
}

// ===== ファクトリー関数テスト =====

TEST_F(KickerModelTest, FactoryFunctions)
{
  auto default_model = createDefaultKickerModel();
  EXPECT_NE(default_model, nullptr);
  EXPECT_TRUE(default_model->validateConfig());
}

// ===== エッジケーステスト =====

TEST_F(KickerModelTest, EdgeCases)
{
  // 単一データポイントでの補間
  KickerModel::Config single_point_config;
  single_point_config.straight_kick_powers = {0.5};
  single_point_config.straight_kick_speeds = {3.0};
  single_point_config.chip_kick_powers = {0.5};
  single_point_config.chip_kick_distances = {1.0};

  KickerModel single_point_model(single_point_config);
  EXPECT_DOUBLE_EQ(single_point_model.predictStraightKickSpeed(0.0), 3.0);
  EXPECT_DOUBLE_EQ(single_point_model.predictStraightKickSpeed(1.0), 3.0);
}

}  // namespace crane
