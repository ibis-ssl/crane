// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <crane_world_model_publisher/calibration/ball_calibration_data_extractor.hpp>
#include <crane_world_model_publisher/calibration/simple_ball_physics_optimizer.hpp>
#include <crane_world_model_publisher/ball_physics_model.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>

namespace crane
{

/**
 * @brief ボールモデルキャリブレーションノード
 */
class BallCalibrationNode : public rclcpp::Node
{
public:
  BallCalibrationNode() : Node("ball_calibration_node")
  {
    // パラメータの宣言
    this->declare_parameter("rosbag_path", "");
    this->declare_parameter("output_config_path", "");
    this->declare_parameter("auto_calibrate", false);
    
    // サービスサーバーの作成
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate_ball_physics", 
      std::bind(&BallCalibrationNode::calibrateCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // パブリッシャーの作成
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("calibration_status", 10);
    
    // 自動キャリブレーションの確認
    bool auto_calibrate = this->get_parameter("auto_calibrate").as_bool();
    if (auto_calibrate) {
      RCLCPP_INFO(this->get_logger(), "自動キャリブレーションを開始します");
      performCalibration();
    }
    
    RCLCPP_INFO(this->get_logger(), "ボールキャリブレーションノードが起動しました");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  
  BallCalibrationDataExtractor data_extractor_;
  SimpleBallPhysicsOptimizer physics_optimizer_;
  SimpleKickerCalibrator kicker_calibrator_;
  
  /**
   * @brief キャリブレーションサービスのコールバック
   */
  void calibrateCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // 未使用パラメータの警告回避
    
    RCLCPP_INFO(this->get_logger(), "キャリブレーションサービスが呼び出されました");
    
    try {
      bool success = performCalibration();
      response->success = success;
      
      if (success) {
        response->message = "キャリブレーションが正常に完了しました";
      } else {
        response->message = "キャリブレーションに失敗しました";
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "キャリブレーション中にエラーが発生: %s", e.what());
      response->success = false;
      response->message = std::string("エラー: ") + e.what();
    }
  }
  
  /**
   * @brief キャリブレーション実行
   */
  bool performCalibration()
  {
    publishStatus("キャリブレーション開始");
    
    // ROSBAGパスの取得
    std::string bag_path = this->get_parameter("rosbag_path").as_string();
    if (bag_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "ROSBAGパスが指定されていません");
      publishStatus("エラー: ROSBAGパスが未指定");
      return false;
    }
    
    if (!std::filesystem::exists(bag_path)) {
      RCLCPP_ERROR(this->get_logger(), "ROSBAGファイルが存在しません: %s", bag_path.c_str());
      publishStatus("エラー: ROSBAGファイルが見つからない");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "ROSBAGファイルを解析中: %s", bag_path.c_str());
    publishStatus("データ抽出中...");
    
    // データ抽出器の設定
    BallCalibrationDataExtractor::ExtractorConfig extractor_config;
    extractor_config.min_kick_speed = 0.8;  // 最小キック速度
    extractor_config.min_trajectory_duration = 0.5;  // 最小軌道継続時間
    extractor_config.min_trajectory_points = 8;  // 最小軌道点数
    extractor_config.extract_straight_kicks_only = true;  // ストレートキックのみ
    data_extractor_.setConfig(extractor_config);
    
    // データ抽出
    std::vector<KickDataPoint> kick_data;
    try {
      kick_data = data_extractor_.extractKickDataFromBag(bag_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "データ抽出に失敗: %s", e.what());
      publishStatus("データ抽出エラー");
      return false;
    }
    
    const auto & stats = data_extractor_.getLastExtractionStats();
    RCLCPP_INFO(this->get_logger(), 
                "データ抽出完了: %zu個のキックイベントを検出 (%zu個が有効)",
                stats.total_kick_events, stats.valid_kick_events);
    
    if (kick_data.empty()) {
      RCLCPP_ERROR(this->get_logger(), "有効なキックデータが見つかりません");
      publishStatus("エラー: キックデータなし");
      return false;
    }
    
    publishStatus("物理パラメータ最適化中...");
    
    // 物理パラメータの最適化
    SimpleBallPhysicsOptimizer::OptimizerConfig optimizer_config;
    optimizer_config.min_deceleration = 0.1;
    optimizer_config.max_deceleration = 1.5;
    optimizer_config.convergence_threshold = 1e-6;
    optimizer_config.max_iterations = 100;
    physics_optimizer_.setConfig(optimizer_config);
    
    auto physics_result = physics_optimizer_.optimizeDecelerationParameter(kick_data);
    
    if (!physics_result.success) {
      RCLCPP_ERROR(this->get_logger(), "物理パラメータの最適化に失敗");
      publishStatus("エラー: 物理パラメータ最適化失敗");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(),
                "物理パラメータ最適化完了: 減速度=%.4f m/s², RMSE=%.4f m, R²=%.3f",
                physics_result.optimized_deceleration, 
                physics_result.residual_error,
                physics_result.r_squared);
    
    publishStatus("キッカーモデル最適化中...");
    
    // キッカーモデルの最適化
    auto kicker_result = kicker_calibrator_.calibrateKickerModel(kick_data);
    
    if (!kicker_result.success) {
      RCLCPP_ERROR(this->get_logger(), "キッカーモデルの最適化に失敗");
      publishStatus("エラー: キッカーモデル最適化失敗");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(),
                "キッカーモデル最適化完了: 線形係数=%.2f, オフセット=%.2f, R²=%.3f",
                kicker_result.straight_kick_model.linear_coefficient,
                kicker_result.straight_kick_model.offset,
                kicker_result.straight_kick_model.r_squared);
    
    publishStatus("設定ファイル出力中...");
    
    // 設定ファイルの出力
    bool save_success = saveCalibrationResults(physics_result, kicker_result);
    if (!save_success) {
      RCLCPP_ERROR(this->get_logger(), "設定ファイルの保存に失敗");
      publishStatus("エラー: 設定ファイル保存失敗");
      return false;
    }
    
    publishStatus("キャリブレーション完了");
    RCLCPP_INFO(this->get_logger(), "キャリブレーションが正常に完了しました");
    
    return true;
  }
  
  /**
   * @brief キャリブレーション結果の保存
   */
  bool saveCalibrationResults(
    const SimpleBallPhysicsOptimizer::OptimizationResult & physics_result,
    const SimpleKickerCalibrator::CalibrationResult & kicker_result)
  {
    // 出力パスの取得
    std::string output_path = this->get_parameter("output_config_path").as_string();
    if (output_path.empty()) {
      output_path = "calibrated_ball_physics.yaml";
    }
    
    try {
      YAML::Node config;
      
      // 物理パラメータ
      config["ball_physics_model"]["deceleration"] = physics_result.optimized_deceleration;
      config["ball_physics_model"]["gravity"] = -9.81;  // 固定値
      config["ball_physics_model"]["air_resistance"] = 0.0;  // 固定値
      config["ball_physics_model"]["height_threshold"] = 0.05;  // 固定値
      config["ball_physics_model"]["speed_threshold"] = 0.1;  // 固定値
      config["ball_physics_model"]["stop_threshold"] = 0.05;  // 固定値
      
      // キッカーモデル
      config["kicker_power_mapping"]["straight_kick"]["linear_coefficient"] = 
        kicker_result.straight_kick_model.linear_coefficient;
      config["kicker_power_mapping"]["straight_kick"]["offset"] = 
        kicker_result.straight_kick_model.offset;
      config["kicker_power_mapping"]["straight_kick"]["r_squared"] = 
        kicker_result.straight_kick_model.r_squared;
      config["kicker_power_mapping"]["straight_kick"]["data_points_used"] = 
        static_cast<int>(kicker_result.straight_kick_model.data_points_used);
      
      // キャリブレーション情報
      config["calibration_info"]["timestamp"] = 
        std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      config["calibration_info"]["physics_rmse"] = physics_result.residual_error;
      config["calibration_info"]["physics_r_squared"] = physics_result.r_squared;
      config["calibration_info"]["data_points_used"] = static_cast<int>(physics_result.data_points_used);
      
      // ファイル出力
      std::ofstream file_stream(output_path);
      if (!file_stream.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "出力ファイルを開けません: %s", output_path.c_str());
        return false;
      }
      
      file_stream << "# ボール物理モデル キャリブレーション結果\n";
      file_stream << "# 生成日時: " << 
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << "\n";
      file_stream << "# RMSE: " << physics_result.residual_error << " m\n";
      file_stream << "# R²: " << physics_result.r_squared << "\n\n";
      
      file_stream << config;
      file_stream.close();
      
      RCLCPP_INFO(this->get_logger(), "設定ファイルを保存しました: %s", output_path.c_str());
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "YAML出力エラー: %s", e.what());
      return false;
    }
    
    return true;
  }
  
  /**
   * @brief ステータスメッセージの配信
   */
  void publishStatus(const std::string & status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "ステータス: %s", status.c_str());
  }
};

} // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<crane::BallCalibrationNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "ボールキャリブレーションノードを開始");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "実行中にエラーが発生: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}