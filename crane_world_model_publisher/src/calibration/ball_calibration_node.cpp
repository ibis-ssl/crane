// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <crane_world_model_publisher/ball_physics_model.hpp>
#include <crane_world_model_publisher/calibration/ball_calibration_data_extractor.hpp>
#include <crane_world_model_publisher/calibration/simple_ball_physics_optimizer.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    this->declare_parameter("kick_power_analysis_output", "");
    this->declare_parameter("auto_calibrate", false);

    // サービスサーバーの作成
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate_ball_physics", std::bind(
                                  &BallCalibrationNode::calibrateCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));

    // パブリッシャーの作成
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("calibration_status", 10);

    // 自動キャリブレーションの確認
    bool auto_calibrate = this->get_parameter("auto_calibrate").as_bool();
    if (auto_calibrate) {
      RCLCPP_INFO(this->get_logger(), "自動キャリブレーションを開始します");
      bool success = performCalibration();
      if (success) {
        RCLCPP_INFO(this->get_logger(), "自動キャリブレーション完了。ノードを終了します");
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "自動キャリブレーションに失敗しました。ノードを終了します");
      }
      // 成功・失敗に関わらず自動キャリブレーション後は終了
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "ボールキャリブレーションノードが起動しました（サービスモード）");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  SimpleBallPhysicsOptimizer physics_optimizer_;
  BallCalibrationDataExtractor data_extractor_;

  /**
   * @brief キャリブレーションサービスのコールバック
   */
  void calibrateCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // 未使用パラメータの警告回避

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
    publishStatus("JSONベースキャリブレーション開始");

    // ROSBAGパスの取得
    std::string rosbag_path = this->get_parameter("rosbag_path").as_string();
    if (rosbag_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "ROSBAGパスが指定されていません");
      publishStatus("エラー: ROSBAGパスが未指定");
      return false;
    }

    if (!std::filesystem::exists(rosbag_path)) {
      RCLCPP_ERROR(this->get_logger(), "ROSBAGディレクトリが存在しません: %s", rosbag_path.c_str());
      publishStatus("エラー: ROSBAGディレクトリが見つからない");
      return false;
    }

    // ROSBAGパスからJSONディレクトリパスを自動生成
    std::string json_dir_path = rosbag_path + "/ball_calibration_analysis";

    // JSONディレクトリが存在しない場合、ROSBAGを処理してJSONデータを生成
    if (!std::filesystem::exists(json_dir_path)) {
      RCLCPP_INFO(
        this->get_logger(),
        "ball_calibration_"
        "analysisディレクトリが存在しません。ROSBAGを処理してJSONデータを生成します: %s",
        json_dir_path.c_str());
      publishStatus("ROSBAGからJSONデータ生成中...");

      // JSONディレクトリを作成
      try {
        std::filesystem::create_directories(json_dir_path);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "JSONディレクトリの作成に失敗: %s", e.what());
        publishStatus("エラー: JSONディレクトリ作成失敗");
        return false;
      }

      // ROSBAGからキックデータを抽出してJSONに変換
      bool extraction_success = processROSBAGToJSON(rosbag_path, json_dir_path);
      if (!extraction_success) {
        RCLCPP_ERROR(this->get_logger(), "ROSBAGからのJSON生成に失敗しました");
        publishStatus("エラー: ROSBAG処理失敗");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "ROSBAGからJSONデータの生成が完了しました");
      publishStatus("JSONデータ生成完了");
    }

    // JSONファイルの存在確認
    std::filesystem::path json_dir(json_dir_path);
    auto json_files = std::filesystem::directory_iterator(json_dir);
    bool has_json_files = false;
    for (const auto & entry : json_files) {
      if (
        entry.path().filename().string().find("kick_event_visualization_") == 0 &&
        entry.path().extension() == ".json") {
        has_json_files = true;
        break;
      }
    }

    if (!has_json_files) {
      RCLCPP_ERROR(
        this->get_logger(), "kick_event_visualization_*_data.jsonファイルが見つかりません: %s",
        json_dir_path.c_str());
      publishStatus("エラー: JSONデータファイルが存在しない");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "ROSBAG: %s -> JSONディレクトリ: %s", rosbag_path.c_str(),
      json_dir_path.c_str());
    publishStatus("JSONデータ読み込み中...");

    // 最適化設定
    SimpleBallPhysicsOptimizer::OptimizationConfig optimizer_config;
    optimizer_config.json_directory_path = json_dir_path;
    optimizer_config.min_trajectory_duration = 0.5;
    optimizer_config.velocity_outlier_threshold = 2.0;
    optimizer_config.min_data_points_per_trajectory = 10;
    optimizer_config.min_fitting_r_squared = 0.3;
    optimizer_config.min_deceleration = 0.1;
    optimizer_config.max_deceleration = 2.0;

    publishStatus("グローバル減速度パラメータ最適化中...");

    // JSONベース最適化実行
    auto optimization_result = physics_optimizer_.optimizeFromJSONDirectory(optimizer_config);

    if (!optimization_result.success) {
      RCLCPP_ERROR(this->get_logger(), "最適化に失敗しました");
      publishStatus("エラー: 最適化失敗");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "最適化完了: 減速度=%.4f m/s², RMSE=%.4f, R²=%.3f, キックデータ=%zu個",
      optimization_result.global_deceleration, optimization_result.global_rmse,
      optimization_result.global_r_squared, optimization_result.kick_data.size());

    publishStatus("設定ファイル出力中...");

    // 設定ファイルの出力
    bool save_success = saveCalibrationResults(optimization_result);
    if (!save_success) {
      RCLCPP_ERROR(this->get_logger(), "設定ファイルの保存に失敗");
      publishStatus("エラー: 設定ファイル保存失敗");
      return false;
    }

    // キックパワー分析結果の出力
    std::string kick_power_output = this->get_parameter("kick_power_analysis_output").as_string();
    if (kick_power_output.empty()) {
      // デフォルト出力パスを自動生成
      kick_power_output = json_dir_path + "/kick_power_velocity_analysis.json";
    }

    publishStatus("キックパワー分析データ出力中...");
    bool export_success =
      physics_optimizer_.exportKickPowerAnalysis(kick_power_output, optimization_result);
    if (!export_success) {
      RCLCPP_WARN(this->get_logger(), "キックパワー分析データの出力に失敗");
    } else {
      RCLCPP_INFO(this->get_logger(), "キックパワー分析結果を出力: %s", kick_power_output.c_str());
    }

    publishStatus("キャリブレーション完了");

    // crane.launch.pyで使用できる形式で標準出力に出力
    outputLaunchFileArrays(optimization_result);

    RCLCPP_INFO(this->get_logger(), "JSONベースキャリブレーションが正常に完了しました");

    return true;
  }

  /**
   * @brief キャリブレーション結果の保存
   */
  bool saveCalibrationResults(
    const SimpleBallPhysicsOptimizer::OptimizationResult & optimization_result)
  {
    // 出力パスの取得
    std::string output_path = this->get_parameter("output_config_path").as_string();
    if (output_path.empty()) {
      output_path = "calibrated_ball_physics.yaml";
    }

    try {
      // 出力ディレクトリの存在確認と作成
      std::filesystem::path output_file_path(output_path);
      std::filesystem::path output_dir = output_file_path.parent_path();

      if (!output_dir.empty() && !std::filesystem::exists(output_dir)) {
        std::filesystem::create_directories(output_dir);
        RCLCPP_INFO(this->get_logger(), "出力ディレクトリを作成しました: %s", output_dir.c_str());
      }

      YAML::Node config;

      // 物理パラメータ
      config["ball_physics_model"]["deceleration"] = optimization_result.global_deceleration;
      config["ball_physics_model"]["gravity"] = -9.81;          // 固定値
      config["ball_physics_model"]["air_resistance"] = 0.0;     // 固定値
      config["ball_physics_model"]["height_threshold"] = 0.05;  // 固定値
      config["ball_physics_model"]["speed_threshold"] = 0.1;    // 固定値
      config["ball_physics_model"]["stop_threshold"] = 0.05;    // 固定値

      // キッカーパワー別速度マッピング（0.0-1.0を0.1刻み）
      std::vector<double> target_powers = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
      for (double target_power : target_powers) {
        std::vector<double> velocities_for_power;
        for (const auto & kick : optimization_result.kick_data) {
          if (!kick.is_chip_kick && std::abs(kick.kick_power - target_power) < 0.05) {
            velocities_for_power.push_back(kick.estimated_initial_velocity);
          }
        }

        if (!velocities_for_power.empty()) {
          double mean_velocity =
            std::accumulate(velocities_for_power.begin(), velocities_for_power.end(), 0.0) /
            velocities_for_power.size();
          std::string power_key = "power_" + std::to_string(static_cast<int>(target_power * 100));
          config["kicker_power_mapping"]["straight_kick"][power_key]["mean_velocity"] =
            mean_velocity;
          config["kicker_power_mapping"]["straight_kick"][power_key]["sample_count"] =
            static_cast<int>(velocities_for_power.size());
        }
      }

      // キャリブレーション情報
      config["calibration_info"]["timestamp"] =
        std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
      config["calibration_info"]["physics_rmse"] = optimization_result.global_rmse;
      config["calibration_info"]["physics_r_squared"] = optimization_result.global_r_squared;
      config["calibration_info"]["trajectories_analyzed"] =
        static_cast<int>(optimization_result.trajectories_analyzed);
      config["calibration_info"]["trajectories_used"] =
        static_cast<int>(optimization_result.trajectories_used);

      // ファイル出力
      std::ofstream file_stream(output_path);
      if (!file_stream.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "出力ファイルを開けません: %s", output_path.c_str());
        return false;
      }

      file_stream << "# ボール物理モデル JSONベースキャリブレーション結果\n";
      file_stream << "# 生成日時: "
                  << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << "\n";
      file_stream << "# グローバル減速度: " << optimization_result.global_deceleration << " m/s²\n";
      file_stream << "# RMSE: " << optimization_result.global_rmse << "\n";
      file_stream << "# R²: " << optimization_result.global_r_squared << "\n";
      file_stream << "# 分析軌道数: " << optimization_result.trajectories_analyzed << "\n";
      file_stream << "# 有効軌道数: " << optimization_result.trajectories_used << "\n\n";

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
   * @brief ROSBAGからJSONデータを生成
   * @param rosbag_path ROSBAGディレクトリパス
   * @param json_output_dir JSON出力ディレクトリパス
   * @return 成功フラグ
   */
  bool processROSBAGToJSON(const std::string & rosbag_path, const std::string & json_output_dir)
  {
    try {
      // データ抽出設定
      BallCalibrationDataExtractor::ExtractorConfig extractor_config;
      extractor_config.min_kick_speed = 0.5;
      extractor_config.max_kick_speed = 30.0;
      extractor_config.min_trajectory_points = 10;
      extractor_config.extract_straight_kicks_only = true;

      data_extractor_.setConfig(extractor_config);

      RCLCPP_INFO(this->get_logger(), "ROSBAGからキックデータを抽出中: %s", rosbag_path.c_str());

      // ROSBAGからキックデータを抽出
      auto kick_data_points = data_extractor_.extractKickDataFromBag(rosbag_path);

      if (kick_data_points.empty()) {
        RCLCPP_WARN(this->get_logger(), "有効なキックデータが見つかりませんでした");
        return false;
      }

      RCLCPP_INFO(
        this->get_logger(), "%zu個のキックデータポイントを抽出しました", kick_data_points.size());

      // 統計情報の取得
      auto stats = data_extractor_.getLastExtractionStats();
      RCLCPP_INFO(
        this->get_logger(),
        "抽出統計: 総イベント数=%zu, 有効イベント数=%zu, ストレートキック=%zu, チップキック=%zu",
        stats.total_kick_events, stats.valid_kick_events, stats.straight_kick_count,
        stats.chip_kick_count);

      // 可視化用JSONデータとPythonスクリプトを生成
      publishStatus("可視化データ生成中...");

      // ボールデータを準備（extractKickDataFromBag内部で処理されるため、ここでは簡略化）
      std::vector<std::pair<rclcpp::Time, Ball>> ball_data;

      // 可視化データの生成（キック力情報付き）
      data_extractor_.visualizeKickEventsWithPower(
        ball_data, kick_data_points, "kick_event_visualization", rosbag_path);

      RCLCPP_INFO(
        this->get_logger(), "JSON可視化データをディレクトリに出力しました: %s",
        json_output_dir.c_str());

      return true;

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "ROSBAG処理中にエラーが発生: %s", e.what());
      return false;
    }
  }

  /**
   * @brief crane.launch.pyで使用できる配列形式を標準出力に出力
   */
  void outputLaunchFileArrays(
    const SimpleBallPhysicsOptimizer::OptimizationResult & optimization_result)
  {
    // 0.0-1.0を0.1刻みで設定
    std::vector<double> target_powers = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    std::vector<double> measured_velocities;

    // 各パワー値での平均速度を計算
    for (double target_power : target_powers) {
      std::vector<double> velocities_for_power;
      for (const auto & kick : optimization_result.kick_data) {
        if (!kick.is_chip_kick && std::abs(kick.kick_power - target_power) < 0.05) {
          velocities_for_power.push_back(kick.estimated_initial_velocity);
        }
      }

      if (!velocities_for_power.empty()) {
        double mean_velocity =
          std::accumulate(velocities_for_power.begin(), velocities_for_power.end(), 0.0) /
          velocities_for_power.size();
        measured_velocities.push_back(mean_velocity);
      } else {
        // データがない場合は0.0で埋める
        measured_velocities.push_back(0.0);
      }
    }

    // crane.launch.pyで使用できる形式で出力
    std::cout << "\n==================================================\n";
    std::cout << "crane.launch.py用キャリブレーション結果\n";
    std::cout << "==================================================\n";
    std::cout << "以下の値をcrane.launch.pyのL198-199に貼り付けてください:\n\n";

    // パワー配列（変更なし）
    std::cout << "                            {\"straight_kick_power_array\": [";
    for (size_t i = 0; i < target_powers.size(); ++i) {
      std::cout << target_powers[i];
      if (i < target_powers.size() - 1) std::cout << ", ";
    }
    std::cout << "]},\n";

    // 測定された速度配列
    std::cout << "                            {\"straight_kick_speed_array\": [";
    for (size_t i = 0; i < measured_velocities.size(); ++i) {
      std::cout << std::fixed << std::setprecision(1) << measured_velocities[i];
      if (i < measured_velocities.size() - 1) std::cout << ", ";
    }
    std::cout << "]},\n\n";

    // 追加情報
    std::cout << "測定結果詳細:\n";
    for (size_t i = 0; i < target_powers.size(); ++i) {
      size_t sample_count = 0;
      for (const auto & kick : optimization_result.kick_data) {
        if (!kick.is_chip_kick && std::abs(kick.kick_power - target_powers[i]) < 0.05) {
          sample_count++;
        }
      }
      std::cout << "  パワー " << std::fixed << std::setprecision(2) << target_powers[i]
                << " -> 速度 " << std::setprecision(1) << measured_velocities[i]
                << " m/s (サンプル数: " << sample_count << ")\n";
    }
    std::cout << "\n減速度パラメータ: " << std::setprecision(3)
              << optimization_result.global_deceleration << " m/s²\n";
    std::cout << "==================================================\n\n";
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

}  // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<crane::BallCalibrationNode>();

  // 自動キャリブレーションが完了してシャットダウンされた場合はここで終了
  if (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "ボールキャリブレーションノード開始（サービスモード）");
    try {
      rclcpp::spin(node);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "実行中にエラーが発生: %s", e.what());
    }
  }

  rclcpp::shutdown();
  return 0;
}
