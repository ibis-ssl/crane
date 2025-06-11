// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_RECEIVER__ROBOT_ERRORS_HPP_
#define CRANE_ROBOT_RECEIVER__ROBOT_ERRORS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
// エラーコード定義
namespace error_codes
{
// POWERエラー定義
enum PowerErrorCode {
  POWER_NONE = 0,
  POWER_UNDER_VOLTAGE = 0x0001,
  POWER_OVER_VOLTAGE = 0x0002,
  POWER_OVER_CURRENT = 0x0004,
  POWER_SHORT_CIRCUIT = 0x0008,
  POWER_CHARGE_TIME = 0x0010,
  POWER_CHARGE_POWER = 0x0020,
  POWER_DISCHARGE = 0x0040,
  POWER_PARAMETER = 0x0080,
  POWER_COMMAND = 0x0100,
  POWER_NO_CAP = 0x0200,
  POWER_DISCHARGE_FAIL = 0x0400,
  POWER_GD_POWER_FAIL = 0x0800,
  POWER_COIL_OVER_HEAT = 0x1000,
  POWER_FET_OVER_HEAT = 0x2000
};

// BLDCエラー定義
enum BldcErrorCode {
  BLDC_NONE = 0,
  BLDC_UNDER_VOLTAGE = 0x0001,
  BLDC_OVER_CURRENT = 0x0002,
  BLDC_MOTOR_OVER_HEAT = 0x0004,
  BLDC_OVER_LOAD = 0x0008,
  BLDC_ENC_ERROR = 0x0010,
  BLDC_OVER_VOLTAGE = 0x0020,
  BLDC_FET_OVER_HEAT = 0x0040
};

// エラーレベル
constexpr int OK = 0;
constexpr int WARN = 1;
constexpr int ERROR = 2;
constexpr int STALE = 3;

}  // namespace error_codes

// ロボットの状態
enum class RobotState {
  ACTIVE,    // アクティブで診断情報を発行すべき
  INACTIVE,  // 一時的に非アクティブ（フィールド外など）
};

namespace utils
{
// BLDCモーターの名前を取得する関数
inline auto getBldcName(uint16_t id) -> std::string
{
  switch (id) {
    case 0:
      return "BLDC-RF";  // Right Front
    case 1:
      return "BLDC-RB";  // Right Back
    case 2:
      return "BLDC-LB";  // Left Back
    case 3:
      return "BLDC-LF";  // Left Front
    default:
      return "BLDC-" + std::to_string(id);
  }
}

// エラー情報をテキストに変換する関数
inline auto convertErrorDataToStr(uint16_t id, uint16_t info) -> std::string
{
  std::string result;
  if (id == 100) {  // POWERエラー
    result = "POWER : ";
    switch (info) {
      case error_codes::POWER_NONE:
        result += "no error";
        break;
      case error_codes::POWER_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case error_codes::POWER_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case error_codes::POWER_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case error_codes::POWER_SHORT_CIRCUIT:
        result += "SHORT_CIRCUIT";
        break;
      case error_codes::POWER_CHARGE_TIME:
        result += "CHARGE_TIME";
        break;
      case error_codes::POWER_CHARGE_POWER:
        result += "CHARGE_POWER";
        break;
      case error_codes::POWER_DISCHARGE:
        result += "DISCHARGE";
        break;
      case error_codes::POWER_PARAMETER:
        result += "PARAMETER";
        break;
      case error_codes::POWER_COMMAND:
        result += "COMMAND";
        break;
      case error_codes::POWER_NO_CAP:
        result += "NO_CAP";
        break;
      case error_codes::POWER_DISCHARGE_FAIL:
        result += "DISCHARGE_FAIL";
        break;
      case error_codes::POWER_GD_POWER_FAIL:
        result += "GD_POWER_FAIL";
        break;
      case error_codes::POWER_COIL_OVER_HEAT:
        result += "COIL_OVER_HEAT";
        break;
      case error_codes::POWER_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  } else {  // BLDCエラー
    result = getBldcName(id) + " : ";

    if (id > 3) {
      return result + "unknown id : " + std::to_string(id) + " info : " + std::to_string(info);
    }

    switch (info) {
      case error_codes::BLDC_NONE:
        result += "no error";
        break;
      case error_codes::BLDC_UNDER_VOLTAGE:
        result += "UNDER_VOLTAGE";
        break;
      case error_codes::BLDC_OVER_CURRENT:
        result += "OVER_CURRENT";
        break;
      case error_codes::BLDC_MOTOR_OVER_HEAT:
        result += "MOTOR_OVER_HEAT";
        break;
      case error_codes::BLDC_OVER_LOAD:
        result += "OVER_LOAD";
        break;
      case error_codes::BLDC_ENC_ERROR:
        result += "ENC_ERROR";
        break;
      case error_codes::BLDC_OVER_VOLTAGE:
        result += "OVER_VOLTAGE";
        break;
      case error_codes::BLDC_FET_OVER_HEAT:
        result += "FET_OVER_HEAT";
        break;
      default:
        result += "unknown info " + std::to_string(info);
        break;
    }
  }
  return result;
}

// temperature配列のインデックスから名前への変換
inline auto getTemperatureLabel(int index) -> std::string
{
  switch (index) {
    case 0:
      return "Motor 1";
    case 1:
      return "Motor 2";
    case 2:
      return "Motor 3";
    case 3:
      return "Motor 4";
    case 4:
      return "FET";
    case 5:
      return "Coil 1";
    case 6:
      return "Coil 2";
    default:
      return "Unknown (" + std::to_string(index) + ")";
  }
}

// エラーレベルに対応する色を取得
inline auto getColorForErrorLevel(int level) -> std::string
{
  switch (level) {
    case error_codes::WARN:
      return "yellow";
    case error_codes::ERROR:
      return "red";
    case error_codes::STALE:
      return "grey";
    default:
      return "white";
  }
}
}  // namespace utils

// エラー情報の構造体
struct ErrorInfo
{
  std::string type;        // エラータイプ (robot_error, communication, battery など)
  std::string message;     // エラーメッセージ
  int level;               // エラーレベル (0: OK, 1: WARN, 2: ERROR, 3: STALE)
  rclcpp::Time timestamp;  // エラーが発生した時刻
};
}  // namespace crane
#endif  // CRANE_ROBOT_RECEIVER__ROBOT_ERRORS_HPP_
