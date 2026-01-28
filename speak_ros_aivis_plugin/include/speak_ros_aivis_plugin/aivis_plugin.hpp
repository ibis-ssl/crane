// Copyright 2025 ibis-ssl All rights reserved.
//
// Licensed under the MIT License.

#ifndef SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP
#define SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP

#include <speak_ros/speak_ros_plugin_base.hpp>

namespace aivis_plugin
{
class AivisPlugin : public speak_ros::SpeakROSPluginBase
{
public:
  AivisPlugin() : speak_ros::SpeakROSPluginBase() {}

  std::string getPluginName() const override { return "aivis_plugin"; }

  std::filesystem::path generateSoundFile(
    const std::string input_text, const std::filesystem::path output_directory,
    const std::string file_name) override;

  std::vector<speak_ros::Parameter> getParametersDefault() const override;

  void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
    override;

  int speaker;
  std::string host_name;
  int port;
  double speed_scale;
  double pitch_scale;
  double intonation_scale;
  double volume_scale;
  double tempo_dynamics_scale;
  double pre_phoneme_length;
  double post_phoneme_length;
  int output_sampling_rate;
  std::string output_stereo;
};
}  // namespace aivis_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aivis_plugin::AivisPlugin, speak_ros::SpeakROSPluginBase)

#endif  // SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP
