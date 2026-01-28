// Copyright 2025 ibis-ssl All rights reserved.
//
// Licensed under the MIT License.

#include <cpprest/filestream.h>
#include <cpprest/http_client.h>

#include "speak_ros_aivis_plugin/aivis_plugin.hpp"

std::filesystem::path aivis_plugin::AivisPlugin::generateSoundFile(
  const std::string input_text, const std::filesystem::path output_directory,
  const std::string file_name)
{
  updateParameters();

  std::string generated_file_path = output_directory / (file_name + ".wav");

  std::string base_url = "http://" + host_name + ":" + std::to_string(port);

  auto audio_query_task =
    pplx::create_task([=] {
      web::http::client::http_client_config config;
      web::http::client::http_client client(base_url, config);
      web::json::value body;

      body[U("accent_phrases")] = web::json::value::array();
      body[U("speedScale")] = web::json::value::number(speed_scale);
      body[U("pitchScale")] = web::json::value::number(pitch_scale);
      body[U("intonationScale")] = web::json::value::number(intonation_scale);
      body[U("volumeScale")] = web::json::value::number(volume_scale);
      body[U("tempoDynamicsScale")] = web::json::value::number(tempo_dynamics_scale);
      body[U("prePhonemeLength")] = web::json::value::number(pre_phoneme_length);
      body[U("postPhonemeLength")] = web::json::value::number(post_phoneme_length);
      body[U("outputSamplingRate")] = web::json::value::number(output_sampling_rate);
      body[U("outputStereo")] = web::json::value::string(output_stereo);

      return client.request(
        web::http::methods::POST,
        web::http::uri_builder(U("/audio_query"))
          .append_query(U("text"), input_text)
          .append_query(U("speaker"), speaker)
          .to_string(),
        web::json::value().serialize(), U("application/json"));
    }).then([](web::http::http_response response) {
      if (response.status_code() == web::http::status_codes::OK) {
        std::cout << "audio_query succeeded" << std::endl;
        std::cout << response.to_string() << std::endl;
        return response;
      }
      throw std::runtime_error("network error");
    });

  audio_query_task.wait();
  auto audio_query_response = audio_query_task.get();

  using concurrency::streams::ostream;
  auto file_stream = std::make_shared<ostream>();

  pplx::task<void> synthesis_task =
    concurrency::streams::fstream::open_ostream(generated_file_path)
      .then([&](ostream out_file) {
        *file_stream = out_file;

        web::json::value body = audio_query_response.extract_json().get();
        body[U("speedScale")] = web::json::value::number(speed_scale);
        body[U("pitchScale")] = web::json::value::number(pitch_scale);
        body[U("intonationScale")] = web::json::value::number(intonation_scale);
        body[U("volumeScale")] = web::json::value::number(volume_scale);
        body[U("tempoDynamicsScale")] = web::json::value::number(tempo_dynamics_scale);
        body[U("prePhonemeLength")] = web::json::value::number(pre_phoneme_length);
        body[U("postPhonemeLength")] = web::json::value::number(post_phoneme_length);
        body[U("outputSamplingRate")] = web::json::value::number(output_sampling_rate);
        body[U("outputStereo")] = web::json::value::string(output_stereo);

        web::http::client::http_client_config config;
        web::http::client::http_client client(base_url, config);
        return client.request(
          web::http::methods::POST,
          web::http::uri_builder(U("/synthesis")).append_query(U("speaker"), speaker).to_string(),
          body.serialize(), "application/json");
      })
      .then([&](web::http::http_response response) {
        if (response.status_code() == web::http::status_codes::OK) {
          // write out the audio
          return response.body().read_to_end(file_stream->streambuf());
        }
        throw std::runtime_error("network error");
      })
      .then([=](size_t) {
        // close the file stream
        return file_stream->close();
      });

  synthesis_task.wait();

  return std::filesystem::path(generated_file_path);
}

std::vector<speak_ros::Parameter> aivis_plugin::AivisPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"speaker", "[number/integer] aivis speaker id", 888753760},
      {"host_name", "[string] host of aivis engine", "localhost"},
      {"port", "[number/integer] port of aivis engine", 10101},
      {"speedScale", "[number] voice speed", 1.0},
      {"pitchScale", "[number] voice pitch", 0.0},
      {"intonationScale", "[number] emotional expression (0.0-2.0)", 1.0},
      {"volumeScale", "[number] volume scale", 1.0},
      {"tempoDynamicsScale", "[number] speech speed fluctuation (0.0-2.0)", 1.0},
      {"prePhonemeLength", "[number] pre phoneme length [sec]", 0.1},
      {"postPhonemeLength", "[number] post phoneme length [sec]", 0.1},
      {"outputSamplingRate", "[number] output sampling rate [Hz]", 44100},
      {"outputStereo", "[bool] output stereo", "false"}
    // clang-format on
  };
}

void aivis_plugin::AivisPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  speaker = std::get<int>(parameters.at("speaker"));
  host_name = std::get<std::string>(parameters.at("host_name"));
  port = std::get<int>(parameters.at("port"));
  speed_scale = std::get<double>(parameters.at("speedScale"));
  pitch_scale = std::get<double>(parameters.at("pitchScale"));
  intonation_scale = std::get<double>(parameters.at("intonationScale"));
  volume_scale = std::get<double>(parameters.at("volumeScale"));
  tempo_dynamics_scale = std::get<double>(parameters.at("tempoDynamicsScale"));
  pre_phoneme_length = std::get<double>(parameters.at("prePhonemeLength"));
  post_phoneme_length = std::get<double>(parameters.at("postPhonemeLength"));
  output_sampling_rate = std::get<int>(parameters.at("outputSamplingRate"));
  output_stereo = std::get<std::string>(parameters.at("outputStereo"));
}
