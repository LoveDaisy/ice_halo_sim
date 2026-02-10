#include <cstdio>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <variant>

#include "include/server.hpp"
#include "util/arg_parser.hpp"
#include "util/log.hpp"


struct SimResultHandler {
  void operator()(const icehalo::v3::NoneResult& /* r */) { LOG_INFO("<NoneResult>"); }

  void operator()(const icehalo::v3::RenderResult& r) {
    LOG_INFO("Renderer {:02d}: w = {}, h = {}, buffer = {:p}", r.renderer_id_, r.img_width_, r.img_height_,
             static_cast<const void*>(r.img_buffer_));
    char filename[32];
    std::snprintf(filename, 32, "img_%02d.jpg", r.renderer_id_);
    // Note: OpenCV Mat constructor requires non-const pointer, but we only read the data
    // The buffer is managed by Server and remains valid during this call
    cv::Mat mat(r.img_height_, r.img_width_, CV_8UC3, const_cast<uint8_t*>(r.img_buffer_));
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imwrite(filename, mat);
    // Alternative: Use CopyBuffer() for long-term storage:
    // auto buffer_copy = r.CopyBuffer();
    // cv::Mat mat_copy(r.img_height_, r.img_width_, CV_8UC3, buffer_copy.data());
  }

  void operator()(const icehalo::v3::StatsResult& r) {
    LOG_INFO("sim rays: {:.2f}M, crystals: {:.2f}M",  //
             r.sim_ray_num_ * 1.0 / 1e6, r.crystal_num_ * 1.0 / 1e6);
  }
};


using namespace std::chrono_literals;

int main(int argc, char** argv) {
  // Setup argument parser and parse arguments
  icehalo::ArgParser parser;
  parser.AddArgument("-v", 0, "verbose", "make output verbose");
  parser.AddArgument("-d", 0, "debug", "display debug info");
  parser.AddArgument("-f", 1, "config-file", "config file");
  icehalo::ArgParseResult arg_parse_result;
  try {
    arg_parse_result = parser.Parse(argc, argv);
  } catch (...) {
    return -1;
  }

  // Initialize logger with project pattern and set log level from args
  icehalo::InitLogger();
  if (arg_parse_result.count("-d")) {
    icehalo::SetLogLevel(spdlog::level::debug);
  } else if (arg_parse_result.count("-v")) {
    icehalo::SetLogLevel(spdlog::level::trace);
  }

  // Open config file
  const char* config_filename = arg_parse_result.at("-f")[0].c_str();

  // Setup simulation server
  icehalo::v3::Server s;
  auto err = s.CommitConfigFromFile(config_filename);
  if (err) {
    LOG_ERROR("Failed to commit configuration: {}", err.message);
    if (!err.field.empty()) {
      LOG_ERROR("Error field: {}", err.field);
    }
    return -1;
  }

  const auto kRefreshInterval = 1000ms;
  while (true) {
    std::this_thread::sleep_for(kRefreshInterval);

    // Get render results
    auto render_results = s.GetRenderResults();
    for (const auto& r : render_results) {
      SimResultHandler handler;
      handler(r);
    }

    // Get statistics result
    auto stats_result = s.GetStatsResult();
    if (stats_result.has_value()) {
      SimResultHandler handler;
      handler(stats_result.value());
    }

    if (s.IsIdle()) {
      s.Terminate();
      break;
    }
  }

  return 0;
}
