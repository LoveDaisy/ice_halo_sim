#include <cstdio>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <variant>

#include "include/arg_parser.hpp"
#include "include/log.hpp"
#include "include/server.hpp"


struct SimResultHandler {
  void operator()(const icehalo::v3::NoneResult& /* r */) { LOG_INFO("<NoneResult>"); }

  void operator()(const icehalo::v3::RenderResult& r) {
    LOG_INFO("Renderer %02d: w = %d, h = %d, buffer = %p", r.renderer_id_, r.img_width_, r.img_height_, r.img_buffer_);
    char filename[32];
    std::snprintf(filename, 32, "img_%02d.jpg", r.renderer_id_);
    cv::Mat mat(r.img_height_, r.img_width_, CV_8UC3, r.img_buffer_);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imwrite(filename, mat);
  }

  void operator()(const icehalo::v3::StatsResult& r) {
    LOG_INFO("sim rays: %.2fM, crystals: %.2fM",  //
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

  // Setup log levels
  if (arg_parse_result.count("-d")) {
    icehalo::LogFilterPtr stdout_filter =
        icehalo::LogFilter::MakeLevelFilter({ icehalo::LogLevel::kDebug, icehalo::LogLevel::kVerbose });
    icehalo::LogDestPtr stdout_dest = icehalo::LogStdOutDest::GetInstance();
    icehalo::Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  } else if (arg_parse_result.count("-v")) {
    icehalo::LogFilterPtr stdout_filter = icehalo::LogFilter::MakeLevelFilter({ icehalo::LogLevel::kVerbose });
    icehalo::LogDestPtr stdout_dest = icehalo::LogStdOutDest::GetInstance();
    icehalo::Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  }

  // Open config file
  const char* config_filename = arg_parse_result.at("-f")[0].c_str();
  std::ifstream config_file(config_filename);

  // Setup simulation server
  icehalo::v3::Server s;
  s.CommitConfig(config_file);

  const auto kRefreshInterval = 1000ms;
  while (true) {
    std::this_thread::sleep_for(kRefreshInterval);
    auto res = s.GetResults();
    if (res.empty()) {
      continue;
    }

    for (const auto& r : res) {
      std::visit(SimResultHandler{}, r);
    }
    if (s.IsIdle()) {
      s.Terminate();
      break;
    }
  }

  return 0;
}