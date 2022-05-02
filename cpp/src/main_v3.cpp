#include <fstream>
#include <thread>
#include <variant>

#include "include/result.hpp"
#include "include/server.hpp"
#include "util/arg_parser.hpp"
#include "util/log.hpp"


struct SimResultHandler {
  void operator()(const icehalo::v3::NoneResult& /* r */) { LOG_INFO("<NoneResult>"); }
  void operator()(const icehalo::v3::RenderResult& r) {
    LOG_INFO("<RenderResult>: w = %d, h = %d, buffer = %p", r.img_width_, r.img_height_, r.img_buffer_);
  }
};


using namespace std::chrono_literals;

int main(int argc, char** argv) {
  // Setup argument parser and parse arguments
  icehalo::ArgParser parser;
  parser.AddArgument("-v", 0, "verbose", "make output verbose");
  parser.AddArgument("-d", 0, "debug", "display debug info");
  parser.AddArgument("-n", 1, "repeat-number", "repeat number (-1 means infinity)");
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

  while (true) {
    auto res = s.GetResults();
    for (const auto& r : res) {
      std::visit(SimResultHandler{}, r);
    }
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}