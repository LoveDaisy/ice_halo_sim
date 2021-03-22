#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"
#include "core/simulation.hpp"
#include "util/arg_parser.hpp"
#include "util/log.hpp"


constexpr size_t kBufSize = 1024;
char str_buf[kBufSize];

using namespace icehalo;
std::vector<SpectrumRenderer> split_renderer_candidates;
std::vector<std::set<size_t>> renderer_ray_set;


void PrintRayPath(const RayPath& ray_path, char* buf, size_t size) {
  bool crystal_flag = true;
  size_t offset = 0;
  for (const auto& fn : ray_path) {
    if (crystal_flag) {
      auto n = std::snprintf(buf + offset, size - offset, "-(%u)", fn);
      offset += n;
      crystal_flag = false;
    } else if (fn == kInvalidId) {
      auto n = std::snprintf(buf + offset, size - offset, "-x");
      offset += n;
      crystal_flag = true;
    } else {
      auto n = std::snprintf(buf + offset, size - offset, "-%u", fn);
      offset += n;
    }
    if (offset > size) {
      break;
    }
  }
}


std::tuple<RayCollectionInfoList, SimpleRayData> RenderSplitHalos(SimulationData& ray_data, ProjectContextPtr& ctx,
                                                                  RenderContextPtr& split_render_ctx) {
  auto split_num = static_cast<size_t>(split_render_ctx->GetSplitNumber());
  size_t split_img_ch_num = split_render_ctx->GetSplitNumberPerImage();

  auto split_ray_data = ray_data.CollectSplitRayData(ctx, split_render_ctx->GetSplitter());
  const auto& exit_ray_data = std::get<1>(split_ray_data);
  const auto& ray_path_map = ray_data.ray_path_map_;

  if (exit_ray_data.buf_ray_num == 0) {
    return split_ray_data;
  }

  size_t curr_split_num = std::min(split_num, std::get<0>(split_ray_data).size());
  for (size_t j = 0; j < curr_split_num; j++) {
    const auto& collection_info = std::get<0>(split_ray_data)[j];
    auto hash = collection_info.identifier;
    size_t img_idx = 0;
    for (size_t k = 0; k < renderer_ray_set.size(); k++) {
      if (renderer_ray_set[k].count(hash)) {
        img_idx = k;
        break;
      } else if (renderer_ray_set[k].size() < split_img_ch_num) {
        renderer_ray_set[k].emplace(hash);
        img_idx = k;
        break;
      }
    }

    PrintRayPath(ray_path_map.at(hash).first, str_buf, kBufSize);
    LOG_VERBOSE("img_idx: %03zu, hash: %016tx, energy: %03.3e, ray_path: %s", img_idx, hash,
                collection_info.total_energy, str_buf);

    if (split_img_ch_num == 1) {
      auto identifier = static_cast<size_t>(exit_ray_data.wavelength);
      split_renderer_candidates[img_idx].LoadRayData(identifier, collection_info, exit_ray_data);
    } else {
      split_renderer_candidates[img_idx].LoadRayData(hash, collection_info, exit_ray_data);
    }
  }

  return split_ray_data;
}


size_t PrepareSplitRender(const ProjectContextPtr& proj_ctx, const RenderContextPtr& split_render_ctx) {
  auto split_img_num = static_cast<size_t>(split_render_ctx->GetSplitImageNumber());
  for (size_t i = 0; i < split_img_num * 1.5; i++) {
    split_renderer_candidates.emplace_back();
    split_renderer_candidates.back().SetCameraContext(proj_ctx->cam_ctx_);
    split_renderer_candidates.back().SetRenderContext(split_render_ctx);
    renderer_ray_set.emplace_back();
  }
  return split_img_num;
}


void WriteRayPathMap(const ProjectContextPtr& proj_ctx, const std::vector<RayCollectionInfo>& ray_info_list,
                     const RayPathMap& ray_path_map) {
  auto ray_path_result_file = PathJoin(proj_ctx->GetDataDirectory(), "ray_path_result.txt");
  auto* fp = fopen(ray_path_result_file.c_str(), "w");
  for (const auto& r : ray_info_list) {
    auto hash = r.identifier;
    size_t img_idx = 0;
    for (size_t k = 0; k < renderer_ray_set.size(); k++) {
      if (renderer_ray_set[k].count(hash)) {
        img_idx = k;
        break;
      }
    }

    PrintRayPath(ray_path_map.at(hash).first, str_buf, kBufSize);
    std::fprintf(fp, "img_idx: %03zu, hash: %016tx, energy: %04.3e, ray_path: %s\n", img_idx, hash, r.total_energy,
                 str_buf);
  }
  fclose(fp);
}


int main(int argc, char* argv[]) {
  ArgParser parser;
  parser.AddArgument("-v", 0, "verbose", "make output verbose");
  parser.AddArgument("-d", 0, "debug", "display debug info");
  parser.AddArgument("--config", 1, "config-file", "config file");
  ArgParseResult arg_parse_result;
  try {
    arg_parse_result = parser.Parse(argc, argv);
  } catch (...) {
    return -1;
  }
  const char* config_filename = arg_parse_result.at("--config")[0].c_str();
  if (arg_parse_result.count("-d")) {
    LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kDebug, LogLevel::kVerbose });
    LogDestPtr stdout_dest = LogStdOutDest::GetInstance();
    Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  } else if (arg_parse_result.count("-v")) {
    LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kVerbose });
    LogDestPtr stdout_dest = LogStdOutDest::GetInstance();
    Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  }

  auto start = std::chrono::system_clock::now();
  ProjectContextPtr proj_ctx = ProjectContext::CreateFromFile(config_filename);
  Simulator simulator(proj_ctx);
  SpectrumRenderer renderer;
  renderer.SetCameraContext(proj_ctx->cam_ctx_);
  renderer.SetRenderContext(proj_ctx->render_ctx_);

  auto& split_render_ctx = proj_ctx->split_render_ctx_;
  size_t split_img_num = 0;
  if (split_render_ctx) {
    split_img_num = PrepareSplitRender(proj_ctx, split_render_ctx);
  }

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t - start;
  LOG_INFO("Initialization: %.2fms", diff.count());

  File file(proj_ctx->GetDefaultImagePath().c_str());
  if (!file.Open(FileOpenMode::kWrite)) {
    LOG_ERROR("Cannot create output image file!");
    return -1;
  }
  file.Close();

  size_t total_ray_num = 0;
  while (true) {
    const auto& wavelengths = proj_ctx->wavelengths_;
    SimpleRayData exit_ray_data;
    std::vector<RayCollectionInfo> ray_info_list;
    for (size_t i = 0; i < wavelengths.size(); i++) {
      LOG_INFO("starting at wavelength: %d", wavelengths[i].wavelength);
      simulator.SetCurrentWavelengthIndex(i);

      auto t0 = std::chrono::system_clock::now();
      simulator.Run();
      auto t1 = std::chrono::system_clock::now();
      diff = t1 - t0;
      LOG_INFO("Ray tracing: %.2fms", diff.count());

      auto simulation_data = simulator.GetSimulationRayData();
      if (split_render_ctx) {
        std::tie(ray_info_list, exit_ray_data) = RenderSplitHalos(simulation_data, proj_ctx, split_render_ctx);
        if (exit_ray_data.buf_ray_num == 0) {
          continue;
        }

        if (total_ray_num == 0) {
          WriteRayPathMap(proj_ctx, ray_info_list, simulation_data.ray_path_map_);
        }

        auto& final_ray_info = ray_info_list[0];
        final_ray_info.is_partial_data = false;
        final_ray_info.total_energy = exit_ray_data.init_ray_num;
        renderer.LoadRayData(wavelengths[i].wavelength, final_ray_info, exit_ray_data);
      } else {
        auto ray_data = simulation_data.CollectFinalRayData();
        if (ray_data.second.buf_ray_num == 0) {
          continue;
        }
        renderer.LoadRayData(wavelengths[i].wavelength, ray_data.first, ray_data.second);
      }
      auto t2 = std::chrono::system_clock::now();
      diff = t2 - t0;
      LOG_INFO("Collecting rays: %.2fms", diff.count());
    }

    renderer.RenderToImage();

    cv::Mat img(proj_ctx->render_ctx_->GetImageHeight(), proj_ctx->render_ctx_->GetImageWidth(), CV_8UC3,
                renderer.GetImageBuffer());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imwrite(PathJoin(proj_ctx->GetDataDirectory(), "img.jpg"), img);

    if (split_render_ctx) {
      for (auto& r : split_renderer_candidates) {
        r.RenderToImage();
      }
      for (size_t i = 0; i < std::min(split_renderer_candidates.size(), split_img_num); i++) {
        cv::Mat halo_img(proj_ctx->split_render_ctx_->GetImageHeight(), proj_ctx->split_render_ctx_->GetImageWidth(),
                         CV_8UC3, split_renderer_candidates[i].GetImageBuffer());
        cv::cvtColor(halo_img, halo_img, cv::COLOR_RGB2BGR);
        std::snprintf(str_buf, kBufSize, "halo_%03zu.png", i);
        cv::imwrite(PathJoin(proj_ctx->GetDataDirectory(), str_buf), halo_img);
      }
    }

    t = std::chrono::system_clock::now();
    total_ray_num += proj_ctx->GetInitRayNum() * wavelengths.size();
    diff = t - start;
    LOG_INFO("=== Total %6.1fM rays finished! ===", total_ray_num / 1.0e6);
    LOG_INFO("=== Spent %7.2f sec!           ===", diff.count() / 1000);
  }

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  std::printf("Total: %.3fs\n", diff.count() / 1e3);

  return 0;
}

