#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"
#include "util/arg_parser.hpp"
#include "util/log.hpp"

constexpr size_t kBufSize = 1024;
char str_buf[kBufSize];

using namespace icehalo;

void PrintRayPath(const RayPath& ray_path, char* buf, size_t size) {
  bool crystal_flag = true;
  size_t offset = 0;
  for (const auto& fn : ray_path) {
    if (crystal_flag) {
      auto n = std::snprintf(buf + offset, size - offset, "-(%u)", fn);
      offset += n;
      crystal_flag = false;
    } else if (fn == kInvalidFaceNumber) {
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


std::tuple<RayCollectionInfoList, SimpleRayData> RenderSplitHalos(
    SimulationData& ray_data, ProjectContextPtr& ctx, RenderContextPtr& split_render_ctx,
    std::vector<SpectrumRenderer>& split_renderer_candidates, std::vector<std::set<size_t>>& renderer_ray_set) {
  auto split_num = static_cast<size_t>(split_render_ctx->GetSplitNumber());
  size_t split_img_ch_num = split_render_ctx->GetSplitNumberPerImage();

  auto split_ray_data = ray_data.CollectSplitRayData(ctx, split_render_ctx->GetSplitter());
  const auto& exit_ray_data = std::get<1>(split_ray_data);
  const auto& ray_path_map = ray_data.ray_path_map_;

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
    LOG_VERBOSE("img_idx: %03zu, hash: %016tx, ray_path: %s", img_idx, hash, str_buf);

    if (split_img_ch_num == 1) {
      auto identifier = static_cast<size_t>(exit_ray_data.wavelength);
      split_renderer_candidates[img_idx].LoadRayData(identifier, collection_info, exit_ray_data);
    } else {
      split_renderer_candidates[img_idx].LoadRayData(hash, collection_info, exit_ray_data);
    }
  }

  return split_ray_data;
}


int main(int argc, char* argv[]) {
  ArgParser parser;
  parser.AddArgument("-v", 0, "verbose", "make output verbose");
  parser.AddArgument("--config", 1, "config-file", "config file");
  ArgParseResult arg_parse_result;
  try {
    arg_parse_result = parser.Parse(argc, argv);
  } catch (...) {
    return -1;
  }
  const char* config_filename = arg_parse_result.at("--config")[0].c_str();
  if (arg_parse_result.count("-v")) {
    LogFilterPtr stdout_filter = LogFilter::MakeLevelFilter({ LogLevel::kVerbose });
    LogDestPtr stdout_dest = LogStdOutDest::GetInstance();
    Logger::GetInstance()->AddDestination(stdout_filter, stdout_dest);
  }

  auto start = std::chrono::system_clock::now();
  ProjectContextPtr ctx = ProjectContext::CreateFromFile(config_filename);
  SpectrumRenderer renderer;
  renderer.SetCameraContext(ctx->cam_ctx_);
  renderer.SetRenderContext(ctx->render_ctx_);

  auto& split_render_ctx = ctx->split_render_ctx_;
  std::vector<SpectrumRenderer> split_renderer_candidates;
  std::vector<std::set<size_t>> renderer_ray_set;
  size_t split_img_num = 0;
  if (split_render_ctx) {
    split_img_num = static_cast<size_t>(split_render_ctx->GetSplitImageNumber());
    for (size_t i = 0; i < split_img_num * 1.5; i++) {
      split_renderer_candidates.emplace_back();
      split_renderer_candidates.back().SetCameraContext(ctx->cam_ctx_);
      split_renderer_candidates.back().SetRenderContext(split_render_ctx);
      renderer_ray_set.emplace_back();
    }
  }

  SimulationData ray_data;
  auto data_files = ListDataFiles(ctx->GetDataDirectory().c_str());
  for (size_t i = 0; i < data_files.size(); i++) {
    auto t0 = std::chrono::system_clock::now();
    auto& file = data_files[i];
    file.Open(FileOpenMode::kRead);
    ray_data.Deserialize(file, endian::kUnknownEndian);
    file.Close();
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> loading_time = t1 - t0;

    size_t init_ray_num;
    size_t exit_seg_num;
    if (split_render_ctx && split_render_ctx->GetSplitNumber() > 0) {
      auto split_ray_data =
          RenderSplitHalos(ray_data, ctx, split_render_ctx, split_renderer_candidates, renderer_ray_set);
      const auto& exit_ray_data = std::get<1>(split_ray_data);
      RayCollectionInfo final_ray_info = std::get<0>(split_ray_data)[0];
      final_ray_info.is_partial_data = false;
      renderer.LoadRayData(static_cast<size_t>(exit_ray_data.wavelength), final_ray_info, exit_ray_data);
      init_ray_num = exit_ray_data.init_ray_num;
      exit_seg_num = exit_ray_data.buf_ray_num;
    } else {
      auto final_ray_data = ray_data.CollectFinalRayData();
      renderer.LoadRayData(static_cast<size_t>(final_ray_data.second.wavelength), final_ray_data.first,
                           final_ray_data.second);
      init_ray_num = final_ray_data.second.init_ray_num;
      exit_seg_num = final_ray_data.second.buf_ray_num;
    }

    auto t2 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> split_render_time = t2 - t1;

    LOG_INFO(
        " Loading data (%zu/%zu): %.2fms. Filtering top halo: %.2fms."
        " Total %zu rays, %zu pts",
        i + 1, data_files.size(), loading_time.count(), split_render_time.count(), init_ray_num, exit_seg_num);
  }
  renderer.RenderToImage();

  cv::Mat img(ctx->render_ctx_->GetImageHeight(), ctx->render_ctx_->GetImageWidth(), CV_8UC3,
              renderer.GetImageBuffer());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  cv::imwrite(PathJoin(ctx->GetDataDirectory(), "img.jpg"), img);

  if (split_render_ctx) {
    for (auto& r : split_renderer_candidates) {
      r.RenderToImage();
    }
    for (size_t i = 0; i < std::min(split_renderer_candidates.size(), split_img_num); i++) {
      cv::Mat halo_img(ctx->split_render_ctx_->GetImageHeight(), ctx->split_render_ctx_->GetImageWidth(), CV_8UC3,
                       split_renderer_candidates[i].GetImageBuffer());
      cv::cvtColor(halo_img, halo_img, cv::COLOR_RGB2BGR);
      std::snprintf(str_buf, kBufSize, "halo_%03zu.jpg", i);
      cv::imwrite(PathJoin(ctx->GetDataDirectory(), str_buf), halo_img);
    }
  }

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t1 - start;
  LOG_INFO("Total: %.2fms", diff.count());

  return 0;
}
