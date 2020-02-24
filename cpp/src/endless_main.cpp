#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"
#include "core/simulation.hpp"
#include "util/log.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"


constexpr size_t kBufSize = 1024;
char str_buf[kBufSize];


void PrintRayPath(const icehalo::RayPath& ray_path, char* buf, size_t size) {
  bool crystal_flag = true;
  size_t offset = 0;
  for (const auto& fn : ray_path) {
    if (crystal_flag) {
      auto n = std::snprintf(buf + offset, size - offset, "-(%u)", fn);
      offset += n;
      crystal_flag = false;
    } else if (fn == icehalo::kInvalidFaceNumber) {
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


std::tuple<icehalo::RayCollectionInfoList, icehalo::SimpleRayData, icehalo::RayPathMap> RenderSplitHalos(
    icehalo::SimulationData& ray_data, icehalo::ProjectContextPtr& ctx, icehalo::RenderContextPtr& split_render_ctx,
    std::vector<icehalo::SpectrumRenderer>& split_renderer_candidates,
    std::vector<std::set<size_t>>& renderer_ray_set) {
  size_t split_img_ch_num =
      icehalo::SpectrumRenderer::kImageBits / static_cast<int>(split_render_ctx->GetColorCompactLevel());
  auto split_num = static_cast<size_t>(split_render_ctx->GetSplitNumber());

  auto split_ray_data = ray_data.CollectSplitRayData(ctx, split_render_ctx->GetSplitter());
  const auto& exit_ray_data = std::get<1>(split_ray_data);
  const auto& ray_path_map = std::get<2>(split_ray_data);

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

    PrintRayPath(ray_path_map.at(hash), str_buf, kBufSize);
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
  if (argc != 2) {
    std::printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  icehalo::ProjectContextPtr proj_ctx = icehalo::ProjectContext::CreateFromFile(argv[1]);
  icehalo::Simulator simulator(proj_ctx);
  icehalo::SpectrumRenderer renderer;
  renderer.SetCameraContext(proj_ctx->cam_ctx_);
  renderer.SetRenderContext(proj_ctx->render_ctx_);

  auto& split_render_ctx = proj_ctx->split_render_ctx_;
  std::vector<icehalo::SpectrumRenderer> split_renderer_candidates;
  std::vector<std::set<size_t>> renderer_ray_set;
  size_t split_img_num = 0;
  if (split_render_ctx) {
    size_t split_img_ch_num =
        icehalo::SpectrumRenderer::kImageBits / static_cast<int>(split_render_ctx->GetColorCompactLevel());
    auto split_num = static_cast<size_t>(split_render_ctx->GetSplitNumber());
    split_img_num = static_cast<size_t>(std::ceil(split_num * 1.0 / split_img_ch_num));
    for (size_t i = 0; i < split_img_num * 1.5; i++) {
      split_renderer_candidates.emplace_back();
      split_renderer_candidates.back().SetCameraContext(proj_ctx->cam_ctx_);
      split_renderer_candidates.back().SetRenderContext(split_render_ctx);
      renderer_ray_set.emplace_back();
    }
  }

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t - start;
  LOG_INFO("Initialization: %.2fms", diff.count());

  icehalo::File file(proj_ctx->GetDefaultImagePath().c_str());
  if (!file.Open(icehalo::FileOpenMode::kWrite)) {
    LOG_ERROR("Cannot create output image file!");
    return -1;
  }
  file.Close();

  size_t total_ray_num = 0;
  while (true) {
    const auto& wavelengths = proj_ctx->wavelengths_;
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
        auto split_ray_data =
            RenderSplitHalos(simulation_data, proj_ctx, split_render_ctx, split_renderer_candidates, renderer_ray_set);
        const auto& exit_ray_data = std::get<1>(split_ray_data);
        icehalo::RayCollectionInfo final_ray_info = std::get<0>(split_ray_data)[0];
        final_ray_info.is_partial_data = false;
        renderer.LoadRayData(static_cast<size_t>(exit_ray_data.wavelength), final_ray_info, exit_ray_data);
      } else {
        auto ray_data = simulation_data.CollectFinalRayData();
        renderer.LoadRayData(static_cast<size_t>(ray_data.second.wavelength), ray_data.first, ray_data.second);
      }
      auto t2 = std::chrono::system_clock::now();
      diff = t2 - t0;
      LOG_INFO("Collecting rays: %.2fms", diff.count());
    }

    renderer.RenderToImage();

    cv::Mat img(proj_ctx->render_ctx_->GetImageHeight(), proj_ctx->render_ctx_->GetImageWidth(), CV_8UC3,
                renderer.GetImageBuffer());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imwrite(icehalo::PathJoin(proj_ctx->GetDataDirectory(), "img.jpg"), img);

    if (split_render_ctx) {
      for (auto& r : split_renderer_candidates) {
        r.RenderToImage();
      }
      for (size_t i = 0; i < std::min(split_renderer_candidates.size(), split_img_num); i++) {
        cv::Mat halo_img(proj_ctx->split_render_ctx_->GetImageHeight(), proj_ctx->split_render_ctx_->GetImageWidth(),
                         CV_8UC3, split_renderer_candidates[i].GetImageBuffer());
        cv::cvtColor(halo_img, halo_img, cv::COLOR_RGB2BGR);
        std::snprintf(str_buf, kBufSize, "halo_%03zu.jpg", i);
        cv::imwrite(icehalo::PathJoin(proj_ctx->GetDataDirectory(), str_buf), halo_img);
      }
    }

    t = std::chrono::system_clock::now();
    total_ray_num += proj_ctx->GetInitRayNum() * wavelengths.size();
    diff = t - start;
    LOG_INFO("=== Total %zu rays finished! ===", total_ray_num);
    LOG_INFO("=== Spent %.3f sec!          ===", diff.count() / 1000);
  }

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  std::printf("Total: %.3fs\n", diff.count() / 1e3);

  return 0;
}

#pragma clang diagnostic pop
