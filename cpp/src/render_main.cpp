#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"
#include "util/log.hpp"

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

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::printf("USAGE: %s config.json\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  icehalo::ProjectContextPtr ctx = icehalo::ProjectContext::CreateFromFile(argv[1]);
  icehalo::SpectrumRenderer renderer;
  renderer.SetCameraContext(ctx->cam_ctx_);
  renderer.SetRenderContext(ctx->render_ctx_);

  auto& split_render_ctx = ctx->split_render_ctx_;
  size_t split_img_ch_num =
      icehalo::SpectrumRenderer::kImageBits / static_cast<int>(split_render_ctx->GetColorCompactLevel());
  auto split_num = static_cast<size_t>(split_render_ctx->GetSplitNumber());
  auto split_img_num = static_cast<size_t>(std::ceil(split_num * 1.0 / split_img_ch_num));
  std::vector<icehalo::SpectrumRenderer> split_renderer_candidates;
  std::vector<std::set<size_t>> renderer_ray_set;
  for (size_t i = 0; i < split_img_num * 1.5; i++) {
    split_renderer_candidates.emplace_back();
    split_renderer_candidates.back().SetCameraContext(ctx->cam_ctx_);
    split_renderer_candidates.back().SetRenderContext(split_render_ctx);
    renderer_ray_set.emplace_back();
  }

  constexpr size_t kBufSize = 1024;
  char str_buf[kBufSize];

  icehalo::SimulationData ray_data;
  auto data_files = icehalo::ListDataFiles(ctx->GetDataDirectory().c_str());
  for (size_t i = 0; i < data_files.size(); i++) {
    auto t0 = std::chrono::system_clock::now();
    auto& file = data_files[i];
    file.Open(icehalo::FileOpenMode::kRead);
    ray_data.Deserialize(file, icehalo::endian::kUnknownEndian);
    file.Close();
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> loading_time = t1 - t0;

    size_t init_ray_num, exit_seg_num;
    {
      auto final_ray_data = ray_data.CollectFinalRayData();
      renderer.LoadRayData(static_cast<size_t>(final_ray_data.second.wavelength), final_ray_data.first,
                           final_ray_data.second);
      init_ray_num = final_ray_data.first.init_ray_num;
      exit_seg_num = final_ray_data.second.size;
    }  // Let final_ray_data be local.
    auto t2 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> final_render_time = t2 - t1;

    {
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
    }  // Let top_halo_ray_data be local.
    auto t3 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> split_render_time = t3 - t2;

    LOG_INFO(
        " Loading data (%zu/%zu): %.2fms. Collecting final rays: %.2fms. Filtering top halo: %.2fms."
        " Total %zu rays, %zu pts",
        i + 1, data_files.size(), loading_time.count(), final_render_time.count(), split_render_time.count(),
        init_ray_num, exit_seg_num);
  }
  renderer.RenderToImage();

  cv::Mat img(ctx->render_ctx_->GetImageHeight(), ctx->render_ctx_->GetImageWidth(), CV_8UC3,
              renderer.GetImageBuffer());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  cv::imwrite(icehalo::PathJoin(ctx->GetDataDirectory(), "img.jpg"), img);

  for (auto& r : split_renderer_candidates) {
    r.RenderToImage();
  }
  for (size_t i = 0; i < std::min(split_renderer_candidates.size(), split_img_num); i++) {
    cv::Mat halo_img(ctx->split_render_ctx_->GetImageHeight(), ctx->split_render_ctx_->GetImageWidth(), CV_8UC3,
                     split_renderer_candidates[i].GetImageBuffer());
    cv::cvtColor(halo_img, halo_img, cv::COLOR_RGB2BGR);
    std::snprintf(str_buf, kBufSize, "halo_%03zu.jpg", i);
    cv::imwrite(icehalo::PathJoin(ctx->GetDataDirectory(), str_buf), halo_img);
  }

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> diff = t1 - start;
  LOG_INFO("Total: %.2fms", diff.count());

  return 0;
}
