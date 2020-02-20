#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"

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

  using ColorCompactLevelDataType = std::underlying_type<icehalo::ColorCompactLevel>::type;
  auto& split_render_ctx = ctx->split_render_ctx_;
  int split_img_ch_num = icehalo::SpectrumRenderer::kImageBits /
                         static_cast<ColorCompactLevelDataType>(split_render_ctx->GetColorCompactLevel());
  int split_img_num = static_cast<int>(std::ceil(split_render_ctx->GetSplitNumber() * 1.0 / split_img_ch_num));
  std::vector<icehalo::SpectrumRenderer> split_renderers;
  for (int i = 0; i < split_img_num; i++) {
    split_renderers.emplace_back();
    split_renderers.back().SetCameraContext(ctx->cam_ctx_);
    split_renderers.back().SetRenderContext(split_render_ctx);
  }

  icehalo::SimulationData ray_data;
  auto data_files = icehalo::ListDataFiles(ctx->GetDataDirectory().c_str());
  for (size_t i = 0; i < data_files.size(); i++) {
    auto t0 = std::chrono::system_clock::now();
    auto& file = data_files[i];
    file.Open(icehalo::FileOpenMode::kRead);
    ray_data.Deserialize(file, icehalo::endian::kUnknownEndian);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000>> loading_time = t1 - t0;

    size_t init_ray_num, exit_seg_num;
    decltype(t1) t2;
    std::chrono::duration<float, std::ratio<1, 1000>> final_render_time{};
    {
      auto final_ray_data = ray_data.CollectFinalRayData();
      renderer.LoadRayData(static_cast<size_t>(final_ray_data.wavelength), final_ray_data);
      t2 = std::chrono::system_clock::now();
      final_render_time = t2 - t1;
      init_ray_num = final_ray_data.init_ray_num;
      exit_seg_num = final_ray_data.size;
    }  // Let final_ray_data be local.

    decltype(t1) t3;
    std::chrono::duration<float, std::ratio<1, 1000>> split_render_time{};
    {
      auto split_ray_data = ray_data.CollectSplitRayData(ctx, split_render_ctx->GetSplitter());
      size_t curr_split_num = std::min(static_cast<size_t>(split_render_ctx->GetSplitNumber()), split_ray_data.size());
      for (size_t j = 0; j < curr_split_num; j++) {
        auto img_idx = j / split_img_ch_num;
        if (split_img_ch_num == 1) {
          split_renderers[img_idx].LoadRayData(static_cast<size_t>(split_ray_data[j].ray_data.wavelength),
                                               split_ray_data[j].ray_data);
        } else {
          split_renderers[img_idx].LoadRayData(split_ray_data[j].ray_path_hash, split_ray_data[j].ray_data);
        }
      }
      t3 = std::chrono::system_clock::now();
      split_render_time = t3 - t2;
    }  // Let top_halo_ray_data be local.

    std::printf(
        " Loading data (%zu/%zu): %.2fms. Collecting final rays: %.2fms. Filtering top halo: %.2fms."
        " Total %zu rays, %zu pts\n",
        i + 1, data_files.size(), loading_time.count(), final_render_time.count(), split_render_time.count(),
        init_ray_num, exit_seg_num);
  }
  renderer.RenderToImage();
  for (auto& r : split_renderers) {
    r.RenderToImage();
  }

  cv::Mat img(ctx->render_ctx_->GetImageHeight(), ctx->render_ctx_->GetImageWidth(), CV_8UC3,
              renderer.GetImageBuffer());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  cv::imwrite(icehalo::PathJoin(ctx->GetDataDirectory(), "img.jpg"), img);
  char filename_buf[256];
  for (size_t i = 0; i < split_renderers.size(); i++) {
    cv::Mat halo_img(ctx->split_render_ctx_->GetImageHeight(), ctx->split_render_ctx_->GetImageWidth(), CV_8UC3,
                     split_renderers[i].GetImageBuffer());
    cv::cvtColor(halo_img, halo_img, cv::COLOR_RGB2BGR);
    std::snprintf(filename_buf, 256, "halo_%03zu.jpg", i);
    cv::imwrite(icehalo::PathJoin(ctx->GetDataDirectory(), filename_buf), halo_img);
  }

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t1 - start;
  std::printf("Total: %.2fms\n", diff.count());

  return 0;
}
