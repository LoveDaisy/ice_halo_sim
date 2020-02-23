#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.hpp"
#include "core/render.hpp"
#include "core/simulation.hpp"
#include "util/log.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

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

      auto ray_data = simulator.GetSimulationRayData().CollectFinalRayData();
      renderer.LoadRayData(static_cast<size_t>(ray_data.second.wavelength), ray_data.first, ray_data.second);
    }

    renderer.RenderToImage();

    cv::Mat img(proj_ctx->render_ctx_->GetImageHeight(), proj_ctx->render_ctx_->GetImageWidth(), CV_8UC3,
                renderer.GetImageBuffer());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imwrite(proj_ctx->GetDefaultImagePath(), img);

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
