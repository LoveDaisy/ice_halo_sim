#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context/context.h"
#include "core/render.h"

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

  icehalo::SimpleRayData ray_data;
  auto data_files = icehalo::ListDataFiles(ctx->GetDataDirectory().c_str());
  for (size_t i = 0; i < data_files.size(); i++) {
    auto t0 = std::chrono::system_clock::now();
    auto& file = data_files[i];
    file.Open(icehalo::FileOpenMode::kRead);
    ray_data.Deserialize(file, icehalo::endian::kUnknownEndian);
    renderer.LoadRayData(ray_data);
    auto t1 = std::chrono::system_clock::now();
    std::chrono::duration<float, std::ratio<1, 1000>> diff = t1 - t0;
    std::printf(" Loading data (%zu/%zu): %.2fms; total %zu pts\n", i + 1, data_files.size(), diff.count(),
                ray_data.size);
  }
  renderer.RenderToImage();

  cv::Mat img(ctx->render_ctx_->GetImageHeight(), ctx->render_ctx_->GetImageWidth(), CV_8UC3,
              renderer.GetImageBuffer());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  cv::imwrite(ctx->GetDefaultImagePath(), img);

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t1 - start;
  std::printf("Total: %.2fms\n", diff.count());

  return 0;
}
