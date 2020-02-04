#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "context.h"
#include "render.h"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::printf("USAGE: %s config.json\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  icehalo::ProjectContextPtr ctx = icehalo::ProjectContext::CreateFromFile(argv[1]);
  icehalo::SpectrumRenderer renderer(ctx);
  renderer.LoadDataFiles();
  renderer.RenderToImage();

  cv::Mat img(ctx->render_ctx_.GetImageHeight(), ctx->render_ctx_.GetImageWidth(), CV_8UC3, renderer.GetImageBuffer());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  try {
    cv::imwrite(ctx->GetDefaultImagePath(), img);
  } catch (cv::Exception& ex) {
    std::fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    return -1;
  }

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t1 - start;
  std::printf("Total: %.2fms\n", diff.count());

  return 0;
}
