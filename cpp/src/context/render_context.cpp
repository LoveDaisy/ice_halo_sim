#include "context/render_context.h"

#include <algorithm>

#include "core/render.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"


namespace icehalo {

using rapidjson::Pointer;

RenderContext::RenderContext()
    : ray_color_{ 1.0f, 1.0f, 1.0f }, background_color_{ 0.0f, 0.0f, 0.0f }, intensity_(1.0f), image_width_(0),
      image_height_(0), offset_x_(0), offset_y_(0), visible_range_(VisibleRange::kUpper) {}


RenderContextPtrU RenderContext::CreateFromJson(rapidjson::Document& d) {
  RenderContextPtrU render_ctx{ new RenderContext };
  render_ctx->SetImageWidth(800);
  auto p = Pointer("/render/width").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.width>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.width> is not an integer, using default 800!\n");
  } else {
    render_ctx->SetImageWidth(p->GetInt());
  }

  render_ctx->SetImageHeight(800);
  p = Pointer("/render/height").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <camera.height>, using default 800!\n");
  } else if (!p->IsInt()) {
    std::fprintf(stderr, "\nWARNING! config <camera.height> is not an integer, using default 800!\n");
  } else {
    render_ctx->SetImageHeight(p->GetInt());
  }

  render_ctx->SetVisibleRange(VisibleRange::kUpper);
  p = Pointer("/render/visible_semi_sphere").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.visible_semi_sphere>, using default kUpper!\n");
  } else if (!p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.visible_semi_sphere> is not a string, using default kUpper!\n");
  } else if (*p == "upper") {
    render_ctx->SetVisibleRange(VisibleRange::kUpper);
  } else if (*p == "lower") {
    render_ctx->SetVisibleRange(VisibleRange::kLower);
  } else if (*p == "camera") {
    render_ctx->SetVisibleRange(VisibleRange::kFront);
  } else if (*p == "full") {
    render_ctx->SetVisibleRange(VisibleRange::kFull);
  } else {
    std::fprintf(stderr,
                 "\nWARNING! Config <render.visible_semi_sphere> cannot be recognized, using default kUpper!\n");
  }

  render_ctx->SetIntensity(1.0f);
  p = Pointer("/render/intensity_factor").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.intensity_factor>, using default 1.0!\n");
  } else if (!p->IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.intensity_factor> is not a number, using default 1.0!\n");
  } else {
    auto f = static_cast<float>(p->GetDouble());
    f = std::max(std::min(f, RenderContext::kMaxIntensity), RenderContext::kMinIntensity);
    render_ctx->SetIntensity(f);
  }

  render_ctx->SetImageOffsetX(0);
  render_ctx->SetImageOffsetY(0);
  p = Pointer("/render/offset").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.offset>, using default [0, 0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> is not an array, using default [0, 0]!\n");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    std::fprintf(stderr, "\nWARNING! Config <render.offset> cannot be recognized, using default [0, 0]!\n");
  } else {
    int offset_x = (*p)[0].GetInt();
    int offset_y = (*p)[1].GetInt();
    offset_x = std::max(std::min(offset_x, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    offset_y = std::max(std::min(offset_y, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    render_ctx->SetImageOffsetX(offset_x);
    render_ctx->SetImageOffsetY(offset_y);
  }

  render_ctx->ResetBackgroundColor();
  p = Pointer("/render/background_color").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.background_color>, using default [0,0,0]!\n");
  } else if (!p->IsArray()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> is not an array, using default [0,0,0]!\n");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    std::fprintf(stderr, "\nWARNING! Config <render.background_color> cannot be recognized, using default [0,0,0]!\n");
  } else {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    render_ctx->SetBackgroundColor(r, g, b);
  }

  render_ctx->ResetRayColor();
  p = Pointer("/render/ray_color").Get(d);
  if (p == nullptr) {
    std::fprintf(stderr, "\nWARNING! Config missing <render.ray_color>, using default real color!\n");
  } else if (!p->IsArray() && !p->IsString()) {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> is not an array nor a string, ");
    std::fprintf(stderr, "using default real color!\n");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> cannot be recognized, using default real color!\n");
  } else if (p->IsString() && (*p) != "real") {
    std::fprintf(stderr, "\nWARNING! Config <render.ray_color> cannot be recognized, using default real color!\n");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    render_ctx->SetRayColor(r, g, b);
  }

  return render_ctx;
}


constexpr float RenderContext::kMinIntensity;
constexpr float RenderContext::kMaxIntensity;
constexpr int RenderContext::kMaxImageSize;


const float* RenderContext::GetRayColor() const {
  return ray_color_;
}


void RenderContext::SetRayColor(float r, float g, float b) {
  ray_color_[0] = std::min(std::max(r, 0.0f), 1.0f);
  ray_color_[1] = std::min(std::max(g, 0.0f), 1.0f);
  ray_color_[2] = std::min(std::max(b, 0.0f), 1.0f);
}


void RenderContext::ResetRayColor() {
  UseRealRayColor();
}


void RenderContext::UseRealRayColor() {
  ray_color_[0] = -1.0f;
  ray_color_[1] = -1.0f;
  ray_color_[2] = -1.0f;
}


const float* RenderContext::GetBackgroundColor() const {
  return background_color_;
}


void RenderContext::SetBackgroundColor(float r, float g, float b) {
  background_color_[0] = std::min(std::max(r, 0.0f), 1.0f);
  background_color_[1] = std::min(std::max(g, 0.0f), 1.0f);
  background_color_[2] = std::min(std::max(b, 0.0f), 1.0f);
}


void RenderContext::ResetBackgroundColor() {
  background_color_[0] = 0.0f;
  background_color_[1] = 0.0f;
  background_color_[2] = 0.0f;
}


void RenderContext::UseSkyBackground() {
  background_color_[0] = -1.0f;
  background_color_[1] = -1.0f;
  background_color_[2] = -1.0f;
}


float RenderContext::GetIntensity() const {
  return intensity_;
}


void RenderContext::SetIntensity(float intensity) {
  intensity_ = std::min(std::max(intensity, kMinIntensity), kMaxIntensity);
}


int RenderContext::GetImageWidth() const {
  return image_width_;
}


int RenderContext::GetImageHeight() const {
  return image_height_;
}


void RenderContext::SetImageWidth(int w) {
  image_width_ = std::min(std::max(w, 0), kMaxImageSize);
}


void RenderContext::SetImageHeight(int h) {
  image_height_ = std::min(std::max(h, 0), kMaxImageSize);
}


int RenderContext::GetImageOffsetX() const {
  return offset_x_;
}


int RenderContext::GetImageOffsetY() const {
  return offset_y_;
}


void RenderContext::SetImageOffsetX(int offset_x) {
  offset_x_ = offset_x;
}


void RenderContext::SetImageOffsetY(int offset_y) {
  offset_y_ = offset_y;
}


VisibleRange RenderContext::GetVisibleRange() const {
  return visible_range_;
}


void RenderContext::SetVisibleRange(VisibleRange r) {
  visible_range_ = r;
}

}  // namespace icehalo
