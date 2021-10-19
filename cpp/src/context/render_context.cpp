#include "context/render_context.hpp"

#include <algorithm>

#include "process/render.hpp"
#include "util/log.hpp"


namespace icehalo {

RenderSplitter::RenderSplitter() : type(RenderSplitterType::kNone), top_halo_num(0) {}


void to_json(nlohmann::json& obj, const RenderSplitter& splitter) {
  obj["type"] = splitter.type;
  switch (splitter.type) {
    case RenderSplitterType::kTopHalo:
      obj["param"] = splitter.top_halo_num;
      break;
    case RenderSplitterType::kFilter: {
      for (const auto& ids : splitter.crystal_filters) {
        obj["param"].emplace_back(nlohmann::json::array());
        for (const auto& f : ids) {
          obj["param"].back().emplace_back(f);
        }
      }
    } break;
    case RenderSplitterType::kNone:
    default:
      break;
  }
}


void from_json(const nlohmann::json& obj, RenderSplitter& splitter) {
  obj.at("type").get_to(splitter.type);
  switch (splitter.type) {
    case RenderSplitterType::kTopHalo:
      obj.at("param").get_to(splitter.top_halo_num);
      break;
    case RenderSplitterType::kFilter: {
      splitter.crystal_filters.clear();
      if (!obj.at("param").is_array() || !obj.at("param")[0].is_array()) {
        throw nlohmann::detail::other_error::create(-1, "param must be a nested array!", obj);
      }
      for (const auto& f : obj.at("param")) {
        splitter.crystal_filters.emplace_back();
        for (const auto& n : f) {
          splitter.crystal_filters.back().emplace_back(n.get<ShortIdType>());
        }
      }
    } break;
    case RenderSplitterType::kNone:
    default:
      break;
  }
}


LineSpecifier::LineSpecifier() : LineSpecifier(LineType::kSolid, kDefaultWidth, kDefaultColor, kDefaultAlpha) {}


LineSpecifier::LineSpecifier(LineType type, float width, const float color[3], float alpha)
    : type(type), width(width), color{ color[0], color[1], color[2] }, alpha(alpha) {}


void to_json(nlohmann::json& obj, const LineSpecifier& line) {
  obj["type"] = line.type;
  obj["width"] = line.width;
  obj["color"] = { line.color[0], line.color[1], line.color[2] };
  obj["alpha"] = line.alpha;
}


void from_json(const nlohmann::json& obj, LineSpecifier& line) {
  obj.at("type").get_to(line.type);
  try {
    obj.at("width").get_to(line.width);
    obj.at("alpha").get_to(line.alpha);
    obj.at("color")[0].get_to(line.color[0]);
    obj.at("color")[1].get_to(line.color[1]);
    obj.at("color")[2].get_to(line.color[2]);
  } catch (...) {}
}


RenderContext::RenderContext()
    : ray_color_{ 1.0f, 1.0f, 1.0f }, background_color_{ 0.0f, 0.0f, 0.0f }, intensity_(1.0f), image_width_(0),
      image_height_(0), offset_x_(0), offset_y_(0), visible_range_(VisibleRange::kUpper),
      color_compact_level_(ColorCompactLevel::kTrueColor) {}


RenderContextPtrU RenderContext::CreateDefault() {
  RenderContextPtrU render_ctx{ new RenderContext };
  return render_ctx;
}


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


const RenderSplitter& RenderContext::GetSplitter() const {
  return splitter_;
}


int RenderContext::GetSplitNumber() const {
  switch (splitter_.type) {
    case RenderSplitterType::kNone:
      return 0;
    case RenderSplitterType::kTopHalo:
      return splitter_.top_halo_num;
    case RenderSplitterType::kFilter:
      return static_cast<int>(splitter_.crystal_filters.size());
  }
}


int RenderContext::GetSplitImageNumber() const {
  auto split_num = GetSplitNumber();
  if (split_num <= 0) {
    return 0;
  }
  auto channels = GetSplitNumberPerImage();
  return static_cast<int>(std::ceil(split_num * 1.0 / channels));
}


int RenderContext::GetSplitNumberPerImage() const {
  return Renderer::kImageBits / static_cast<int>(color_compact_level_);
}


void RenderContext::SetColorCompactLevel(ColorCompactLevel level) {
  color_compact_level_ = level;
}


ColorCompactLevel RenderContext::GetColorCompactLevel() const {
  return color_compact_level_;
}


VisibleRange RenderContext::GetVisibleRange() const {
  return visible_range_;
}


void RenderContext::SetVisibleRange(VisibleRange r) {
  visible_range_ = r;
}


const std::vector<GridLine>& RenderContext::GetElevationGrids() const {
  return elevation_grid_;
}


const std::vector<GridLine>& RenderContext::GetRadiusGrids() const {
  return radius_rid_;
}


void to_json(nlohmann::json& obj, const RenderContext& ctx) {
  ctx.SaveColorConfig(obj);
  ctx.SaveIntensity(obj);
  ctx.SaveImageSize(obj);
  ctx.SaveImageOffset(obj);
  ctx.SaveVisibleRange(obj);
  ctx.SaveRenderSplitter(obj);
  ctx.SaveGridLines(obj);
}


void RenderContext::SaveColorConfig(nlohmann::json& obj) const {
  obj["ray_compact_level"] = color_compact_level_;
  obj["background_color"][0] = background_color_[0];
  obj["background_color"][1] = background_color_[1];
  obj["background_color"][2] = background_color_[2];

  if (ray_color_[0] < 0) {
    obj["ray_color"] = "real";
  } else {
    obj["ray_color"][0] = ray_color_[0];
    obj["ray_color"][1] = ray_color_[1];
    obj["ray_color"][2] = ray_color_[2];
  }
}


void RenderContext::SaveIntensity(nlohmann::json& obj) const {
  obj["intensity_factor"] = intensity_;
}


void RenderContext::SaveImageSize(nlohmann::json& obj) const {
  obj["width"] = image_width_;
  obj["height"] = image_height_;
}


void RenderContext::SaveImageOffset(nlohmann::json& obj) const {
  obj["offset"][0] = offset_x_;
  obj["offset"][1] = offset_y_;
}


void RenderContext::SaveVisibleRange(nlohmann::json& obj) const {
  obj["visible_semi_sphere"] = visible_range_;
}


void RenderContext::SaveRenderSplitter(nlohmann::json& obj) const {
  obj["splitter"] = splitter_;
}


void RenderContext::SaveGridLines(nlohmann::json& obj) const {
  for (const auto& l : elevation_grid_) {
    obj["elevation_grid"].emplace_back(l);
  }
  for (const auto& l : radius_rid_) {
    obj["radius_grid"].emplace_back(l);
  }
}


void from_json(const nlohmann::json& obj, RenderContext& ctx) {
  ctx.LoadColorConfig(obj);
  ctx.LoadImageSize(obj);
  ctx.LoadImageOffset(obj);
  ctx.LoadVisibleRange(obj);
  ctx.LoadIntensity(obj);
  ctx.LoadRenderSplitter(obj);
  ctx.LoadGridLines(obj);
}


void RenderContext::LoadColorConfig(const nlohmann::json& obj) {
  SetColorCompactLevel(ColorCompactLevel::kTrueColor);
  try {
    obj.at("color_compact_level").get_to(color_compact_level_);
  } catch (...) {
    LOG_VERBOSE("cannot parse color_compact_level. use default %d", color_compact_level_);
  }

  ResetBackgroundColor();
  auto r = obj.at("background_color")[0].get<float>();
  auto g = obj.at("background_color")[1].get<float>();
  auto b = obj.at("background_color")[2].get<float>();
  SetBackgroundColor(r, g, b);

  ResetRayColor();
  if (obj.at("ray_color").is_array()) {
    r = obj.at("ray_color")[0].get<float>();
    g = obj.at("ray_color")[1].get<float>();
    b = obj.at("ray_color")[2].get<float>();
    SetRayColor(r, g, b);
  }
}


void RenderContext::LoadImageSize(const nlohmann::json& obj) {
  SetImageWidth(kDefaultImageSize);
  try {
    auto width = obj.at("width").get<int>();
    SetImageWidth(width);
  } catch (...) {}

  SetImageHeight(kDefaultImageSize);
  try {
    auto height = obj.at("height").get<int>();
    SetImageHeight(height);
  } catch (...) {}
}


void RenderContext::LoadImageOffset(const nlohmann::json& obj) {
  SetImageOffsetX(0);
  SetImageOffsetY(0);
  auto offset_x = obj.at("offset")[0].get<int>();
  auto offset_y = obj.at("offset")[1].get<int>();
  offset_x = std::max(std::min(offset_x, RenderContext::kMaxImageSize / 2), -RenderContext::kMaxImageSize / 2);
  offset_y = std::max(std::min(offset_y, RenderContext::kMaxImageSize / 2), -RenderContext::kMaxImageSize / 2);
  SetImageOffsetX(offset_x);
  SetImageOffsetY(offset_y);
}


void RenderContext::LoadVisibleRange(const nlohmann::json& obj) {
  SetVisibleRange(VisibleRange::kUpper);
  auto range = obj.at("visible_semi_sphere").get<VisibleRange>();
  SetVisibleRange(range);
}


void RenderContext::LoadIntensity(const nlohmann::json& obj) {
  SetIntensity(kDefaultIntensity);
  auto f = obj.at("intensity_factor").get<float>();
  f = std::max(std::min(f, RenderContext::kMaxIntensity), RenderContext::kMinIntensity);
  SetIntensity(f);
}


void RenderContext::LoadRenderSplitter(const nlohmann::json& obj) {
  try {
    obj.at("splitter").get_to(splitter_);
  } catch (...) {}
}


void RenderContext::LoadGridLines(const nlohmann::json& obj) {
  try {
    if (!obj.at("elevation_grid").is_array()) {
      throw nlohmann::detail::other_error::create(-1, "elevation_grid must be an array!", obj);
    }

    for (const auto& l : obj.at("elevation_grid")) {
      auto curr_line = l.get<GridLine>();
      elevation_grid_.emplace_back(curr_line);
    }
  } catch (...) {}

  try {
    if (!obj.at("radius_grid").is_array()) {
      throw nlohmann::detail::other_error::create(-1, "radius_grid must be an array!", obj);
    }

    for (const auto& l : obj.at("radius_grid")) {
      auto curr_line = l.get<GridLine>();
      radius_rid_.emplace_back(curr_line);
    }
  } catch (...) {}
}

}  // namespace icehalo
