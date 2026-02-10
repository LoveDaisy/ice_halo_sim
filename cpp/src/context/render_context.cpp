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
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "width", line.width)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "alpha", line.alpha)
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(obj, "color", line.color, 3)
}


void to_json(nlohmann::json& obj, const GridLine& line) {
  obj = line.line_specifier;
  obj["value"] = line.value;
}


void from_json(const nlohmann::json& obj, GridLine& line) {
  obj.get_to(line.line_specifier);
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "value", line.value)
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
  intensity_ = std::clamp(intensity, kMinIntensity, kMaxIntensity);
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
  offset_x_ = std::clamp(offset_x, -kMaxImageSize / 2, kMaxImageSize / 2);
}


void RenderContext::SetImageOffsetY(int offset_y) {
  offset_y_ = std::clamp(offset_y, -kMaxImageSize / 2, kMaxImageSize / 2);
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
    default:
      return -1;
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
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "color_compact_level", ColorCompactLevel, SetColorCompactLevel)

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
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "width", int, SetImageWidth)

  SetImageHeight(kDefaultImageSize);
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "height", int, SetImageHeight)
}


void RenderContext::LoadImageOffset(const nlohmann::json& obj) {
  int offset[2]{};
  JSON_CHECK_AND_UPDATE_ARRAY_VALUE(obj, "offset", offset, 2)
  SetImageOffsetX(offset[0]);
  SetImageOffsetY(offset[1]);
}


void RenderContext::LoadVisibleRange(const nlohmann::json& obj) {
  SetVisibleRange(VisibleRange::kUpper);
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "visible_semi_sphere", VisibleRange, SetVisibleRange)
}


void RenderContext::LoadIntensity(const nlohmann::json& obj) {
  SetIntensity(kDefaultIntensity);
  JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, "intensity_factor", float, SetIntensity)
}


void RenderContext::LoadRenderSplitter(const nlohmann::json& obj) {
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "splitter", splitter_)
}


void RenderContext::LoadGridLines(const nlohmann::json& obj) {
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "elevation_grid", elevation_grid_)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "radius_grid", radius_rid_)
}

}  // namespace icehalo
