#include "context/render_context.hpp"

#include <algorithm>

#include "process/render.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "util/log.hpp"


namespace icehalo {

using rapidjson::Pointer;

RenderSplitter::RenderSplitter() : type(RenderSplitterType::kNone), top_halo_num(0), crystal_filters{} {}


void RenderSplitter::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  switch (type) {
    case RenderSplitterType::kTopHalo:
      Pointer("/type").Set(root, "top_halo", allocator);
      Pointer("/param").Set(root, top_halo_num, allocator);
      break;
    case RenderSplitterType::kFilter: {
      char id_buf[128];
      Pointer("/type").Set(root, "filter", allocator);
      for (size_t i = 0; i < crystal_filters.size(); i++) {
        const auto& ids = crystal_filters[i];
        for (size_t j = 0; j < ids.size(); j++) {
          std::snprintf(id_buf, 128, "/param/%zu/%zu", i, j);
          Pointer(id_buf).Set(root, ids[j], allocator);
        }
      }
    } break;
    case RenderSplitterType::kNone:
      break;
  }
}


void RenderSplitter::LoadFromJson(const rapidjson::Value& root) {
  const auto* p = Pointer("/type").Get(root);
  if (p && p->IsString() && *p == "top_halo") {
    type = RenderSplitterType::kTopHalo;
  } else if (p && p->IsString() && *p == "filter") {
    type = RenderSplitterType::kFilter;
  }

  p = Pointer("/param").Get(root);
  if (p && p->IsInt() && type == RenderSplitterType::kTopHalo) {
    top_halo_num = std::min(std::max(p->GetInt(), 0), RenderContext::kMaxTopHaloNumber);
  } else if (p && p->IsArray() && type == RenderSplitterType::kFilter) {
    std::vector<std::vector<ShortIdType>> new_crystal_filters;
    for (const auto& pi : p->GetArray()) {
      if (!pi.IsArray() || pi.GetArray().Size() % 2 != 0) {
        LOG_VERBOSE("crystal filter param not recognize!");
        continue;
      }
      std::vector<ShortIdType> tmp_ids{};
      for (const auto& fid : pi.GetArray()) {
        if (!fid.IsUint()) {
          LOG_VERBOSE("crystal filter param not recognize!");
          continue;
        }
        tmp_ids.emplace_back(fid.GetUint());
      }
      if (!tmp_ids.empty()) {
        new_crystal_filters.emplace_back(tmp_ids);
      }
    }
    if (!new_crystal_filters.empty()) {
      crystal_filters = new_crystal_filters;
    }
  }
}


constexpr float LineSpecifier::kDefaultColor[3];
constexpr float LineSpecifier::kMinWidth;


LineSpecifier::LineSpecifier() : LineSpecifier(LineType::kSolid, kDefaultWidth, kDefaultColor) {}


LineSpecifier::LineSpecifier(LineType type, float width, const float color[3])
    : type(type), width(width), color{ color[0], color[1], color[2] } {}


void LineSpecifier::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  switch (type) {
    case LineType::kSolid:
      Pointer("/type").Set(root, "solid", allocator);
      break;
    case LineType::kDashed:
      Pointer("/type").Set(root, "dashed", allocator);
      break;
  }

  Pointer("/width").Set(root, width, allocator);

  Pointer("/color/0").Set(root, color[0], allocator);
  Pointer("/color/-").Set(root, color[1], allocator);
  Pointer("/color/-").Set(root, color[2], allocator);
}


void LineSpecifier::LoadFromJson(const rapidjson::Value& root) {
  type = LineType::kSolid;
  const auto* p = Pointer("/type").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Line specifier missing <type>. Use default solid type.");
  } else if (!p->IsString()) {
    LOG_VERBOSE("Line specifier <type> is not a string. Ignore it.");
  } else if (*p == "solid") {
    type = LineType::kSolid;
  } else if (*p == "dashed") {
    type = LineType::kDashed;
  } else {
    LOG_VERBOSE("Line specifier <type> cannot recognize! Use default solid type.");
  }

  width = kDefaultWidth;
  p = Pointer("/width").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Line specifier missing <width>. Use default %.2f.", kDefaultWidth);
  } else if (!p->IsNumber()) {
    LOG_VERBOSE("Line specifier <width> is not a number. Use default %.2f.", kDefaultWidth);
  } else {
    auto w = p->GetDouble();
    width = std::max(static_cast<float>(w), kMinWidth);
  }

  std::memcpy(color, kDefaultColor, sizeof(color));
  p = Pointer("/color").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Line specifier missing <color>. Use default color!");
  } else if (!p->IsArray()) {
    LOG_VERBOSE("Line specifier <color> is not an array. Use default color!");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    LOG_VERBOSE("Line Specifier <color> cannot be recognized. Use default color!");
  } else {
    auto pa = p->GetArray();
    color[0] = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    color[1] = static_cast<float>(std::min(std::max(pa[1].GetDouble(), 0.0), 1.0));
    color[2] = static_cast<float>(std::min(std::max(pa[2].GetDouble(), 0.0), 1.0));
  }
}


RenderContext::RenderContext()
    : ray_color_{ 1.0f, 1.0f, 1.0f }, background_color_{ 0.0f, 0.0f, 0.0f }, intensity_(1.0f), image_width_(0),
      image_height_(0), offset_x_(0), offset_y_(0), visible_range_(VisibleRange::kUpper),
      color_compact_level_(ColorCompactLevel::kTrueColor), splitter_{} {}


RenderContextPtrU RenderContext::CreateDefault() {
  RenderContextPtrU render_ctx{ new RenderContext };
  return render_ctx;
}


constexpr float RenderContext::kMinIntensity;
constexpr float RenderContext::kMaxIntensity;
constexpr int RenderContext::kMaxImageSize;
constexpr int RenderContext::kMaxTopHaloNumber;


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


void RenderContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  root.Clear();
  SaveColorConfig(root, allocator);
  SaveIntensity(root, allocator);
  SaveImageSize(root, allocator);
  SaveImageOffset(root, allocator);
  SaveVisibleRange(root, allocator);
  SaveRenderSplitter(root, allocator);
  SaveGridLines(root, allocator);
}


void RenderContext::SaveColorConfig(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  switch (color_compact_level_) {
    case ColorCompactLevel::kTrueColor:
      Pointer("/ray_compact_level").Set(root, "true_color", allocator);
      break;
    case ColorCompactLevel::kMonochrome:
      Pointer("/ray_compact_level").Set(root, "monochrome", allocator);
      break;
    case ColorCompactLevel::kLowQuality:
      Pointer("/ray_compact_level").Set(root, "low_quality", allocator);
      break;
  }

  Pointer("/background_color/0").Set(root, background_color_[0], allocator);
  Pointer("/background_color/-").Set(root, background_color_[1], allocator);
  Pointer("/background_color/-").Set(root, background_color_[2], allocator);

  if (ray_color_[0] < 0) {
    Pointer("/ray_color").Set(root, "real", allocator);
  } else {
    Pointer("/ray_color/0").Set(root, ray_color_[0], allocator);
    Pointer("/ray_color/-").Set(root, ray_color_[1], allocator);
    Pointer("/ray_color/-").Set(root, ray_color_[2], allocator);
  }
}


void RenderContext::SaveIntensity(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/intensity_factor").Set(root, intensity_, allocator);
}


void RenderContext::SaveImageSize(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/width").Set(root, image_width_, allocator);
  Pointer("/height").Set(root, image_height_, allocator);
}


void RenderContext::SaveImageOffset(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/offset/0").Set(root, offset_x_, allocator);
  Pointer("/offset/-").Set(root, offset_y_, allocator);
}


void RenderContext::SaveVisibleRange(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  switch (visible_range_) {
    case VisibleRange::kLower:
      Pointer("/visible_semi_sphere").Set(root, "lower", allocator);
      break;
    case VisibleRange::kFront:
      Pointer("/visible_semi_sphere").Set(root, "camera", allocator);
      break;
    case VisibleRange::kFull:
      Pointer("/visible_semi_sphere").Set(root, "full", allocator);
      break;
    case VisibleRange::kUpper:
    default:
      Pointer("/visible_semi_sphere").Set(root, "upper", allocator);
      break;
  }
}


void RenderContext::SaveRenderSplitter(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  splitter_.SaveToJson(Pointer("/splitter").Create(root, allocator), allocator);
}


void RenderContext::SaveGridLines(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/elevation_grid/0").Create(root, allocator);
  for (auto& g : elevation_grid_) {
    auto& curr_obj = Pointer("/elevation_grid/-").Create(root, allocator);
    Pointer("/value").Set(curr_obj, g.value, allocator);
    g.line_specifier.SaveToJson(curr_obj, allocator);
  }

  Pointer("/radius_grid/0").Create(root, allocator);
  for (auto& g : radius_rid_) {
    auto& curr_obj = Pointer("/radius_grid/-").Create(root, allocator);
    Pointer("/value").Set(curr_obj, g.value, allocator);
    g.line_specifier.SaveToJson(curr_obj, allocator);
  }
}


void RenderContext::LoadFromJson(const rapidjson::Value& root) {
  LoadColorConfig(root);
  LoadImageSize(root);
  LoadImageOffset(root);
  LoadVisibleRange(root);
  LoadIntensity(root);
  LoadRenderSplitter(root);
  LoadGridLines(root);
}


void RenderContext::LoadColorConfig(const rapidjson::Value& root) {
  SetColorCompactLevel(ColorCompactLevel::kTrueColor);
  const auto* p = Pointer("/color_compact_level").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <color_compact_level>. Use default true color!");
  } else if (!p->IsString()) {
    LOG_VERBOSE("Render config <color_compact_level> is not a string. Use default true color!");
  } else {
    if (*p == "no_compact") {
      color_compact_level_ = ColorCompactLevel::kTrueColor;
    } else if (*p == "monochrome") {
      color_compact_level_ = ColorCompactLevel::kMonochrome;
    } else if (*p == "low_quality") {
      color_compact_level_ = ColorCompactLevel::kLowQuality;
    } else {
      LOG_VERBOSE("Render config <color_compact_level> cannot recognize. Use default true color!");
    }
  }

  ResetBackgroundColor();
  p = Pointer("/background_color").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <background_color>. Use default [0,0,0]!");
  } else if (!p->IsArray()) {
    LOG_VERBOSE("Config <background_color> is not an array. Use default [0,0,0]!");
  } else if (p->Size() != 3 || !(*p)[0].IsNumber()) {
    LOG_VERBOSE("Config <background_color> cannot be recognized. Use default [0,0,0]!");
  } else {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    SetBackgroundColor(r, g, b);
  }

  ResetRayColor();
  p = Pointer("/ray_color").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Config missing <ray_color>. Use default real color!");
  } else if (!p->IsArray() && !p->IsString()) {
    LOG_VERBOSE("Config <ray_color> is not an array nor a string. Use default real color!");
  } else if (p->IsArray() && (p->Size() != 3 || !(*p)[0].IsNumber())) {
    LOG_VERBOSE("Config <ray_color> cannot be recognized. Use default real color!");
  } else if (p->IsString() && (*p) != "real") {
    LOG_VERBOSE("Config <ray_color> cannot be recognized. Use default real color!");
  } else if (p->IsArray()) {
    auto pa = p->GetArray();
    float r = static_cast<float>(std::min(std::max(pa[0].GetDouble(), 0.0), 1.0));
    float g = static_cast<float>(std::min(std::max(pa[1].GetDouble(), 0.0), 1.0));
    float b = static_cast<float>(std::min(std::max(pa[2].GetDouble(), 0.0), 1.0));
    SetRayColor(r, g, b);
  }
}


void RenderContext::LoadImageSize(const rapidjson::Value& root) {
  SetImageWidth(kDefaultImageSize);
  const auto* p = Pointer("/width").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <width>. Use default 800!");
  } else if (!p->IsInt()) {
    LOG_VERBOSE("Render config <width> is not an integer. Use default 800!");
  } else {
    SetImageWidth(p->GetInt());
  }

  SetImageHeight(kDefaultImageSize);
  p = Pointer("/height").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <height>. Use default 800!");
  } else if (!p->IsInt()) {
    LOG_VERBOSE("Render config <height> is not an integer. Use default 800!");
  } else {
    SetImageHeight(p->GetInt());
  }
}


void RenderContext::LoadImageOffset(const rapidjson::Value& root) {
  SetImageOffsetX(0);
  SetImageOffsetY(0);
  const auto* p = Pointer("/offset").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <offset>. Use default [0, 0]!");
  } else if (!p->IsArray()) {
    LOG_VERBOSE("Render config <offset> is not an array. Use default [0, 0]!");
  } else if (p->Size() != 2 || !(*p)[0].IsInt() || !(*p)[1].IsInt()) {
    LOG_VERBOSE("Config <offset> cannot be recognized. Use default [0, 0]!");
  } else {
    int offset_x = (*p)[0].GetInt();
    int offset_y = (*p)[1].GetInt();
    offset_x = std::max(std::min(offset_x, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    offset_y = std::max(std::min(offset_y, static_cast<int>(RenderContext::kMaxImageSize / 2)),
                        -static_cast<int>(RenderContext::kMaxImageSize / 2));
    SetImageOffsetX(offset_x);
    SetImageOffsetY(offset_y);
  }
}


void RenderContext::LoadVisibleRange(const rapidjson::Value& root) {
  SetVisibleRange(VisibleRange::kUpper);
  const auto* p = Pointer("/visible_semi_sphere").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <visible_semi_sphere>. Use default upper!");
  } else if (!p->IsString()) {
    LOG_VERBOSE("Render config <visible_semi_sphere> is not a string. Use default upper!");
  } else if (*p == "upper") {
    SetVisibleRange(VisibleRange::kUpper);
  } else if (*p == "lower") {
    SetVisibleRange(VisibleRange::kLower);
  } else if (*p == "camera") {
    SetVisibleRange(VisibleRange::kFront);
  } else if (*p == "full") {
    SetVisibleRange(VisibleRange::kFull);
  } else {
    LOG_VERBOSE("Render config <visible_semi_sphere> cannot be recognized. Use default upper!");
  }
}


void RenderContext::LoadIntensity(const rapidjson::Value& root) {
  SetIntensity(kDefaultIntensity);
  const auto* p = Pointer("/intensity_factor").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <intensity_factor>. Use default 1.0!");
  } else if (!p->IsNumber()) {
    LOG_VERBOSE("Render config <intensity_factor> is not a number. Use default 1.0!");
  } else {
    auto f = static_cast<float>(p->GetDouble());
    f = std::max(std::min(f, RenderContext::kMaxIntensity), RenderContext::kMinIntensity);
    SetIntensity(f);
  }
}


void RenderContext::LoadRenderSplitter(const rapidjson::Value& root) {
  const auto* p = Pointer("/splitter").Get(root);
  splitter_ = {};
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <splitter>. Use default!");
  } else if (!p->IsObject()) {
    LOG_VERBOSE("Render config <splitter> is not an object. Use default!");
  } else {
    splitter_.LoadFromJson(*p);
  }
}


void RenderContext::LoadGridLines(const rapidjson::Value& root) {
  const auto* p = Pointer("/elevation_grid").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <elevation_grid>. No elevation grid will be drawn.");
  } else if (!p->IsArray() || !(*p)[0].IsObject()) {
    LOG_VERBOSE("Render config <elevation_grid> is not an array. Ignore it.");
  } else {
    for (size_t i = 0; i < p->GetArray().Size(); i++) {
      if (!(*p)[i].HasMember("value") || !(*p)[i]["value"].IsNumber()) {
        LOG_VERBOSE("Render config <elevation_grid>[%zu] cannot recognize. Ignore it.", i);
        continue;
      }
      GridLine tmp_line;
      tmp_line.value = (*p)[i]["value"].GetDouble();
      tmp_line.line_specifier.LoadFromJson((*p)[i]);
      elevation_grid_.emplace_back(tmp_line);
    }
  }

  p = Pointer("/radius_grid").Get(root);
  if (p == nullptr) {
    LOG_VERBOSE("Render config missing <radius_grid>. No elevation grid will be drawn.");
  } else if (!p->IsArray() || !(*p)[0].IsObject()) {
    LOG_VERBOSE("Render config <radius_grid> is not an array. Ignore it.");
  } else {
    for (size_t i = 0; i < p->GetArray().Size(); i++) {
      if (!(*p)[i].HasMember("value") || !(*p)[i]["value"].IsNumber()) {
        LOG_VERBOSE("Render config <radius_grid>[%zu] cannot recognize. Ignore it.", i);
        continue;
      }
      GridLine tmp_line;
      tmp_line.value = (*p)[i]["value"].GetDouble();
      tmp_line.line_specifier.LoadFromJson((*p)[i]);
      radius_rid_.emplace_back(tmp_line);
    }
  }
}

}  // namespace icehalo
