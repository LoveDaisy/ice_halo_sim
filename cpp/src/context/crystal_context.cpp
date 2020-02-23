#include "context/crystal_context.hpp"

#include <algorithm>
#include <limits>

#include "core/enum_map.hpp"
#include "rapidjson/document.h"
#include "rapidjson/pointer.h"
#include "util/log.hpp"

namespace icehalo {

using rapidjson::Pointer;

void CrystalContext::ParseHexPrism(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  }
  auto h = static_cast<float>(p->GetDouble());
  h_param_[0] = h;
  crystal_ = Crystal::CreateHexPrism(h);
}


void CrystalContext::ParseHexPyramid(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else if (p->Size() == 3) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    auto h3 = static_cast<float>((*p)[2].GetDouble());
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    crystal_ = Crystal::CreateHexPyramid(h1, h2, h3);
  } else if (p->Size() == 5) {
    int i1 = (*p)[0].GetInt();
    int i2 = (*p)[1].GetInt();
    auto h1 = static_cast<float>((*p)[2].GetDouble());
    auto h2 = static_cast<float>((*p)[3].GetDouble());
    auto h3 = static_cast<float>((*p)[4].GetDouble());
    idx_param_[0] = i1;
    idx_param_[1] = i2;
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    crystal_ = Crystal::CreateHexPyramid(i1, i2, h1, h2, h3);
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    idx_param_[0] = upper_idx1;
    idx_param_[1] = upper_idx2;
    idx_param_[2] = lower_idx1;
    idx_param_[3] = lower_idx2;
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    crystal_ = Crystal::CreateHexPyramid(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseHexPyramidStackHalf(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    idx_param_[0] = upper_idx1;
    idx_param_[1] = upper_idx2;
    idx_param_[2] = lower_idx1;
    idx_param_[3] = lower_idx2;
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    crystal_ = Crystal::CreateHexPyramidStackHalf(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseCubicPyramid(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else if (p->Size() == 2) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    h_param_[0] = h1;
    h_param_[1] = h2;
    crystal_ = Crystal::CreateCubicPyramid(h1, h2);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseIrregularHexPrism(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray() || p->Size() != 7) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else {
    auto d1 = static_cast<float>((*p)[0].GetDouble());
    auto d2 = static_cast<float>((*p)[1].GetDouble());
    auto d3 = static_cast<float>((*p)[2].GetDouble());
    auto d4 = static_cast<float>((*p)[3].GetDouble());
    auto d5 = static_cast<float>((*p)[4].GetDouble());
    auto d6 = static_cast<float>((*p)[5].GetDouble());
    auto h = static_cast<float>((*p)[6].GetDouble());

    float dist[6] = { d1, d2, d3, d4, d5, d6 };

    d_param_[0] = d1;
    d_param_[1] = d2;
    d_param_[2] = d3;
    d_param_[3] = d4;
    d_param_[4] = d5;
    d_param_[5] = d6;
    h_param_[0] = h;
    crystal_ = Crystal::CreateIrregularHexPrism(dist, h);
  }
}


void CrystalContext::ParseIrregularHexPyramid(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else if (p->Size() == 13) {
    auto d1 = static_cast<float>((*p)[0].GetDouble());
    auto d2 = static_cast<float>((*p)[1].GetDouble());
    auto d3 = static_cast<float>((*p)[2].GetDouble());
    auto d4 = static_cast<float>((*p)[3].GetDouble());
    auto d5 = static_cast<float>((*p)[4].GetDouble());
    auto d6 = static_cast<float>((*p)[5].GetDouble());
    int i1 = (*p)[6].GetInt();
    int i2 = (*p)[7].GetInt();
    int i3 = (*p)[8].GetInt();
    int i4 = (*p)[9].GetInt();
    auto h1 = static_cast<float>((*p)[10].GetDouble());
    auto h2 = static_cast<float>((*p)[11].GetDouble());
    auto h3 = static_cast<float>((*p)[12].GetDouble());

    float dist[6] = { d1, d2, d3, d4, d5, d6 };
    int idx[4] = { i1, i2, i3, i4 };
    float height[3] = { h1, h2, h3 };

    d_param_[0] = d1;
    d_param_[1] = d2;
    d_param_[2] = d3;
    d_param_[3] = d4;
    d_param_[4] = d5;
    d_param_[5] = d6;
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    idx_param_[0] = i1;
    idx_param_[1] = i2;
    idx_param_[2] = i3;
    idx_param_[3] = i4;
    crystal_ = Crystal::CreateIrregularHexPyramid(dist, idx, height);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseCustomCrystal(const rapidjson::Value& c) {
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    throw std::invalid_argument("<crystal.parameter> cannot recognize!");
  } else {
    std::FILE* file = std::fopen(p->GetString(), "r");
    if (!file) {
      throw std::invalid_argument("<crystal.parameter> cannot open model file!");
    }

    std::vector<math::Vec3f> vertexes;
    std::vector<math::TriangleIdx> faces;
    float v_buf[3];
    int f_buf[3];
    int curr_char;
    while ((curr_char = std::fgetc(file)) != EOF) {
      switch (curr_char) {
        case 'v':
        case 'V':
          std::fscanf(file, "%f %f %f", v_buf + 0, v_buf + 1, v_buf + 2);
          vertexes.emplace_back(v_buf);
          break;
        case 'f':
        case 'F':
          std::fscanf(file, "%d %d %d", f_buf + 0, f_buf + 1, f_buf + 2);
          faces.emplace_back(f_buf[0] - 1, f_buf[1] - 1, f_buf[2] - 1);
          break;
        default:
          break;
      }
    }
    std::fclose(file);

    file_param_ = p->GetString();
    crystal_ = Crystal::CreateCustomCrystal(vertexes, faces);
  }
}


AxisDistribution ParseCrystalAxis(const rapidjson::Value& c) {
  using math::Distribution;

  AxisDistribution axis{};

  // Start parsing zenith settings.
  const auto* p = Pointer("/zenith/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    throw std::invalid_argument("<zenith.type> cannot recognize!");
  } else if (*p == "gauss") {
    axis.latitude_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.latitude_dist = Distribution::kUniform;
  } else {
    throw std::invalid_argument("<zenith.type> cannot recognize!");
  }

  p = Pointer("/zenith/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<zenith.mean> cannot recognize!");
  } else {
    axis.latitude_mean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/zenith/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<zenith.std> cannot recognize!");
  } else {
    axis.latitude_std = static_cast<float>(p->GetDouble());
  }

  // Start parsing azimuth settings.
  axis.azimuth_dist = math::Distribution::kUniform;
  axis.azimuth_mean = 0;
  axis.azimuth_std = 360;
  p = Pointer("/azimuth").Get(c);
  if (p == nullptr || !p->IsObject()) {
    LOG_VERBOSE("<azimuth> cannot recognize! Use default.");
  } else {
    p = Pointer("/azimuth/type").Get(c);
    if (p == nullptr || !p->IsString()) {
      throw std::invalid_argument("<azimuth.type> cannot recognize!");
    } else if (*p == "gauss") {
      axis.azimuth_dist = Distribution::kGaussian;
    } else if (*p == "uniform") {
      axis.azimuth_dist = Distribution::kUniform;
    } else {
      throw std::invalid_argument("<azimuth.type> cannot recognize!");
    }

    p = Pointer("/azimuth/mean").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      throw std::invalid_argument("<azimuth.mean> cannot recognize!");
    } else {
      axis.azimuth_mean = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/azimuth/std").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      throw std::invalid_argument("<azimuth.std> cannot recognize!");
    } else {
      axis.azimuth_std = static_cast<float>(p->GetDouble());
    }
  }

  // Start parsing roll settings.
  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    throw std::invalid_argument("<roll.type> cannot recognize!");
  } else if (*p == "gauss") {
    axis.roll_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.roll_dist = Distribution::kUniform;
  } else {
    throw std::invalid_argument("<roll.type> cannot recognize!");
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<roll.mean> cannot recognize!");
  } else {
    axis.roll_mean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    throw std::invalid_argument("<roll.std> cannot recognize!");
  } else {
    axis.roll_std = static_cast<float>(p->GetDouble());
  }

  return axis;
}


CrystalContext::CrystalContext()
    : id_(kInvalidId), crystal_{}, axis_{}, idx_param_{}, h_param_{}, d_param_{}, file_param_{} {}


CrystalContextPtrU CrystalContext::CreateDefault() {
  CrystalContextPtrU ctx{ new CrystalContext };
  return ctx;
}


ShortIdType CrystalContext::GetId() const {
  return id_;
}


const Crystal* CrystalContext::GetCrystal() const {
  return crystal_.get();
}


AxisDistribution CrystalContext::GetAxisDistribution() const {
  return axis_;
}


int CrystalContext::RandomSampleFace(const float* ray_dir) const {
  int total_faces = crystal_->TotalFaces();
  std::unique_ptr<float[]> face_prob_buf{ new float[total_faces] };
  const auto* face_norm = crystal_->GetFaceNorm();
  const auto* face_area = crystal_->GetFaceArea();

  float sum = 0;
  for (int k = 0; k < total_faces; k++) {
    face_prob_buf[k] = 0;
    if (!std::isnan(face_norm[k * 3 + 0]) && face_area[k] > 0) {
      face_prob_buf[k] = std::max(-math::Dot3(face_norm + k * 3, ray_dir) * face_area[k], 0.0f);
      sum += face_prob_buf[k];
    }
  }
  for (int k = 0; k < total_faces; k++) {
    face_prob_buf[k] /= sum;
  }

  return static_cast<ShortIdType>(math::RandomSampler::SampleInt(face_prob_buf.get(), total_faces));
}


void CrystalContext::SaveHexPrismParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "HexPrism", allocator);
  Pointer("/parameter").Set(root, h_param_[0], allocator);
}


void CrystalContext::SaveHexPyramidH3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "HexPyramid", allocator);
  Pointer("/parameter/0").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[2], allocator);
}


void CrystalContext::SaveHexPyramidI2H3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "HexPyramid", allocator);
  Pointer("/parameter/0").Set(root, idx_param_[0], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[2], allocator);
}


void CrystalContext::SaveHexPyramidI4H3Param(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "HexPyramid", allocator);
  Pointer("/parameter/0").Set(root, idx_param_[0], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[1], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[2], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[3], allocator);
  Pointer("/parameter/-").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[2], allocator);
}


void CrystalContext::SaveHexPyramidStackHalfParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "HexPyramidStackHalf", allocator);
  Pointer("/parameter/0").Set(root, idx_param_[0], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[1], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[2], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[3], allocator);
  Pointer("/parameter/-").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[2], allocator);
}


void CrystalContext::SaveIrregularHexPrismParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "IrregularHexPrism", allocator);
  Pointer("/parameter/0").Set(root, d_param_[0], allocator);
  Pointer("/parameter/-").Set(root, d_param_[1], allocator);
  Pointer("/parameter/-").Set(root, d_param_[2], allocator);
  Pointer("/parameter/-").Set(root, d_param_[3], allocator);
  Pointer("/parameter/-").Set(root, d_param_[4], allocator);
  Pointer("/parameter/-").Set(root, d_param_[5], allocator);
  Pointer("/parameter/-").Set(root, h_param_[0], allocator);
}


void CrystalContext::SaveIrregularHexPyramidParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "IrregularHexPyramid", allocator);
  Pointer("/parameter/0").Set(root, d_param_[0], allocator);
  Pointer("/parameter/-").Set(root, d_param_[1], allocator);
  Pointer("/parameter/-").Set(root, d_param_[2], allocator);
  Pointer("/parameter/-").Set(root, d_param_[3], allocator);
  Pointer("/parameter/-").Set(root, d_param_[4], allocator);
  Pointer("/parameter/-").Set(root, d_param_[5], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[0], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[1], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[2], allocator);
  Pointer("/parameter/-").Set(root, idx_param_[3], allocator);
  Pointer("/parameter/-").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
  Pointer("/parameter/-").Set(root, h_param_[2], allocator);
}


void CrystalContext::SaveCubicPyramidParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "CubicPyramid", allocator);
  Pointer("/parameter/0").Set(root, h_param_[0], allocator);
  Pointer("/parameter/-").Set(root, h_param_[1], allocator);
}


void CrystalContext::SaveCustomCrystalParam(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  Pointer("/type").Set(root, "Custom", allocator);
  Pointer("/parameter").Set(root, file_param_.c_str(), allocator);
}


void CrystalContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {
  root.Clear();

  Pointer("/id").Set(root, id_, allocator);

  using ParamSaver = std::function<void(CrystalContext*, rapidjson::Value&, rapidjson::Value::AllocatorType&)>;
  EnumMap<CrystalType, ParamSaver> param_saver{
    { CrystalType::kPrism, &CrystalContext::SaveHexPrismParam },
    { CrystalType::kPyramid_H3, &CrystalContext::SaveHexPyramidH3Param },
    { CrystalType::kPyramid_I2H3, &CrystalContext::SaveHexPyramidI2H3Param },
    { CrystalType::kPyramid_I4H3, &CrystalContext::SaveHexPyramidI4H3Param },
    { CrystalType::kPyramidStackHalf, &CrystalContext::SaveHexPyramidStackHalfParam },
    { CrystalType::kIrregularPrism, &CrystalContext::SaveIrregularHexPrismParam },
    { CrystalType::kIrregularPyramid, &CrystalContext::SaveIrregularHexPyramidParam },
    { CrystalType::kCubicPyramid, &CrystalContext::SaveCubicPyramidParam },
    { CrystalType::kCustom, &CrystalContext::SaveCustomCrystalParam },
  };

  if (param_saver.count(crystal_->GetType())) {
    param_saver[crystal_->GetType()](this, root, allocator);
  }

  Pointer("/zenith/mean").Set(root, 90.0f - axis_.latitude_mean, allocator);
  Pointer("/zenith/std").Set(root, axis_.latitude_std, allocator);
  switch (axis_.latitude_dist) {
    case math::Distribution::kGaussian:
      Pointer("/zenith/type").Set(root, "gauss", allocator);
      break;
    case math::Distribution::kUniform:
      Pointer("/zenith/type").Set(root, "uniform", allocator);
      break;
  }

  Pointer("/roll/mean").Set(root, axis_.roll_mean, allocator);
  Pointer("/roll/std").Set(root, axis_.roll_std, allocator);
  switch (axis_.roll_dist) {
    case math::Distribution::kGaussian:
      Pointer("/roll/type").Set(root, "gauss", allocator);
      break;
    case math::Distribution::kUniform:
      Pointer("/roll/type").Set(root, "uniform", allocator);
      break;
  }

  Pointer("/azimuth/mean").Set(root, axis_.azimuth_mean, allocator);
  Pointer("/azimuth/std").Set(root, axis_.azimuth_std, allocator);
  switch (axis_.azimuth_dist) {
    case math::Distribution::kGaussian:
      Pointer("/azimuth/type").Set(root, "gauss", allocator);
      break;
    case math::Distribution::kUniform:
      Pointer("/azimuth/type").Set(root, "uniform", allocator);
      break;
  }
}


void CrystalContext::LoadFromJson(const rapidjson::Value& root) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(root);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  using CrystalParser = std::function<void(CrystalContext*, const rapidjson::Value&)>;
  std::unordered_map<std::string, CrystalParser> crystal_parsers{
    { "HexPrism", &CrystalContext::ParseHexPrism },
    { "HexPyramid", &CrystalContext::ParseHexPyramid },
    { "HexPyramidStackHalf", &CrystalContext::ParseHexPyramidStackHalf },
    { "CubicPyramid", &CrystalContext::ParseCubicPyramid },
    { "IrregularHexPrism", &CrystalContext::ParseIrregularHexPrism },
    { "IrregularHexPyramid", &CrystalContext::ParseIrregularHexPyramid },
    { "Custom", &CrystalContext::ParseCustomCrystal },
  };

  std::string type(root["type"].GetString());
  if (crystal_parsers.find(type) == crystal_parsers.end()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/id").Get(root);
  if (p == nullptr || !p->IsUint()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.id> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  axis_ = ParseCrystalAxis(root);
  id_ = p->GetUint();
  crystal_parsers[type](this, root);
}

}  // namespace icehalo
