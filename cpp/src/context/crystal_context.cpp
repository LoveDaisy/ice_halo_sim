#include "context/crystal_context.h"

#include <algorithm>
#include <limits>

#include "context.h"
#include "rapidjson/document.h"
#include "rapidjson/pointer.h"

namespace icehalo {

using rapidjson::Pointer;
using CrystalParser = std::function<CrystalPtrU(const rapidjson::Value&)>;


CrystalPtrU ParseCrystalHexPrism(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }
  auto h = static_cast<float>(p->GetDouble());
  return Crystal::CreateHexPrism(h);
}


CrystalPtrU ParseCrystalHexPyramid(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 3) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    auto h3 = static_cast<float>((*p)[2].GetDouble());
    return Crystal::CreateHexPyramid(h1, h2, h3);
  } else if (p->Size() == 5) {
    int i1 = (*p)[0].GetInt();
    int i2 = (*p)[1].GetInt();
    auto h1 = static_cast<float>((*p)[2].GetDouble());
    auto h2 = static_cast<float>((*p)[3].GetDouble());
    auto h3 = static_cast<float>((*p)[4].GetDouble());
    return Crystal::CreateHexPyramid(i1, i2, h1, h2, h3);
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    return Crystal::CreateHexPyramid(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> number doesn't match!");
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU ParseCrystalHexPyramidStackHalf(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 7) {
    int upper_idx1 = (*p)[0].GetInt();
    int upper_idx2 = (*p)[1].GetInt();
    int lower_idx1 = (*p)[2].GetInt();
    int lower_idx2 = (*p)[3].GetInt();
    auto h1 = static_cast<float>((*p)[4].GetDouble());
    auto h2 = static_cast<float>((*p)[5].GetDouble());
    auto h3 = static_cast<float>((*p)[6].GetDouble());
    return Crystal::CreateHexPyramidStackHalf(upper_idx1, upper_idx2, lower_idx1, lower_idx2, h1, h2, h3);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> number doesn't match!");
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU ParseCrystalCubicPyramid(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else if (p->Size() == 2) {
    auto h1 = static_cast<float>((*p)[0].GetDouble());
    auto h2 = static_cast<float>((*p)[1].GetDouble());
    return Crystal::CreateCubicPyramid(h1, h2);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> number doesn't match!");
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU ParseCrystalIrregularHexPrism(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray() || p->Size() != 7) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    auto d1 = static_cast<float>((*p)[0].GetDouble());
    auto d2 = static_cast<float>((*p)[1].GetDouble());
    auto d3 = static_cast<float>((*p)[2].GetDouble());
    auto d4 = static_cast<float>((*p)[3].GetDouble());
    auto d5 = static_cast<float>((*p)[4].GetDouble());
    auto d6 = static_cast<float>((*p)[5].GetDouble());
    auto h = static_cast<float>((*p)[6].GetDouble());

    float dist[6] = { d1, d2, d3, d4, d5, d6 };
    return Crystal::CreateIrregularHexPrism(dist, h);
  }
}


CrystalPtrU ParseCrystalIrregularHexPyramid(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsArray()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
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

    return Crystal::CreateIrregularHexPyramid(dist, idx, height);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> number doesn't match!");
    throw std::invalid_argument(msg_buffer);
  }
}


CrystalPtrU ParseCrystalCustom(const rapidjson::Value& c) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];
  const auto* p = Pointer("/parameter").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "models/%s", p->GetString());
    std::FILE* file = std::fopen(msg_buffer, "r");
    if (!file) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.parameter> cannot open model file!");
      throw std::invalid_argument(msg_buffer);
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

    return Crystal::CreateCustomCrystal(vertexes, faces);
  }
}


std::unordered_map<std::string, CrystalParser>& GetCrystalParsers() {
  static std::unordered_map<std::string, CrystalParser> crystal_parsers = {
    { "HexPrism", ParseCrystalHexPrism },
    { "HexPyramid", ParseCrystalHexPyramid },
    { "HexPyramidStackHalf", ParseCrystalHexPyramidStackHalf },
    { "CubicPyramid", ParseCrystalCubicPyramid },
    { "IrregularHexPrism", ParseCrystalIrregularHexPrism },
    { "IrregularHexPyramid", ParseCrystalIrregularHexPyramid },
    { "Custom", ParseCrystalCustom },
  };
  return crystal_parsers;
}


AxisDistribution ParseCrystalAxis(const rapidjson::Value& c) {
  using math::Distribution;

  AxisDistribution axis{};
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  // Start parsing zenith settings.
  const auto* p = Pointer("/zenith/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.zenith.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "gauss") {
    axis.latitude_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.latitude_dist = Distribution::kUniform;
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.zenith.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/zenith/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.zenith.mean> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.latitude_mean = static_cast<float>(90 - p->GetDouble());
  }

  p = Pointer("/zenith/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.zenith.std> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.latitude_std = static_cast<float>(p->GetDouble());
  }

  // Start parsing azimuth settings.
  axis.azimuth_dist = math::Distribution::kUniform;
  axis.azimuth_mean = 0;
  axis.azimuth_std = 360;
  p = Pointer("/azimuth").Get(c);
  if (p == nullptr || !p->IsObject()) {
    std::fprintf(stderr, "<crystal.azimuth> cannot recognize! Use default.\n");
  } else {
    p = Pointer("/azimuth/type").Get(c);
    if (p == nullptr || !p->IsString()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.azimuth.type> cannot recognize!");
      throw std::invalid_argument(msg_buffer);
    } else if (*p == "gauss") {
      axis.azimuth_dist = Distribution::kGaussian;
    } else if (*p == "uniform") {
      axis.azimuth_dist = Distribution::kUniform;
    } else {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.azimuth.type> cannot recognize!");
      throw std::invalid_argument(msg_buffer);
    }

    p = Pointer("/azimuth/mean").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.azimuth.mean> cannot recognize!");
      throw std::invalid_argument(msg_buffer);
    } else {
      axis.azimuth_mean = static_cast<float>(p->GetDouble());
    }

    p = Pointer("/azimuth/std").Get(c);
    if (p == nullptr || !p->IsNumber()) {
      std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.azimuth.std> cannot recognize!");
      throw std::invalid_argument(msg_buffer);
    } else {
      axis.azimuth_std = static_cast<float>(p->GetDouble());
    }
  }

  // Start parsing roll settings.
  p = Pointer("/roll/type").Get(c);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.roll.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else if (*p == "gauss") {
    axis.roll_dist = Distribution::kGaussian;
  } else if (*p == "uniform") {
    axis.roll_dist = Distribution::kUniform;
  } else {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.roll.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/roll/mean").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.roll.mean> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_mean = static_cast<float>(p->GetDouble());
  }

  p = Pointer("/roll/std").Get(c);
  if (p == nullptr || !p->IsNumber()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.roll.std> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  } else {
    axis.roll_std = static_cast<float>(p->GetDouble());
  }

  return axis;
}


CrystalContext::CrystalContext() : id_(kInvalidId), crystal_{}, axis_{} {}


CrystalContext::CrystalContext(int id, AxisDistribution axis, CrystalPtrU g)
    : id_(id), crystal_(std::move(g)), axis_(axis) {}


int CrystalContext::GetId() const {
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

  return math::RandomSampler::SampleInt(face_prob_buf.get(), total_faces);
}


void CrystalContext::SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) {}


void CrystalContext::LoadFromJson(const rapidjson::Value& root) {
  constexpr size_t kMsgBufferSize = 256;
  char msg_buffer[kMsgBufferSize];

  auto p = Pointer("/type").Get(root);
  if (p == nullptr || !p->IsString()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  auto& crystal_parsers = GetCrystalParsers();
  std::string type(root["type"].GetString());
  if (crystal_parsers.find(type) == crystal_parsers.end()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.type> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  p = Pointer("/id").Get(root);
  if (p == nullptr || !p->IsInt()) {
    std::snprintf(msg_buffer, kMsgBufferSize, "<crystal.id> cannot recognize!");
    throw std::invalid_argument(msg_buffer);
  }

  axis_ = ParseCrystalAxis(root);
  id_ = p->GetInt();
  crystal_ = crystal_parsers[type](root);
}

}  // namespace icehalo
