#include "context/crystal_context.hpp"

#include <algorithm>
#include <sstream>

#include "util/log.hpp"

namespace icehalo {

void CrystalContext::ParseHexPrism(const nlohmann::json& obj) {
  h_param_[0] = obj.at("parameter").get<float>();
  crystal_ = Crystal::CreateHexPrism(h_param_[0]);
}


void CrystalContext::ParseHexPyramid(const nlohmann::json& obj) {
  auto p_obj = obj.at("parameter");
  if (!p_obj.is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal parameter must be array!", obj);
  }
  if (p_obj.size() == 3) {
    auto h1 = p_obj[0].get<float>();
    auto h2 = p_obj[1].get<float>();
    auto h3 = p_obj[2].get<float>();
    h_param_[0] = h1;
    h_param_[1] = h2;
    h_param_[2] = h3;
    crystal_ = Crystal::CreateHexPyramid(h1, h2, h3);
  } else if (p_obj.size() == 5) {
    if (p_obj[0].is_number_integer()) {  // Using Miller Index
      int i1 = p_obj[0].get<int>();
      int i2 = p_obj[1].get<int>();
      auto h1 = p_obj[2].get<float>();
      auto h2 = p_obj[3].get<float>();
      auto h3 = p_obj[4].get<float>();
      idx_param_[0] = i1;
      idx_param_[1] = i2;
      h_param_[0] = h1;
      h_param_[1] = h2;
      h_param_[2] = h3;
      crystal_ = Crystal::CreateHexPyramid(i1, i2, h1, h2, h3);
    } else {  // Using wedge angle
      auto a1 = p_obj[0].get<float>();
      auto a2 = p_obj[1].get<float>();
      auto h1 = p_obj[2].get<float>();
      auto h2 = p_obj[3].get<float>();
      auto h3 = p_obj[4].get<float>();
      a_param_[0] = a1;
      a_param_[1] = a2;
      h_param_[0] = h1;
      h_param_[1] = h2;
      h_param_[2] = h3;
      crystal_ = Crystal::CreateHexPyramid(a1, a2, h1, h2, h3);
    }
  } else if (p_obj.size() == 7) {
    int upper_idx1 = p_obj[0].get<int>();
    int upper_idx2 = p_obj[1].get<int>();
    int lower_idx1 = p_obj[2].get<int>();
    int lower_idx2 = p_obj[3].get<int>();
    auto h1 = p_obj[4].get<float>();
    auto h2 = p_obj[5].get<float>();
    auto h3 = p_obj[6].get<float>();
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


void CrystalContext::ParseHexPyramidStackHalf(const nlohmann::json& obj) {
  auto p_obj = obj.at("parameter");
  if (!p_obj.is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal parameter must be array!", obj);
  }
  if (p_obj.size() == 7) {
    int upper_idx1 = p_obj[0].get<int>();
    int upper_idx2 = p_obj[1].get<int>();
    int lower_idx1 = p_obj[2].get<int>();
    int lower_idx2 = p_obj[3].get<int>();
    auto h1 = p_obj[4].get<float>();
    auto h2 = p_obj[5].get<float>();
    auto h3 = p_obj[6].get<float>();
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


void CrystalContext::ParseCubicPyramid(const nlohmann::json& obj) {
  auto p_obj = obj.at("parameter");
  if (!p_obj.is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal parameter must be array!", obj);
  }
  if (p_obj.size() == 2) {
    auto h1 = p_obj[0].get<float>();
    auto h2 = p_obj[1].get<float>();
    h_param_[0] = h1;
    h_param_[1] = h2;
    crystal_ = Crystal::CreateCubicPyramid(h1, h2);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseIrregularHexPrism(const nlohmann::json& obj) {
  auto p_obj = obj.at("parameter");
  if (!p_obj.is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal parameter must be array!", obj);
  }
  if (p_obj.size() == 7) {
    auto d1 = p_obj[0].get<float>();
    auto d2 = p_obj[1].get<float>();
    auto d3 = p_obj[2].get<float>();
    auto d4 = p_obj[3].get<float>();
    auto d5 = p_obj[4].get<float>();
    auto d6 = p_obj[5].get<float>();
    auto h = p_obj[6].get<float>();

    float dist[6] = { d1, d2, d3, d4, d5, d6 };

    d_param_[0] = d1;
    d_param_[1] = d2;
    d_param_[2] = d3;
    d_param_[3] = d4;
    d_param_[4] = d5;
    d_param_[5] = d6;
    h_param_[0] = h;
    crystal_ = Crystal::CreateIrregularHexPrism(dist, h);
  } else {
    throw std::invalid_argument("<crystal.parameter> number doesn't match!");
  }
}


void CrystalContext::ParseIrregularHexPyramid(const nlohmann::json& obj) {
  auto p_obj = obj.at("parameter");
  if (!p_obj.is_array()) {
    throw nlohmann::detail::other_error::create(-1, "crystal parameter must be array!", obj);
  }
  if (p_obj.size() == 13) {
    auto d1 = p_obj[0].get<float>();
    auto d2 = p_obj[1].get<float>();
    auto d3 = p_obj[2].get<float>();
    auto d4 = p_obj[3].get<float>();
    auto d5 = p_obj[4].get<float>();
    auto d6 = p_obj[5].get<float>();
    int i1 = p_obj[6].get<int>();
    int i2 = p_obj[7].get<int>();
    int i3 = p_obj[8].get<int>();
    int i4 = p_obj[9].get<int>();
    auto h1 = p_obj[10].get<float>();
    auto h2 = p_obj[11].get<float>();
    auto h3 = p_obj[12].get<float>();

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


void CrystalContext::ParseCustomCrystal(const nlohmann::json& obj) {
  file_param_ = obj.at("parameter").get<std::string>();
  std::FILE* file = std::fopen(file_param_.c_str(), "r");
  if (!file) {
    throw std::invalid_argument("<crystal.parameter> cannot open model file!");
  }

  std::vector<Vec3f> vertexes;
  std::vector<TriangleIdx> faces;
  float v_buf[3];
  int f_buf[3];
  int curr_char = 0;
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
  crystal_ = Crystal::CreateCustomCrystal(vertexes, faces);
}


void to_json(nlohmann::json& obj, const AxisDistribution& axis) {
  obj["zenith"] = axis.latitude_dist;
  obj["zenith"]["mean"] = 90.0f - axis.latitude_dist.mean;
  obj["azimuth"] = axis.azimuth_dist;
  obj["roll"] = axis.roll_dist;
}

void from_json(const nlohmann::json& obj, AxisDistribution& axis) {
  obj.at("zenith").get_to(axis.latitude_dist);
  axis.latitude_dist.mean = 90.0f - axis.latitude_dist.mean;

  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "azimuth", axis.azimuth_dist)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "roll", axis.roll_dist)
}


CrystalContext::CrystalContext() : id_(kInvalidId), idx_param_{}, a_param_{}, h_param_{}, d_param_{} {}


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


int CrystalContext::RandomSampleFace(const float* ray_dir, float* prob_buf) const {
  int total_faces = crystal_->TotalFaces();
  const auto* face_norm = crystal_->GetFaceNorm();
  const auto* face_area = crystal_->GetFaceArea();

  std::unique_ptr<float[]> face_prob_buf;
  if (!prob_buf) {
    face_prob_buf.reset(new float[total_faces]);
    prob_buf = face_prob_buf.get();
  }

  float sum = 0;
  for (int k = 0; k < total_faces; k++) {
    prob_buf[k] = 0;
    if (!std::isnan(face_norm[k * 3 + 0]) && face_area[k] > 0) {
      prob_buf[k] = std::max(-Dot3(face_norm + k * 3, ray_dir) * face_area[k], 0.0f);
      sum += prob_buf[k];
    }
  }
  for (int k = 0; k < total_faces; k++) {
    prob_buf[k] /= sum;
  }

  return static_cast<ShortIdType>(RandomSampler::SampleInt(prob_buf, total_faces));
}


void CrystalContext::SaveHexPrismParam(nlohmann::json& obj) const {
  obj["type"] = "HexPrism";
  obj["parameter"] = h_param_[0];
}


void CrystalContext::SaveHexPyramidH3Param(nlohmann::json& obj) const {
  obj["type"] = "HexPyramid";
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveHexPyramidI2H3Param(nlohmann::json& obj) const {
  obj["type"] = "HexPyramid";
  obj["parameter"].emplace_back(idx_param_[0]);
  obj["parameter"].emplace_back(idx_param_[1]);
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveHexPyramidI4H3Param(nlohmann::json& obj) const {
  obj["type"] = "HexPyramid";
  obj["parameter"].emplace_back(idx_param_[0]);
  obj["parameter"].emplace_back(idx_param_[1]);
  obj["parameter"].emplace_back(idx_param_[2]);
  obj["parameter"].emplace_back(idx_param_[3]);
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveHexPyramidA2H3Param(nlohmann::json& obj) const {
  obj["type"] = "HexPyramid";
  obj["parameter"].emplace_back(a_param_[0]);
  obj["parameter"].emplace_back(a_param_[1]);
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveHexPyramidStackHalfParam(nlohmann::json& obj) const {
  obj["type"] = "HexPyramidStackHalf";
  obj["parameter"].emplace_back(idx_param_[0]);
  obj["parameter"].emplace_back(idx_param_[1]);
  obj["parameter"].emplace_back(idx_param_[2]);
  obj["parameter"].emplace_back(idx_param_[3]);
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveIrregularHexPrismParam(nlohmann::json& obj) const {
  obj["type"] = "IrregularHexPrism";
  obj["parameter"].emplace_back(d_param_[0]);
  obj["parameter"].emplace_back(d_param_[1]);
  obj["parameter"].emplace_back(d_param_[2]);
  obj["parameter"].emplace_back(d_param_[3]);
  obj["parameter"].emplace_back(d_param_[4]);
  obj["parameter"].emplace_back(d_param_[5]);
  obj["parameter"].emplace_back(h_param_[0]);
}


void CrystalContext::SaveIrregularHexPyramidParam(nlohmann::json& obj) const {
  obj["type"] = "IrregularHexPyramid";
  obj["parameter"].emplace_back(d_param_[0]);
  obj["parameter"].emplace_back(d_param_[1]);
  obj["parameter"].emplace_back(d_param_[2]);
  obj["parameter"].emplace_back(d_param_[3]);
  obj["parameter"].emplace_back(d_param_[4]);
  obj["parameter"].emplace_back(d_param_[5]);
  obj["parameter"].emplace_back(idx_param_[0]);
  obj["parameter"].emplace_back(idx_param_[1]);
  obj["parameter"].emplace_back(idx_param_[2]);
  obj["parameter"].emplace_back(idx_param_[3]);
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
  obj["parameter"].emplace_back(h_param_[2]);
}


void CrystalContext::SaveCubicPyramidParam(nlohmann::json& obj) const {
  obj["type"] = "CubicPyramid";
  obj["parameter"].emplace_back(h_param_[0]);
  obj["parameter"].emplace_back(h_param_[1]);
}


void CrystalContext::SaveCustomCrystalParam(nlohmann::json& obj) const {
  obj["type"] = "Custom";
  obj["parameter"] = file_param_;
}


void to_json(nlohmann::json& obj, const CrystalContext& ctx) {
  obj["id"] = ctx.id_;

  using ParamSaver = std::function<void(const CrystalContext*, nlohmann::json&)>;
  std::map<CrystalType, ParamSaver> param_saver{
    { CrystalType::kPrism, &CrystalContext::SaveHexPrismParam },
    { CrystalType::kPyramid_H3, &CrystalContext::SaveHexPyramidH3Param },
    { CrystalType::kPyramid_I2H3, &CrystalContext::SaveHexPyramidI2H3Param },
    { CrystalType::kPyramid_I4H3, &CrystalContext::SaveHexPyramidI4H3Param },
    { CrystalType::kPyramid_A2H3, &CrystalContext::SaveHexPyramidA2H3Param },
    { CrystalType::kPyramidStackHalf, &CrystalContext::SaveHexPyramidStackHalfParam },
    { CrystalType::kIrregularPrism, &CrystalContext::SaveIrregularHexPrismParam },
    { CrystalType::kIrregularPyramid, &CrystalContext::SaveIrregularHexPyramidParam },
    { CrystalType::kCubicPyramid, &CrystalContext::SaveCubicPyramidParam },
    { CrystalType::kCustom, &CrystalContext::SaveCustomCrystalParam },
  };
  param_saver.at(ctx.crystal_->GetType())(&ctx, obj);

  obj["zenith"] = ctx.axis_.latitude_dist;
  obj["zenith"]["mean"] = 90.0f - ctx.axis_.latitude_dist.mean;
  obj["azimuth"] = ctx.axis_.azimuth_dist;
  obj["roll"] = ctx.axis_.roll_dist;
}


void from_json(const nlohmann::json& obj, CrystalContext& ctx) {
  obj.at("id").get_to(ctx.id_);

  using CrystalParser = std::function<void(CrystalContext*, const nlohmann::json&)>;
  std::map<std::string, CrystalParser> crystal_parsers{
    { "HexPrism", &CrystalContext::ParseHexPrism },
    { "HexPyramid", &CrystalContext::ParseHexPyramid },
    { "HexPyramidStackHalf", &CrystalContext::ParseHexPyramidStackHalf },
    { "CubicPyramid", &CrystalContext::ParseCubicPyramid },
    { "IrregularHexPrism", &CrystalContext::ParseIrregularHexPrism },
    { "IrregularHexPyramid", &CrystalContext::ParseIrregularHexPyramid },
    { "Custom", &CrystalContext::ParseCustomCrystal },
  };

  auto type = obj.at("type").get<std::string>();
  crystal_parsers.at(type)(&ctx, obj);

  obj.at("zenith").get_to(ctx.axis_.latitude_dist);
  ctx.axis_.latitude_dist.mean = 90.0f - ctx.axis_.latitude_dist.mean;

  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "azimuth", ctx.axis_.azimuth_dist)
  JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, "roll", ctx.axis_.roll_dist)
}


void CrystalContext::PrintCrystal() const {
  const auto* g = GetCrystal();
  LOG_VERBOSE("-- ID: %d --", GetId());
  for (const auto& v : g->GetVertexes()) {
    LOG_VERBOSE("v %+.4f %+.4f %+.4f", v.x(), v.y(), v.z());
  }
  for (const auto& f : g->GetMergedFaces()) {
    std::stringstream ss;
    ss << "f";
    for (auto id : f.second.idx()) {
      ss << " " << id;
    }
    LOG_VERBOSE(ss.str());
  }
}

}  // namespace icehalo
