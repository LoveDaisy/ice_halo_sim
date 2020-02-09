#include "core/optics.h"
#include "gtest/gtest.h"
#include "io/file.h"
#include "util/obj_pool.h"

extern std::string working_dir;

namespace {

class RaySegmentSerializationTest : public ::testing::Test {
 protected:
  void CheckEqualFloat3(const float* a, const float* b) {
    EXPECT_EQ(a[0], b[0]);
    EXPECT_EQ(a[1], b[1]);
    EXPECT_EQ(a[2], b[2]);
  }
};

TEST_F(RaySegmentSerializationTest, SingleRaySegment) {
  auto ray_seg_pool = icehalo::RaySegmentPool::GetInstance();
  ray_seg_pool->Clear();

  float pt[] = { -1.0f, 0.3f,  0.5f,     // For r0
                 0.2f,  -0.8f, 0.1f,     // For r1
                 0.7f,  0.7f,  1.0f };   // For r2
  float dir[] = { -0.8f, 0.9f,  0.1f,    // For r0
                  -0.9f, 0.9f,  1.0f,    // For r1
                  1.0f,  -1.0f, 0.2f };  // For r2
  float w[] = { 1.0f, 0.9f, 0.1f };
  int face_id[] = { 23, 5, 37 };
  auto r0 = ray_seg_pool->GetObject(pt + 0, dir + 0, w[0], face_id[0]);
  auto r1 = ray_seg_pool->GetObject(pt + 3, dir + 3, w[1], face_id[1]);
  auto r2 = ray_seg_pool->GetObject(pt + 6, dir + 6, w[2], face_id[2]);

  r0->next_reflect = r1;
  r0->next_refract = r2;
  r1->prev = r0;
  r2->prev = r0;

  icehalo::File file(working_dir.c_str(), "tmp.bin");
  file.Open(icehalo::FileOpenMode::kWrite);
  r0->Serialize(file, true);
  file.Close();

  ray_seg_pool->Clear();

  file.Open(icehalo::FileOpenMode::kRead);
  auto r_test = ray_seg_pool->GetObject();
  r_test->Deserialize(file, icehalo::endian::kUnknownEndian);

  EXPECT_EQ(r_test->next_reflect, reinterpret_cast<icehalo::RaySegment*>(1));
  EXPECT_EQ(r_test->next_refract, reinterpret_cast<icehalo::RaySegment*>(2));
  EXPECT_EQ(r_test->prev, reinterpret_cast<icehalo::RaySegment*>(0xffffffffffffffff));
  EXPECT_EQ(r_test->root_ctx, reinterpret_cast<icehalo::RayInfo*>(0xffffffffffffffff));
  EXPECT_EQ(r_test->w, w[0]);
  CheckEqualFloat3(r_test->pt.val(), pt + 0);
  CheckEqualFloat3(r_test->dir.val(), dir + 0);
  EXPECT_EQ(r_test->face_id, face_id[0]);
  EXPECT_EQ(r_test->state, icehalo::RaySegmentState::kOnGoing);
}

TEST_F(RaySegmentSerializationTest, RaySegPool) {
  auto ray_seg_pool = icehalo::RaySegmentPool::GetInstance();
  ray_seg_pool->Clear();

  float pt[] = { -1.0f, 0.3f,  0.5f,     // For r0
                 0.2f,  -0.8f, 0.1f,     // For r1
                 0.7f,  0.7f,  1.0f };   // For r2
  float dir[] = { -0.8f, 0.9f,  0.1f,    // For r0
                  -0.9f, 0.9f,  1.0f,    // For r1
                  1.0f,  -1.0f, 0.2f };  // For r2
  float w[] = { 1.0f, 0.9f, 0.1f };
  int face_id[] = { 23, 5, 37 };
  auto r0 = ray_seg_pool->GetObject(pt + 0, dir + 0, w[0], face_id[0]);
  auto r1 = ray_seg_pool->GetObject(pt + 3, dir + 3, w[1], face_id[1]);
  auto r2 = ray_seg_pool->GetObject(pt + 6, dir + 6, w[2], face_id[2]);

  r0->next_reflect = r1;
  r0->next_refract = r2;
  r1->prev = r0;
  r2->prev = r0;

  icehalo::File file(working_dir.c_str(), "tmp.bin");
  file.Open(icehalo::FileOpenMode::kWrite);
  ray_seg_pool->Serialize(file, true);
  file.Close();

  file.Open(icehalo::FileOpenMode::kRead);
  ray_seg_pool->Deserialize(file, icehalo::endian::kUnknownEndian);

  using icehalo::RaySegment;
  ray_seg_pool->Map([=](RaySegment& r) {
    r.next_reflect = ray_seg_pool->GetPointerFromSerializeData(r.next_reflect);
    r.next_refract = ray_seg_pool->GetPointerFromSerializeData(r.next_refract);
    r.prev = ray_seg_pool->GetPointerFromSerializeData(r.prev);
  });

  using icehalo::CombineU32AsPointer;
  r0 = ray_seg_pool->GetPointerFromSerializeData(reinterpret_cast<RaySegment*>(CombineU32AsPointer(0, 0)));
  r1 = ray_seg_pool->GetPointerFromSerializeData(reinterpret_cast<RaySegment*>(CombineU32AsPointer(0, 1)));
  r2 = ray_seg_pool->GetPointerFromSerializeData(reinterpret_cast<RaySegment*>(CombineU32AsPointer(0, 2)));

  EXPECT_EQ(r0->next_reflect, r1);
  EXPECT_EQ(r0->next_refract, r2);
  EXPECT_EQ(r0->prev, nullptr);
  EXPECT_EQ(r0->root_ctx, reinterpret_cast<icehalo::RayInfo*>(0xffffffffffffffff));
  EXPECT_EQ(r0->w, w[0]);
  CheckEqualFloat3(r0->pt.val(), pt + 0);
  CheckEqualFloat3(r0->dir.val(), dir + 0);
  EXPECT_EQ(r0->face_id, face_id[0]);
  EXPECT_EQ(r0->state, icehalo::RaySegmentState::kOnGoing);

  EXPECT_EQ(r1->prev, r0);
  EXPECT_EQ(r1->root_ctx, reinterpret_cast<icehalo::RayInfo*>(0xffffffffffffffff));
  EXPECT_EQ(r1->w, w[1]);
  CheckEqualFloat3(r1->pt.val(), pt + 3);
  CheckEqualFloat3(r1->dir.val(), dir + 3);
  EXPECT_EQ(r1->face_id, face_id[1]);
  EXPECT_EQ(r1->state, icehalo::RaySegmentState::kOnGoing);

  EXPECT_EQ(r2->prev, r0);
  EXPECT_EQ(r2->root_ctx, reinterpret_cast<icehalo::RayInfo*>(0xffffffffffffffff));
  EXPECT_EQ(r2->w, w[2]);
  CheckEqualFloat3(r2->pt.val(), pt + 6);
  CheckEqualFloat3(r2->dir.val(), dir + 6);
  EXPECT_EQ(r2->face_id, face_id[2]);
  EXPECT_EQ(r2->state, icehalo::RaySegmentState::kOnGoing);
}

}  // namespace
