// Bug-resurfacing guard for the result-lifetime defect: a reader held a pointer into
// server-owned result data but no share of its lifetime, so the NEXT snapshot freed
// (composite: std::move over the owning cache) or rewrote in place (mono image / raw XYZ:
// the RenderConsumer's fixed snapshot members) the memory under it. The composite half was
// reproduced deterministically under ASan as a heap-use-after-free, with the free stack
// inside DoSnapshot; the mono/XYZ half tears rather than dangles, which no sanitizer can
// see — so its evidence has to be a content assertion, and it is the one below.
//
// Shape of every case here (deliberately single-threaded): acquire a frame, copy what it
// points at, force ANOTHER snapshot, then read the FIRST frame again and require it
// unchanged. Two threads racing would test the same property with far less power — the
// window is microseconds wide, so a green run would mean almost nothing.
//
// Each case also asserts that the second snapshot really did produce different bytes.
// Without that, "frame1 is unchanged" would also pass on a build where nothing ever
// changes, which is the classic way a lifetime test quietly stops testing anything.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lumice.h"
#include "server/render.hpp"

namespace {

// Small, fast, and colored: the raypath_color class makes ColoredMask() non-zero, which is
// what makes DoSnapshot Phase 2 produce a composite at all. Resolution is deliberately tiny
// — these cases assert on bytes, not on physics.
const char* kColorConfig = R"({
  "crystal": [{
    "id": 1, "type": "prism",
    "shape": {"height": 1.5},
    "axis": {"zenith": {"type": "gauss", "mean": 90.0, "std": 10.0},
             "azimuth": {"type": "uniform", "mean": 0.0, "std": 180.0},
             "roll": {"type": "uniform", "mean": 0.0, "std": 180.0}}
  }],
  "filter": [],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "azimuth": 0.0,
                     "diameter": 0.5, "spectrum": "D65"},
    "ray_num": 20000,
    "max_hits": 8,
    "scattering": [{"prob": 0.0, "entries": [{"crystal": 1, "proportion": 1.0}]}]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "dual_fisheye_equal_area", "fov": 180.0},
    "resolution": [64, 32],
    "view": {"elevation": 0, "azimuth": 0, "roll": 0},
    "visible": "full", "background": [0, 0, 0],
    "intensity_factor": 1.0
  }],
  "raypath_color": {
    "mode": "dominant",
    "classes": [
      {"color": [1.0, 0.0, 0.0], "match": [{"layer": 0, "crystal": 1}]}
    ]
  }
})";

LUMICE_ErrorCode CommitJson(LUMICE_Server* server, const std::string& json) {
  LUMICE_Scene* scene = nullptr;
  if (auto err = LUMICE_SceneFromJson(json.c_str(), &scene); err != LUMICE_OK) {
    return err;
  }
  const auto err = LUMICE_CommitScene(server, scene, /*out_reused=*/nullptr);
  LUMICE_SceneDestroy(scene);
  return err;
}

bool WaitForIdle(LUMICE_Server* server, int timeout_ms) {
  using clock = std::chrono::steady_clock;
  auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (clock::now() < deadline) {
    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

class ResultFrameLifetime : public ::testing::Test {
 protected:
  void SetUp() override {
    LUMICE_ServerConfig cfg{};
    cfg.num_workers = 1;
    server_ = LUMICE_CreateServerEx(&cfg);
    ASSERT_NE(server_, nullptr);
  }

  void TearDown() override {
    if (server_ != nullptr) {
      LUMICE_StopServer(server_);
      LUMICE_DestroyServer(server_);
      server_ = nullptr;
    }
  }

  void RunSim() {
    ASSERT_EQ(CommitJson(server_, kColorConfig), LUMICE_OK);
    ASSERT_TRUE(WaitForIdle(server_, 20000)) << "simulation did not finish in 20s";
  }

  LUMICE_Server* server_ = nullptr;
};

// Composite path — the one whose failure mode was a confirmed heap-use-after-free.
// The second snapshot is forced by a display-time exposure change: it re-bakes the
// composite (new pixel buffer, different bytes) without re-running the simulation, which
// is the shortest deterministic route to "the server published a new composite while a
// reader still holds the old one".
TEST_F(ResultFrameLifetime, CompositePathSurvivesNextSnapshot) {
  RunSim();

  LUMICE_ResultFrame* frame1 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame1), LUMICE_OK);
  ASSERT_NE(frame1, nullptr);

  LUMICE_RenderResult comp1[2]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame1, comp1, 1), LUMICE_OK);
  ASSERT_NE(comp1[0].img_buffer, nullptr) << "no composite produced — the case would be vacuous";
  const size_t comp_bytes = static_cast<size_t>(comp1[0].img_width) * comp1[0].img_height * 3;
  ASSERT_GT(comp_bytes, 0u);
  std::vector<uint8_t> comp_copy(comp1[0].img_buffer, comp1[0].img_buffer + comp_bytes);

  // Second snapshot, frame1 deliberately NOT released.
  ASSERT_EQ(LUMICE_SetCompositeExposure(server_, 2.0f), LUMICE_OK);
  LUMICE_ResultFrame* frame2 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame2), LUMICE_OK);
  LUMICE_RenderResult comp2[2]{};
  ASSERT_EQ(LUMICE_FrameGetComposite(frame2, comp2, 1), LUMICE_OK);
  ASSERT_NE(comp2[0].img_buffer, nullptr);

  // Power check: the new snapshot must actually differ, or "unchanged" proves nothing.
  EXPECT_NE(comp2[0].img_buffer, comp1[0].img_buffer) << "second snapshot reused the first's buffer";
  EXPECT_NE(0, std::memcmp(comp2[0].img_buffer, comp_copy.data(), comp_bytes))
      << "exposure change produced identical pixels — pick a stronger perturbation";

  // The property under test: frame1's pixels are still frame1's pixels. Under the defect
  // this read is a use-after-free (ASan) — the bytes it produces are not even the point.
  EXPECT_EQ(0, std::memcmp(comp1[0].img_buffer, comp_copy.data(), comp_bytes))
      << "the frame held across a snapshot no longer owns its own pixels";

  LUMICE_ReleaseResultFrame(frame1);
  LUMICE_ReleaseResultFrame(frame2);
}

// Mono image + raw XYZ path — same property, different backing (the two pooled
// RenderConsumer buffers). Both are checked in one case because they are borrowed from two
// different pools and a fix that covers one can miss the other. The second snapshot comes
// from re-running the simulation, which is what makes the XYZ accumulator produce different
// numbers; a display-time exposure change would not touch either buffer.
TEST_F(ResultFrameLifetime, MonoXyzPathSurvivesNextSnapshot) {
  RunSim();

  LUMICE_ResultFrame* frame1 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame1), LUMICE_OK);

  LUMICE_RawXyzResult xyz1[2]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame1, xyz1, 1), LUMICE_OK);
  ASSERT_NE(xyz1[0].xyz_buffer, nullptr);
  LUMICE_RenderResult render1[2]{};
  ASSERT_EQ(LUMICE_FrameGetRender(frame1, render1, 1), LUMICE_OK);
  ASSERT_NE(render1[0].img_buffer, nullptr);

  const size_t float_count = static_cast<size_t>(xyz1[0].img_width) * xyz1[0].img_height * 3;
  const size_t render_bytes = static_cast<size_t>(render1[0].img_width) * render1[0].img_height * 3;
  ASSERT_GT(float_count, 0u);
  ASSERT_GT(render_bytes, 0u);
  std::vector<float> xyz_copy(xyz1[0].xyz_buffer, xyz1[0].xyz_buffer + float_count);
  std::vector<uint8_t> render_copy(render1[0].img_buffer, render1[0].img_buffer + render_bytes);

  // Second run, frame1 deliberately NOT released. This also drops and rebuilds the
  // consumer's snapshot state, so it covers "a frame outlives the producer's next reset".
  RunSim();
  LUMICE_ResultFrame* frame2 = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame2), LUMICE_OK);
  LUMICE_RawXyzResult xyz2[2]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame2, xyz2, 1), LUMICE_OK);
  ASSERT_NE(xyz2[0].xyz_buffer, nullptr);
  LUMICE_RenderResult render2[2]{};
  ASSERT_EQ(LUMICE_FrameGetRender(frame2, render2, 1), LUMICE_OK);

  // Power check on both buffers.
  EXPECT_NE(xyz2[0].xyz_buffer, xyz1[0].xyz_buffer) << "second snapshot reused the first's XYZ buffer";
  EXPECT_NE(0, std::memcmp(xyz2[0].xyz_buffer, xyz_copy.data(), float_count * sizeof(float)))
      << "second run produced identical XYZ — the case cannot distinguish anything";

  // The property under test, once per backing.
  EXPECT_EQ(0, std::memcmp(xyz1[0].xyz_buffer, xyz_copy.data(), float_count * sizeof(float)))
      << "raw XYZ was rewritten under a frame that still holds it";
  EXPECT_EQ(0, std::memcmp(render1[0].img_buffer, render_copy.data(), render_bytes))
      << "mono image was rewritten under a frame that still holds it";

  LUMICE_ReleaseResultFrame(frame1);
  LUMICE_ReleaseResultFrame(frame2);
}

// A frame acquired before a server is torn down keeps its data readable afterwards. This is
// the strongest statement of "the reader owns a share": not merely "survives the next
// snapshot" but "survives the producer entirely" — the pool's buffers are kept alive by the
// frame, and the pool itself by the buffers' deleters.
TEST_F(ResultFrameLifetime, FrameOutlivesServer) {
  RunSim();

  LUMICE_ResultFrame* frame = nullptr;
  ASSERT_EQ(LUMICE_AcquireResultFrame(server_, &frame), LUMICE_OK);
  LUMICE_RawXyzResult xyz[2]{};
  ASSERT_EQ(LUMICE_FrameGetRawXyz(frame, xyz, 1), LUMICE_OK);
  ASSERT_NE(xyz[0].xyz_buffer, nullptr);
  const size_t float_count = static_cast<size_t>(xyz[0].img_width) * xyz[0].img_height * 3;
  std::vector<float> xyz_copy(xyz[0].xyz_buffer, xyz[0].xyz_buffer + float_count);

  LUMICE_StopServer(server_);
  LUMICE_DestroyServer(server_);
  server_ = nullptr;  // TearDown must not double-destroy

  EXPECT_EQ(0, std::memcmp(xyz[0].xyz_buffer, xyz_copy.data(), float_count * sizeof(float)));
  LUMICE_ReleaseResultFrame(frame);
}

// White-box on the pool underneath: a buffer still held by somebody must never be handed
// out again, and one that has been dropped must be. The first half is the property the
// frames above depend on; the second is what stops the pool from being a leak.
TEST(FrameBufferPool, RecyclesOnlyReleasedBuffers) {
  auto pool = std::make_shared<lumice::FrameBufferPool<float>>();

  auto a = pool->Acquire(16);
  auto b = pool->Acquire(16);
  EXPECT_NE(a.get(), b.get()) << "handed out a buffer that was still held";
  EXPECT_EQ(pool->FreeCountForTest(), 0u);

  float* a_raw = a.get();
  a.reset();
  EXPECT_EQ(pool->FreeCountForTest(), 1u) << "dropped buffer was not recycled";

  auto c = pool->Acquire(16);
  EXPECT_EQ(c.get(), a_raw) << "recycled buffer was not reused";
  EXPECT_NE(c.get(), b.get());

  // A different element count invalidates the free list rather than handing back a
  // wrong-sized buffer.
  b.reset();
  auto d = pool->Acquire(32);
  EXPECT_EQ(pool->FreeCountForTest(), 0u);
  EXPECT_NE(d.get(), nullptr);
}

// The pool outliving its owner is the exact deleter-lifetime trap this task is about: a
// buffer returned after the pool's owner is gone must not touch freed memory.
TEST(FrameBufferPool, BufferOutlivesPoolOwner) {
  std::shared_ptr<float[]> buf;
  {
    auto pool = std::make_shared<lumice::FrameBufferPool<float>>();
    buf = pool->Acquire(8);
    buf[0] = 42.0f;
  }  // last owning reference dropped here; the buffer's deleter still holds the pool alive
  EXPECT_FLOAT_EQ(buf[0], 42.0f);
  buf.reset();  // recycles into a pool nobody else references — must not crash
}

}  // namespace
