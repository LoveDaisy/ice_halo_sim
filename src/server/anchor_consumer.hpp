#ifndef SERVER_ANCHOR_CONSUMER_H_
#define SERVER_ANCHOR_CONSUMER_H_

#include <cstddef>
#include <memory>

#include "core/shared/projection_shared.h"
#include "server/consumer.hpp"

namespace lumice {

/**
 * @brief The session's single exposure anchor: one full-sky plane, one scalar.
 * @details The THIRD kind of IConsume, alongside RenderConsumer and StatsConsumer, and it
 *          is the third kind for the same reason StatsConsumer is the second: it is a
 *          property of the SESSION, not of any renderer. Exactly one instance lives in
 *          `consumers_` no matter how many renderers the config declares.
 *
 *          That is not an economy, it is the contract. Every element of `consumers_` gets
 *          its own Consume() call on the same batch, so an anchor folded into
 *          RenderConsumer would accumulate the same physical rays once per renderer, and
 *          the published scalar would become a function of how many renderers happen to be
 *          configured. It also cannot be a second entry in `renders_`: CanUseBackend()
 *          rejects any config with more than one renderer and drops the whole session back
 *          to the legacy CPU path (measured at 35x slower on Metal), so the anchor has to
 *          be a second accumulation TARGET inside the one dispatch, never a second view.
 *
 *          Geometry, resolution and the P99 -> L99_sky conversion all live in
 *          core/anchor_buffer.hpp; this class owns only the accumulation and the snapshot
 *          discipline.
 *
 *          THREAD MODEL. Consume() and PrepareSnapshot() are both called under the server's
 *          consumer_mutex_ (server.cpp holds it across every `for (auto& c : consumers_)`
 *          loop and across the snapshot's Phase 1), so the accumulation needs no atomics of
 *          its own — the same ground StatsConsumer stands on.
 */
class AnchorConsumer : public IConsume {
 public:
  AnchorConsumer();

  void Consume(const SimData& data) override;
  /**
   * @brief Freeze the scalar for this snapshot pass.
   * @details Unlike RenderConsumer this does NOT copy the plane into a second buffer.
   *          The two-phase snapshot exists so that a reader borrowing a PIXEL VIEW cannot
   *          have it rewritten underneath; what this consumer publishes is a single float,
   *          computed here under consumer_mutex_ and then never touched until the next
   *          pass. A snapshot copy of an 8 MB plane would buy nothing a frozen float does
   *          not already give, so the P99 is taken directly off the live accumulator while
   *          the lock that stops Consume() is still held.
   */
  void PrepareSnapshot() override;
  void Reset() override;

  /// The frozen L99_sky (P99 radiance per steradian) for the last snapshot pass, 0 before
  /// the first one and whenever the sky carried no positive Y.
  float SnapshotL99Sky() const { return snapshot_l99_sky_; }

  /// White-box handle on the live accumulator, for the tests that pin WHERE energy lands
  /// rather than only what statistic comes out of it. Same rationale as
  /// RenderConsumer::VisibleMaskForTest.
  const float* AnchorPlaneForTest() const { return anchor_y_.get(); }

 private:
  // Host path: project this batch's outgoing rays into the anchor plane. One
  // ProjectExitToPixel call per ray against the fixed anchor params — the same single
  // arithmetic source RenderConsumer and both GPU kernels use.
  void AccumulateOutgoing(const SimData& data);
  // Device path: fold a plane the backend already accumulated.
  void AccumulateDevicePlane(const SimData& data);

  // kAnchorWidth * kAnchorHeight floats of Y. Allocated once in the constructor: its size
  // is a compile-time constant, so unlike a renderer's buffer it can never need regrowing.
  std::unique_ptr<float[]> anchor_y_;
  lm_proj::ProjParams proj_params_;
  float snapshot_l99_sky_ = 0.0f;
};

}  // namespace lumice

#endif  // SERVER_ANCHOR_CONSUMER_H_
