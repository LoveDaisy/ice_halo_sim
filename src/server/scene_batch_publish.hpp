#ifndef LUMICE_SERVER_SCENE_BATCH_PUBLISH_HPP
#define LUMICE_SERVER_SCENE_BATCH_PUBLISH_HPP

#include <atomic>
#include <cstddef>
#include <utility>

namespace lumice {

// Credits `sim_scene_cnt` for the SimData a batch will yield, THEN publishes the batch
// to `queue` — never the other way round. This is GenerateScene's only production call
// site (server.cpp); factored out so the order is a named, single-owner invariant
// instead of two adjacent statements a future edit can swap without noticing.
//
// ConsumeData does NOT gate consumption on this counter (see its generation-mismatch
// check) — a batch is consumed once its generation matches, regardless of what
// sim_scene_cnt reads. This counter exists solely so GetStatus /
// PublishDrainedEpochIfSettled (both gated on sim_scene_cnt <= 0) can tell when an epoch
// has fully drained. Crediting AFTER the publish would let a consumer that dequeues and
// finishes processing a batch before the producer resumes observe the pre-credit
// (too-low) count and declare the epoch drained one batch too early — a stale signal
// that self-corrects the moment that batch is actually accounted for, not a data-loss
// bug on its own. Crediting first removes that window entirely: the count already
// reflects an in-flight batch by the time any consumer could observe it as "empty".
//
// This same misordering, on an earlier form of this code where ConsumeData still gated
// consumption on sim_scene_cnt directly, caused outright silent data loss under a stall
// injected between the two statements: at 0.2-1ms it dropped whole dispatch grains
// (matches two independently observed CI shortfalls at grain 128: 19616 and 19872 rays
// against an expected total of 20000); crediting first closes that window on its own,
// independent of whether ConsumeData still gates on this counter. See
// test/unit-correctness/server/test_scene_batch_publish_order.cpp for the deterministic
// (non-timing-dependent) regression test pinning this order.
template <class QueueT, class BatchT>
void AccountThenPublishBatch(std::atomic_int& sim_scene_cnt, int credit, QueueT& queue, BatchT&& batch) {
  sim_scene_cnt += credit;
  queue.Emplace(std::forward<BatchT>(batch));
}

// The inverse of AccountThenPublishBatch, and the single owner of that inverse: drops
// every batch still waiting in `queue` and gives `sim_scene_cnt` back the credit those
// batches were charged. Returns the total ray count of the dropped batches — their budget
// was already spent against the producer's ray ledger, so a finite ray_num run would trace
// that many rays fewer unless the caller hands it back (see GenerateScene's committed_num).
// `out_batch_count`, when non-null, receives how many batches were dropped.
//
// Drain FIRST, discount SECOND — the mirror image of AccountThenPublishBatch's order, for
// the mirror-image reason. Discounting first would leave the counter reading lower than the
// work still dequeuable from the queue, which is exactly the window that lets a consumer
// call the epoch drained while batches remain; this order errs the other way, briefly
// over-reporting in-flight work, which costs nothing but a late "drained" verdict.
//
// The caller must know that every batch it is discarding was charged the same credit — true
// for GenerateScene, whose scene snapshot (and hence per-batch SimData count) is fixed for
// the whole invocation, and whose queue cannot hold a batch from any other invocation
// (Stop() swap-clears the queue, and CommitConfig restarts through Stop/Start).
template <class QueueT>
size_t DiscardQueuedBatchesThenRefund(std::atomic_int& sim_scene_cnt, int credit_per_batch, QueueT& queue,
                                      size_t* out_batch_count = nullptr) {
  auto dropped = queue.DrainAll();
  size_t ray_num = 0;
  for (const auto& batch : dropped) {
    ray_num += batch.ray_num_;
  }
  sim_scene_cnt -= static_cast<int>(dropped.size()) * credit_per_batch;
  if (out_batch_count) {
    *out_batch_count = dropped.size();
  }
  return ray_num;
}

}  // namespace lumice

#endif  // LUMICE_SERVER_SCENE_BATCH_PUBLISH_HPP
