# TF readiness queue experiment

Not connected to ROS and not a measured localization improvement. Base92252eb.
The shared200ms blocking-wait experiment retained external prediction but did
not solve queue1 delivery or fault freshness. This candidate separates pending
TF readiness from scan execution without creating a thread per cloud.

`tf_ready_queue.hpp` is caller-serialized and owns a bounded FIFO of payloads.
Readiness queries do not block. The oldest item dispatches when TF is available
or its wait deadline expires; expiry enables existing prediction fallback.
Items reaching max_age are discarded even if ready. Overflow discards oldest.
Reset releases snapshots and permits a new timestamp epoch. Close is permanent.
Readiness exceptions leave the queued payload intact. Counts expose drops.

The fixture uses capacity4, wait250ms, max_age400ms only as experimental values.
At10Hz and200ms delay it dispatches100/100 with no fallback; a3sec TF hole
produces30 fallback dispatches. This assumes an immediately available worker
and synthetic readiness, not real TF interpolation or NDT execution. Separate
checks cover blocked-worker overflow/staleness, order, duplicates, reset,
payload lifetime, shutdown, and errors. No new production settings are added.

## Required native integration before evaluating efficacy

- Enqueue in a dedicated callback group and serialize queue access. The worker
  belongs to the existing mutually exclusive cloud/map group, so registrations
  cannot overlap when runAlignmentPipelineForScan releases the state lock.
- Do not call admitScanMessage at enqueue: it mutates last_scan_ptr_,
  last_cloud_process_time_, crop guard and prediction state. Admission remains
  at dispatch under the state lock and occurs once per processed scan.
- Capture immutable twist and initial-pose generation at receive time. This is
  an explicit timing change relative to current admission-time capture; it
  needs normal/fault replay validation, not a claim of identical behavior.
  Preserve the chosen snapshot throughout seed and rejected-advance handling.
- Initialpose/reset/map replacement and backward ROS clock jumps must invalidate
  pending entries. Recheck generation under state lock at dispatch and after
  alignment. Queue reset alone cannot invalidate a dispatch already popped.
- Preserve the direct cloud path when external prediction/map->odom anchoring
  is disabled or not established. Do not delay startup waiting for nonexistent TF.
- Query canTransform with zero timeout. After dispatch, odom lookups must also
  be nonblocking; reuse the scoped lookup mechanism only within this path.
  A successful readiness query is not a guarantee that the later lookup succeeds.
- A steady-clock timer must be cancelled and the queue closed at shutdown;
  no raw owner pointer may escape into asynchronous TF callbacks.
- Bound and report overflow, stale, reset drops separately from registration
  rejection. A silently dropped scan cannot be counted as processed input.

Next: implement the guarded native path in this isolated worktree, build and
run all required checks, then frozen savedTF gap/delay plus normal controls.
Only passing that screen permits live/front-end and broader regression work.
Avoid promoting this helper alone: passing queue assertions is not evidence
that the requested localization robustness has improved.
