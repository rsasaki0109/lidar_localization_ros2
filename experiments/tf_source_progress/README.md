# Source-progress-aware TF wait experiment

Base: `92252eb8c48c946c09fa9f29692d35982e8f9faa`. Not adopted.

The prediction, map-to-odom publication and auxiliary bridge publication paths
currently each wait up to 100 ms for the same missing odometry. This candidate
always tries an immediate exact lookup. After a bounded wait fails, subsequent
calls skip waiting until the latest available source stamp changes. A successful
lookup, accepted initial pose, map update or lifecycle cleanup resets suppression;
the adapter also resets when the frame pair changes. Existing component state
locking serializes calls. TF listener insertion runs separately.

The candidate uses a 200 ms bound at those three call sites. This is experimental:
prior single-100-ms waiting lost external prediction with a 200 ms delayed source,
while shared-200-ms waiting alone did not solve missing-input problems. The
source-progress policy must be tested against both counterexamples. It does not
queue clouds, return stale transforms, change the latest-transform extrapolation
path or relax registration acceptance guards.

`test_source_wait_policy.cpp` covers policy state independently.
`test_native_buffer.cpp` uses a real `tf2_ros::Buffer`, including delayed insertion,
partial progress during timeout, restored input, historical interpolation,
out-of-order insertion, explicit reset, frame change and cleared/backward data.
It uses system time and insertion threads; it does not validate ROS graph delivery
or simulation-clock pause/rewind. Assertions remain enabled in Release.

Validation artifacts are outside the repository under
`/media/sasaki/aiueo2/jeplo_data/experiments/go2_tf_source_progress`.
Delayed-200-ms and live combined-fault replay candidates use separate artifact
directories with `_delay` and `_live` suffixes. Mainline promotion requires replay
evidence for accuracy, coverage, input loss and recovery, not just fixture success.
