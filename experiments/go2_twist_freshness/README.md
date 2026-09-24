# Go2 twist freshness experiment

This candidate does not modify the registration objective or add parameters.
Twist prediction now checks the timestamp and the velocity components it uses.
The maximum sample age uses the existing `max_twist_prediction_dt`; the allowed
future offset is one `scan_period`, because scans are stamped at acquisition
start. The frame must match `base_frame_id`. An ineligible sample falls back to
the existing seed priority and rejected-scan advance policy. The diagnostic
`registration_seed_source` reports this fallback.

Compare the normal JEPLO preset (1.5 s horizon) and a 0.25 s horizon on normal
and fault-injected bags. The latter bounds stale integration earlier. Keep this
branch experimental until full trajectory and transition tests pass. There is
no covariance admission or slip detector: finite, fresh but biased velocity
still reaches the existing registration/correction guard. EKF and GTSAM twist
callbacks are outside this candidate's scope (both disabled in the Go2 preset).

Baseline, prior disabled with the canonical prediction state: four-second Box
twist dropout produced ATE 0.214487 m, maximum 0.898716 m; a +1 m/s x bias with
unchanged covariance produced ATE approximately 0.085 m but lower availability.
Both baseline cases diagnosed every scan. Inputs/results are recorded under
`experiments/go2_twist_faults` and `loc_cross/go2_prior0_fault_*_20260924_r1`.
