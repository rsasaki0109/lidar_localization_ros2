# Independent rejection accounting

Candidate based on main cddae53, without the rejected freshness policy. When prediction is disabled, a rejected measurement still increments consecutive_rejected_updates while the stored pose and its timestamp stay unchanged. Acceptance resets the count.

Observed held-pose failure logs contained twelve consecutive rejections reporting zero. The isolated before/candidate fixture under /media/sasaki/aiueo2/jeplo_data/experiments/go2_rejection_accounting fails on the old implementation and passes the candidate. The updated existing policy test passes with Release assertions enabled.

Not promoted: replay validation pending. The serial Release overlay build passed, all 65 CTests passed, and all 232 Python tests passed (logs `/tmp/jeplo_rejection_accounting_{build,ctest,pytest}.log`). This changes count-driven guard release/recovery in no-prediction and odom-only modes. It does not solve dropout drift alone, and release after30 rejects can still admit an incorrect match. Existing twist/previous-delta paths are unchanged.

Count-consumer audit: measurement_gate_policy uses the streak for post-reject score tightening, odom recovery correction limits, seed correction guard release, consistency recovery, and rejected-seed reuse. alignment_retry_policy uses it to enable retry from the last accepted pose. recovery_supervisor uses it in reinitialization scoring and recovery status/action classification. Thus this is a behavior change when no motion predictor is available, not merely a diagnostic correction. Existing mode-independent guard unit tests pass; replay must establish the combined behavior.
