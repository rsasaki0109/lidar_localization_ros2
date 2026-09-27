# Experimental callback delivery trace

Historical diagnostic documentation: all fixed-dataset traces were removed in
`2d9d7bc`. Reception/lifecycle fixes were integrated after final-candidate tests;
see [the integration record](../../docs/go2_twist_reception.md).

Bounded diagnostic logs for Box source stamps 1787732247–1787732254.
No decision, QoS, guard, prediction, or history policy changes.

- TWIST_RECEIVE: source stamp (seconds), callback entry, state-lock acquired,
  history insertion complete (the last three are steady-clock nanoseconds).
- SCAN_RECEIVE: source stamp, callback entry, state-lock acquired, snapshot
  selection complete, selected twist source stamp (or -1).

For a missing offline endpoint, compare its callback entry and insertion against
scan admission. Entry before admission but lock acquisition after admission establishes
an overlapping lock-acquisition interval (including possible thread scheduling
delay), not time spent exclusively blocked on the mutex. Entry after admission
does not distinguish middleware
arrival from executor scheduling. No native DDS receive timestamp is recorded.
Twist callbacks are mutually exclusive within their dedicated group, so waiting
on a lock can also delay the entry of subsequent messages.

Logging happens after captured events and can perturb later callbacks. Compare
with the parent candidate; never claim zero overhead or production readiness.
Fixed dataset timestamps and these logs must not be promoted to main.

## Independent reception candidate

`TWIST_BUFFER` replaces `TWIST_RECEIVE`: the middle timestamp now measures
acquisition of the short history mutex, not localization state. Direct prediction
callbacks never acquire localization state. EKF/GTSAM callbacks retain the old
state lock before history insertion/backend updates; configuration captures that
route at subscription creation. No new parameter or queue/QoS change.

The only lock order is localization state (optional) then history. No callback
holds history while acquiring localization state. Shutdown sets its atomic flag
before taking state and history locks; insertion rechecks the flag under history
lock, so clearing cannot be followed by a late insert during shutdown. Scan
snapshots remain copies under the state lock and survive later history changes.
The lifecycle reconfiguration/old callback boundary needs integration coverage
before adoption, as do both enabled pose backends.

`SCAN_BUFFER`: source stamp, callback entry, localization-state lock acquired,
history lock acquired, selection complete (monotonic nanoseconds), selected
twist source stamp. Selection completion is captured while holding history lock
to keep its interval comparable with `TWIST_BUFFER` insertion intervals.

## Subscription lifetime guard

Cleanup increments `twist_subscription_generation_` while clearing history under
its mutex. Each subscription captures its generation. Insert checks both shutdown
and generation under the same mutex, including when an old saved callback runs
after configure makes shutdown false again. Optional backend calls return before
prediction if the generation is obsolete. No new runtime parameter.

`test_twist_subscription_lifecycle` invokes actual subscription closures and
actual configure/cleanup transitions for direct, EKF, and GTSAM routes. It checks
progress under a held localization-state lock, guarded backend updates, obsolete
callbacks after cleanup/reconfigure, and fresh subscription acceptance. It is a
deterministic callback/lifecycle integration test, not a DDS scheduling test.
