# Experimental callback delivery trace

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
