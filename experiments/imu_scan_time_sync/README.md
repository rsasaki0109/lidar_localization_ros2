# IMU / scan time synchronization experiment

Date: 2026-07-14

This experiment tested whether the low continuous-time deskew application rate
on Koide `outdoor_hard_01a` was caused by IMU samples arriving while NDT was
registering an older cloud. The controlled window was bag offset 85 s for 27 s.

The candidate buffered transformed IMU samples and integrated only samples at
or before the cloud timestamp. The diagnostic integration horizon was also
measured at the cloud timestamp instead of at the newest received IMU sample.

The candidate failed. After recovering across one 0.65 s IMU gap, only 1 of 26
diagnostic scans applied deskew. The first inaccurate IMU prediction led to
registration rejection, after which the accepted-pose integration horizon
correctly exceeded 1 s. Trajectory coverage fell to 6.3% and translation RMSE
rose to 6.599 m. The implementation was therefore reverted.

The same run also confirmed that increasing application rate alone is unsafe:
rotation-only IMU pose-history deskew applied to 94.3% of scans but increased
translation RMSE from 0.141 m to 1.886 m. Offline gyro/reference alignment over
1,200 scans preferred the documented `header=start` convention (0.0581 rad/s
fitted residual versus 0.0794 rad/s for `header=end`), so changing the cloud
timestamp convention is not justified.

Conclusion: keep continuous-time deskew default-off. The next IMU candidate
must estimate or observe velocity at initialization and pass an open-loop seed
accuracy gate before it is allowed to affect NDT or deskew.

See [results.json](results.json) for the measured values.
