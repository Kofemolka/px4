# AUX

## Requirements for first lock:

1. Fresh readings in the ranging_beacon
2. Something in the navput_local_position
3. Select projection center:
    1. local_pos.xy_valid && local_pos.xy_global
    2. _has_last_solution
    3. at least one beacon
4. 3 distinct beacons with <1s readings
5. Solution HDOP ≤ 5.0
6. Unambiguous solution
7. Solution is published to aux_global_position
8. param EKF2_AGP0_ID == 111
9. param NPT_FUSE_AGP0 = 1
10. No global_origin_valid or no other aiding source
    1. navput_local_position.ref_lat/lon ≠ NaN
    2. navput_fusion_control: gps_active[0] || ev_active || rngbcn_active
11. Fusion result is published to estimator_aid_src_aux_global_position
12. Aux machine needs navput_status_flags.cs_yaw_align == true

## Who can align Yaw:

- Ekf::resetYawToEKFGSF()
- EV yaw control
- resetYawToGnss()
- Ekf::controlMagFusion()

## Mag Fusion:

1. param EKF2_MAG_TYPE ≠ 5
2. param NPT_FUSE_MAG = 1

---

# GPS

## Why origin sets after driving a car for hundreds of meters, but not on foot:

Both origin-set and yaw-align funnel through the same gate: GNSS-reported speed
accuracy (`sacc`) vs param EKF2_REQ_SACC (default 0.5 m/s). At walking pace
(~1.3 m/s) the receiver's Doppler-derived `sacc` is often noisy/degraded and
stays above 0.5 m/s; at car speed it drops cleanly below it within seconds.

- Ekf::controlGnssPosFusion() (gps_control.cpp:245-247): sets local origin via
  resetHorizontalPositionToGnss() as soon as `gpos_init_conditions_passing`
  (gnss_pos_enabled && _gnss_checks.passed()) is true — deliberately NOT
  gated on tilt_align/yaw_align.
- GnssChecks::runInitialFixChecks() (gnss_checks.cpp) fails while
  `gnss.sacc > EKF2_REQ_SACC`. Must pass continuously for EKF2_REQ_GPS_H
  (default 10 s) — any single bad sample resets the timer to zero.
- Ekf::controlGnssYawEstimator() (gps_control.cpp:384-388): feeds the
  EKF-GSF yaw estimator ONLY when `vel_accuracy < EKF2_REQ_SACC`. If sacc is
  too high, the estimator gets zero updates (not just noisy ones), so
  yaw_align via GPS never happens — blocking full vel/pos fusion start
  (starting_conditions_passing requires tilt_align && yaw_align).
- EKFGSF_yaw::fuseVelocity() also has its own soft minimum-motion guard:
  it only activates once `|velocity| > vel_accuracy`, harder to satisfy at
  walking speed with degraded sacc.

## Param to tweak for faster/easier GPS engagement:

- **EKF2_REQ_SACC** — raise this (e.g. 1.0-2.0 m/s) to admit noisier
  low-speed GPS velocity fixes into both the checks gate and the yaw
  estimator. Primary lever; safer than disabling the check outright since
  it keeps the sanity check, just relaxes the threshold.
- **EKF2_GPS_CHECK** — bitmask, default 2047 (bits 0-10). Clear bit 4
  (kSacc = 1<<4 = 16) → 2031 to drop the sacc check from
  runInitialFixChecks() entirely. Coarser than raising EKF2_REQ_SACC; does
  NOT fix the separate sacc gate in controlGnssYawEstimator()
  (gps_control.cpp:385), which is hardcoded to ekf2_req_sacc regardless of
  this bitmask.
- **EKF2_REQ_GPS_H** — default 10 s continuous pass required before checks
  latch. Lowering shortens the time to first pass once sacc is under
  threshold, but doesn't help if sacc never drops below EKF2_REQ_SACC at
  the given speed.

---

