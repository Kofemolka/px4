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
