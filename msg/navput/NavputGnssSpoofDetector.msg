# GNSS spoof-detector state and its internal GNSS Kalman filter snapshot.
# Published for live Zenoh/Debin diagnostics only.

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint64 timestamp_sample # GNSS-KF measurement timestamp

uint8 STATE_NO_ORIGIN = 0
uint8 STATE_TRUSTED = 1
uint8 STATE_UNTRUSTED = 2
uint8 state

float32 imu_velocity_suspicion
float32 gnss_velocity_consistency_suspicion
float32 aux_position_suspicion
bool needs_aux_recovery

bool gnss_kf_valid
float64 ref_lat
float64 ref_lon
float32[3] position_ned
float32[3] velocity_ned
float32[3] position_variance
float32[3] velocity_variance

# TOPICS navput_gnss_spoof_detector
