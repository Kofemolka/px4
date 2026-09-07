# Selected Navputer GPS receiver input.
# Published regardless of whether Navputer fuses GPS.

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint64 timestamp_sample

float64 ref_lat
float64 ref_lon

float32[3] position_ned
float32[3] velocity_ned

float32[3] position_variance
float32[3] velocity_variance

# TOPICS navput_spoof_detector_gnss_kf
