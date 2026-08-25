# Selected Navputer GPS receiver input.
# Published regardless of whether Navputer fuses GPS.

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint64 timestamp_sample

uint8 gps_instance
uint32 device_id
uint8 fix_type

float64 lat
float64 lon
float32 alt_msl

float32 vel_n
float32 vel_e
float32 vel_d

float32 eph
float32 epv
float32 s_variance

# TOPICS navput_gps_position
