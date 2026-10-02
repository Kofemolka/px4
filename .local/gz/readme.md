# Launch

1. Launch Gazebo World first
```
./gz.bash
```

2. Build and launch PX4 SITL, attaching to the Gazebo
```
./px4_sitl.bash
```

3. Build and launch NPT SITL, attaching to the same model in Gazebo
```
./npt.bash
```

# Ports

When running PX4 and Navput, they expose following UDP ports:


| Link         | Instance 0  listens | Instance 1 listens | Sends to (0 / 1) |
| ------------ | ------------------- | ------------------ | ---------------- |
| GCS          | 18570               | 18571              | 14550 / 14550    |
| Offboard/API | 14580               | 14581              | 14540 / 14541    |
| Payload      | 14280               | 14281              | 14030 / 14031    |
| Gimbal       | 13030               | 13031              | 13280 / 13281    |

# TODO
* [ ] Gazebo has no ranging beacon simulation (SIH only)
* [ ] Get rid of CONFIG_MAVLINK_SOURCE_NAVPUTER
* [ ] Get rid of Navput* uORB topics
