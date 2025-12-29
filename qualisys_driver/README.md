# Qualisys driver for ROS 2
This package provides a driver for using the Qualisys motion capture system in ROS 2.

## Installing on Linux
First obtain the Qualisys C++ SDK from [here](https://www.github.com/qualisys/qualisys_cpp_sdk) and place the files in the include/qualisys folder, then follow the building instructions. Afterwards, copy `qualisys_cpp_sdk.a` to `/usr/lib`.

Use the following command to build the workspace, excluding the vicon driver:
`colcon build --packages-skip vicon2_driver`

## Parameters
You can configure the following parameters in `config/qualisys_driver_params.yaml`:
* `host_name` (mocap): The host part of an URI, on which the Qualisys Track Manager is running.
* `port` (22222): The port to which QTM is publishing data.
* `last_frame_number` (0): Frame number from which the system should start keeping track.
* `frame_count` (0): Initial value of the frame counter.
* `dropped_frame_count` (0): Initial value of the dropped frame counter.
* `qos_history_policy` (keep_all): Quality of Service history policy.
* `qos_reliability_policy` (best_effort): Quality of Service reliability policy.
* `qos_depth` (10): Quality of Service depth.
* `use_system_timestamp` (true): When set to `true`, uses ROS system time (`now()`) for message timestamps. When set to `false`, uses the actual timestamp from the Qualisys camera data. Using Qualisys timestamps ensures synchronization with the actual capture time rather than the message publication time.
* `calibrate_timestamp_offset` (false): When set to `true` and `use_system_timestamp` is `false`, the driver will calibrate the offset between camera time and system time on startup. This is useful when the Qualisys cameras use relative timestamps (time since camera startup) rather than absolute UTC time via PTP. The calibration is performed by triggering software events to QTM and measuring the time difference between when the event is sent and when it's received with the camera timestamp.
* `calibration_samples` (10): Number of samples to collect during timestamp offset calibration. More samples provide better accuracy but take longer to calibrate. Each sample takes approximately 300ms.

## Timestamp Calibration

When Qualisys cameras are not synchronized with UTC time via PTP (Precision Time Protocol), they may report timestamps relative to camera startup. In this case, you should:

1. Set `use_system_timestamp: false` to use camera timestamps
2. Set `calibrate_timestamp_offset: true` to enable offset calibration
3. Optionally adjust `calibration_samples` (default: 10) for calibration accuracy

The calibration process:
- Takes control of QTM to enable sending software events
- Sends timestamped software events to QTM at known system times
- Waits for event notifications (separate from streaming data) with camera timestamps
- Calculates the offset between system time (when event was sent) and camera time (when event was recorded)
- Averages multiple samples for better accuracy
- Releases QTM control after calibration

**Note:** Event notifications are sent over the TCP connection separately from streaming data, so the calibration listens for event packets directly rather than using the streaming protocol.

If taking control fails (e.g., another application has control), it falls back to a simpler approach that compares frame reception time with camera timestamps.

This ensures that published ROS messages have accurate timestamps that reflect the true capture time while maintaining synchronization across the system.
