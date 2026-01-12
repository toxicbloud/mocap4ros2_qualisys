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

The calibration process uses an **NTP-inspired ping-pong approach**:

1. **Send "ping"**: Request a frame from QTM using `GetCurrentFrame()` and record the send time (T1)
2. **Receive "pong"**: Receive the frame packet with QTM's timestamp (T2/T3) and record the receive time (T4)
3. **Calculate RTT**: Round-trip time = T4 - T1
4. **Estimate midpoint**: Assume the packet was created at T1 + (RTT/2), accounting for symmetric network delay
5. **Calculate offset**: PC_time_at_midpoint - QTM_timestamp
6. **Average samples**: Collect multiple measurements and use the median for robustness

This method is more reliable than event-based approaches because:
- QTM doesn't reliably send event notifications back through the protocol
- It uses the actual data stream that's continuously available
- The NTP-style calculation accounts for network latency
- Multiple samples with median filtering removes outliers from network jitter

**Formula**: `Offset = PC_time_estimated_at_packet_creation - QTM_timestamp`

Where `PC_time_estimated_at_packet_creation = T1_send + (RTT / 2)`

This ensures that published ROS messages have accurate timestamps that reflect the true capture time while maintaining synchronization across the system.
