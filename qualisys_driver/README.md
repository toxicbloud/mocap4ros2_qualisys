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
* `publish_pose_with_covariance` (false): Enable publishing of PoseWithCovarianceStamped for robot_localization.
* `pose_covariance_diagonal` ([0.001, 0.001, 0.001, 0.001, 0.001, 0.001]): Diagonal covariance values for pose [x, y, z, rot_x, rot_y, rot_z].

## Using with robot_localization

To use the Qualisys system as ground truth with `robot_localization` EKF for map-to-odom transform:

1. Enable PoseWithCovarianceStamped publishing:
```yaml
publish_pose_with_covariance: true
```

2. Adjust the covariance values based on your system's accuracy. The values represent variance (meters² for position, radians² for orientation):
```yaml
pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
```

3. Configure your robot_localization node to subscribe to the `/pose_with_covariance` topic.

**Note**: The Qualisys SDK does not provide residual or covariance information. The covariance matrix uses configured constant diagonal values. Adjust these based on your system calibration and expected accuracy.
