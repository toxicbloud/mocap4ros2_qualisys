// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Floris Erich <floris.erich@aist.go.jp>,
//         David Vargas Frutos <david.vargas@urjc.es>
//         José Miguel Guerrero Hernández <josemiguel.guerrero@urjc.es>
//
// Also includes code fragments from Kumar Robotics ROS 1 Qualisys driver

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <utility>
#include <thread>
#include <chrono>
#include "qualisys_driver/qualisys_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;


struct Quaternion {
    float w, x, y, z;
};

Quaternion matrixToQuaternion(float* matrix) {
    Quaternion quaternion;

    float trace = matrix[0] + matrix[4] + matrix[8];
    if (trace > 0) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        quaternion.w = 0.25f / s;
        quaternion.x = (matrix[5] - matrix[7]) * s;
        quaternion.y = (matrix[6] - matrix[2]) * s;
        quaternion.z = (matrix[1] - matrix[3]) * s;
    } else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[0] - matrix[4] - matrix[8]);
            quaternion.w = (matrix[5] - matrix[7]) / s;
            quaternion.x = 0.25f * s;
            quaternion.y = (matrix[3] + matrix[1]) / s;
            quaternion.z = (matrix[6] + matrix[2]) / s;
        } else if (matrix[4] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[4] - matrix[0] - matrix[8]);
            quaternion.w = (matrix[6] - matrix[2]) / s;
            quaternion.x = (matrix[3] + matrix[1]) / s;
            quaternion.y = 0.25f * s;
            quaternion.z = (matrix[7] + matrix[5]) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + matrix[8] - matrix[0] - matrix[4]);
            quaternion.w = (matrix[1] - matrix[3]) / s;
            quaternion.x = (matrix[6] + matrix[2]) / s;
            quaternion.y = (matrix[7] + matrix[5]) / s;
            quaternion.z = 0.25f * s;
        }
    }

    return quaternion;
}

void QualisysDriver::set_settings_qualisys()
{
}

void QualisysDriver::loop()
{
  CRTPacket * prt_packet = port_protocol_.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol_.GetCurrentFrame(CRTProtocol::cComponent3d + CRTProtocol::cComponent6d);
  if (port_protocol_.ReceiveRTPacket(e_type, true)) {
    switch (e_type) {
      case CRTPacket::PacketError:
        {
          std::string s = "Error when streaming frames: ";
          s += port_protocol_.GetRTPacket()->GetErrorString();
          RCLCPP_ERROR(get_logger(), s.c_str());
          break;
        }
      case CRTPacket::PacketNoMoreData:
        RCLCPP_WARN(get_logger(), "No data received");
        break;
      case CRTPacket::PacketData:
        process_packet(prt_packet);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown CRTPacket");
    }
  }
}

void QualisysDriver::process_packet(CRTPacket * const packet)
{
  unsigned int marker_count = packet->Get3DMarkerCount();
  unsigned int rb_count = packet->Get6DOFBodyCount();
  int frame_number = packet->GetFrameNumber();

  int frame_diff = 0;
  if (last_frame_number_ != 0) {
    frame_diff = frame_number - last_frame_number_;
    frame_count_ += frame_diff;

    if (frame_diff > 1) {
      dropped_frame_count_ += frame_diff;
      double dropped_frame_pct = static_cast<double>(dropped_frame_count_ / frame_count_ * 100);

      RCLCPP_DEBUG(
        get_logger(),
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frame_diff, dropped_frame_count_, frame_count_, dropped_frame_pct
      );
    }
  }
  last_frame_number_ = frame_number;

  if (!mocap_markers_pub_->is_activated() && !mocap_rigid_bodies_pub_->is_activated() ) {
    return;
  }

  // Get timestamp - use Qualisys timestamp if use_system_timestamp is false
  rclcpp::Time timestamp;
  if (use_system_timestamp_) {
    timestamp = rclcpp::Clock().now();
  } else {
    // GetTimeStamp() returns timestamp in microseconds
    const uint64_t MICROSECONDS_PER_SECOND = 1000000;
    const uint32_t NANOSECONDS_PER_MICROSECOND = 1000;
    
    uint64_t qualisys_timestamp_us = packet->GetTimeStamp();
    // Convert microseconds to nanoseconds
    int64_t camera_time_ns = static_cast<int64_t>(qualisys_timestamp_us * NANOSECONDS_PER_MICROSECOND);
    
    // Apply calibrated offset if available
    if (calibrate_timestamp_offset_ && timestamp_offset_calibrated_) {
      camera_time_ns += timestamp_offset_ns_;
    }
    
    // Convert to ROS time
    timestamp = rclcpp::Time(camera_time_ns);
  }

  if (mocap_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers markers_msg;
    markers_msg.header.frame_id = frame_id_;
    markers_msg.header.stamp = timestamp;
    markers_msg.frame_number = frame_number;

    for (unsigned int i = 0; i < marker_count; ++i) {
      float x, y, z;
      packet->Get3DMarker((float)i, x, y, z);
      mocap4r2_msgs::msg::Marker this_marker;
      this_marker.marker_index = i;
      this_marker.translation.x = x / 1000;
      this_marker.translation.y = y / 1000;
      this_marker.translation.z = z / 1000;
      if (!std::isnan(this_marker.translation.x) && !std::isnan(this_marker.translation.y) && !std::isnan(this_marker.translation.z)){
        markers_msg.markers.push_back(this_marker);
      }
    }

    mocap_markers_pub_->publish(markers_msg);
  }

  if (mocap_rigid_bodies_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.frame_id = frame_id_;
    msg_rb.header.stamp = timestamp;
    msg_rb.frame_number = frame_number;

    for (unsigned int i = 0; i < rb_count; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      float x, y, z;
      float rot_matrix[9];
      // Get6DOFBody(unsigned int nBodyIndex, float &fX, float &fY, float &fZ, float afRotMatrix[9]);
      packet->Get6DOFBody(i, x, y, z, rot_matrix);
      Quaternion quaternion = matrixToQuaternion(rot_matrix);

      const char* label = port_protocol_.Get6DOFBodyName(i);
  
      rb.rigid_body_name = label;
      rb.pose.position.x = x / 1000;
      rb.pose.position.y = y / 1000;
      rb.pose.position.z = z / 1000;
      rb.pose.orientation.x = quaternion.x;
      rb.pose.orientation.y = quaternion.y;
      rb.pose.orientation.z = quaternion.z;
      rb.pose.orientation.w = quaternion.w;

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap_rigid_bodies_pub_->publish(msg_rb);
  }
}

bool QualisysDriver::stop_qualisys()
{
  RCLCPP_INFO(get_logger(), "Stopping the Qualisys motion capture");
  port_protocol_.StreamFramesStop();
  port_protocol_.Disconnect();

  return true;
}

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions node_options)
: rclcpp_lifecycle::LifecycleNode("qualisys_driver_node", node_options)
{
  initParameters();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT QualisysDriver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
  auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      rmw_qos_history_policy->second,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      qos_depth_
  ));
  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(rmw_qos_reliability_policy->second);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/qualisys_driver/change_state");

  mocap_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "/markers", 100);

  mocap_rigid_bodies_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/qualisys_driver/update_notify", qos);

  set_settings_qualisys();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  mocap_markers_pub_->on_activate();
  mocap_rigid_bodies_pub_->on_activate();
  bool success = connect_qualisys();

  if (success) {
    // Perform timestamp offset calibration if enabled and not using system timestamp
    if (!use_system_timestamp_ && calibrate_timestamp_offset_) {
      calibrate_timestamp_offset();
    }
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / publish_rate_), std::bind(&QualisysDriver::loop, this));
    RCLCPP_INFO(get_logger(), "Activated!\n");

    return CallbackReturnT::SUCCESS;
  } else {
    RCLCPP_INFO(get_logger(), "Unable to activate!\n");

    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT QualisysDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  timer_->reset();
  update_pub_->on_deactivate();
  mocap_markers_pub_->on_deactivate();
  mocap_rigid_bodies_pub_->on_deactivate();
  stop_qualisys();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_.reset();
  mocap_markers_pub_.reset();
  mocap_rigid_bodies_pub_.reset();
  timer_->reset();
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Shut down stuff */
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT QualisysDriver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  return CallbackReturnT::SUCCESS;
}

bool QualisysDriver::connect_qualisys()
{
  RCLCPP_WARN(
    get_logger(),
    "Trying to connect to Qualisys host at %s:%d", host_name_.c_str(), port_);

  if (!port_protocol_.Connect(
      reinterpret_cast<const char *>(host_name_.data()), port_, 0, 1, 7))
  {
    RCLCPP_FATAL(get_logger(), "Connection error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected");

  bool settings_read;
  port_protocol_.Read6DOFSettings(settings_read);

  return settings_read;
}

void QualisysDriver::calibrate_timestamp_offset()
{
  RCLCPP_INFO(get_logger(), "Starting timestamp offset calibration with %d samples...", calibration_samples_);
  
  std::vector<int64_t> offset_samples;
  const uint32_t NANOSECONDS_PER_MICROSECOND = 1000;
  
  // Take control of QTM to enable sending events
  if (!port_protocol_.TakeControl()) {
    RCLCPP_WARN(get_logger(), "Failed to take control of QTM, trying calibration without events");
    
    // Fallback: Collect samples by requesting individual frames
    for (int i = 0; i < calibration_samples_; ++i) {
      if (!port_protocol_.GetCurrentFrame(CRTProtocol::cComponent3d + CRTProtocol::cComponent6d)) {
        RCLCPP_WARN(get_logger(), "Failed to get current frame during calibration sample %d", i);
        continue;
      }
      
      CRTPacket * prt_packet = port_protocol_.GetRTPacket();
      if (prt_packet == nullptr) {
        RCLCPP_WARN(get_logger(), "GetRTPacket returned null during calibration");
        continue;
      }
      
      CRTPacket::EPacketType e_type;
      if (port_protocol_.ReceiveRTPacket(e_type, true)) {
        if (e_type == CRTPacket::PacketData) {
          auto current_ros_time = rclcpp::Clock().now();
          uint64_t camera_timestamp_us = prt_packet->GetTimeStamp();
          int64_t camera_time_ns = static_cast<int64_t>(camera_timestamp_us * NANOSECONDS_PER_MICROSECOND);
          int64_t ros_time_ns = current_ros_time.nanoseconds();
          int64_t offset = ros_time_ns - camera_time_ns;
          offset_samples.push_back(offset);
          RCLCPP_DEBUG(get_logger(), "Calibration sample %d (fallback): offset = %ld ns", i, offset);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } else {
    RCLCPP_INFO(get_logger(), "Successfully took control of QTM for event-based calibration");
    
    // Event-based calibration: send events and wait for event notifications
    // Note: We don't use StreamFrames here because event notifications come through
    // the regular TCP connection, not the streaming channel
    for (int i = 0; i < calibration_samples_; ++i) {
      // Record system time when we send the event
      auto trigger_ros_time = rclcpp::Clock().now();
      
      // Send event to QTM
      std::string event_label = "ROS_CALIB_" + std::to_string(i);
      if (!port_protocol_.SetQTMEvent(event_label.c_str())) {
        RCLCPP_WARN(get_logger(), "Failed to send calibration event %d", i);
        continue;
      }
      
      RCLCPP_DEBUG(get_logger(), "Sent calibration event %d", i);
      
      // Wait for the event notification (not streaming data, just event packet)
      bool event_received = false;
      int attempts = 0;
      const int max_attempts = 30; // Timeout after ~3 seconds
      
      while (!event_received && attempts < max_attempts) {
        CRTPacket * prt_packet = port_protocol_.GetRTPacket();
        if (prt_packet == nullptr) {
          attempts++;
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
        
        CRTPacket::EPacketType e_type;
        // Use Receive directly (not ReceiveRTPacket) to get event packets
        auto response = port_protocol_.Receive(e_type, false, 100); // 100ms timeout, don't skip events
        
        if (response == CNetwork::ResponseType::success) {
          if (e_type == CRTPacket::PacketEvent) {
            CRTPacket::EEvent event;
            if (prt_packet->GetEvent(event)) {
              // Accept any trigger event - we have QTM control so these should be ours
              if (event == CRTPacket::EventTrigger) {
                // Get the camera timestamp when the event was recorded
                uint64_t camera_timestamp_us = prt_packet->GetTimeStamp();
                int64_t camera_time_ns = static_cast<int64_t>(camera_timestamp_us * NANOSECONDS_PER_MICROSECOND);
                
                // Calculate offset: system_time_when_sent - camera_time_when_recorded
                int64_t ros_time_ns = trigger_ros_time.nanoseconds();
                int64_t offset = ros_time_ns - camera_time_ns;
                offset_samples.push_back(offset);
                
                RCLCPP_DEBUG(get_logger(), "Calibration sample %d: offset = %ld ns", i, offset);
                event_received = true;
              }
            }
          }
        } else if (response == CNetwork::ResponseType::timeout) {
          // Just a timeout, keep trying
          attempts++;
        } else {
          // Error or disconnect
          RCLCPP_WARN(get_logger(), "Error receiving packet during calibration");
          attempts++;
        }
      }
      
      if (!event_received) {
        RCLCPP_WARN(get_logger(), "Calibration event %d not received after timeout", i);
      }
      
      // Small delay between samples
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Release QTM control
    port_protocol_.ReleaseControl();
    RCLCPP_INFO(get_logger(), "Released QTM control after calibration");
  }
  
  // Calculate average offset
  if (offset_samples.empty()) {
    RCLCPP_ERROR(get_logger(), "No calibration samples collected, using zero offset");
    timestamp_offset_ns_ = 0;
    timestamp_offset_calibrated_ = false;
    return;
  }
  
  int64_t sum = 0;
  for (int64_t offset : offset_samples) {
    sum += offset;
  }
  timestamp_offset_ns_ = sum / static_cast<int64_t>(offset_samples.size());
  timestamp_offset_calibrated_ = true;
  
  // Calculate standard deviation for information
  int64_t variance_sum = 0;
  for (int64_t offset : offset_samples) {
    int64_t diff = offset - timestamp_offset_ns_;
    variance_sum += diff * diff;
  }
  // Use sample standard deviation (divide by n-1) for better accuracy with small samples
  size_t divisor = offset_samples.size() > 1 ? offset_samples.size() - 1 : 1;
  double std_dev_ns = std::sqrt(static_cast<double>(variance_sum) / divisor);
  
  RCLCPP_INFO(get_logger(), "Timestamp offset calibration complete:");
  RCLCPP_INFO(get_logger(), "  Samples collected: %zu", offset_samples.size());
  RCLCPP_INFO(get_logger(), "  Average offset: %ld ns (%.6f s)", 
              timestamp_offset_ns_, timestamp_offset_ns_ / 1e9);
  RCLCPP_INFO(get_logger(), "  Standard deviation: %.6f ms", std_dev_ns / 1e6);
}


void QualisysDriver::initParameters()
{
  declare_parameter<std::string>("host_name", "mocap");
  declare_parameter<int>("port", 22222);
  declare_parameter<int>("last_frame_number", 0);
  declare_parameter<int>("frame_count", 0);
  declare_parameter<int>("dropped_frame_count", 0);
  declare_parameter<std::string>("qos_history_policy", "keep_all");
  declare_parameter<std::string>("qos_reliability_policy", "best_effort");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<bool>("use_markers_with_id", true);
  declare_parameter<int>("publish_rate", 10);
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<bool>("use_system_timestamp", true);
  declare_parameter<bool>("calibrate_timestamp_offset", false);
  declare_parameter<int>("calibration_samples", 10);

  get_parameter<std::string>("host_name", host_name_);
  get_parameter<int>("port", port_);
  get_parameter<int>("last_frame_number", last_frame_number_);
  get_parameter<int>("frame_count", frame_count_);
  get_parameter<int>("dropped_frame_count", dropped_frame_count_);
  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);
  get_parameter<bool>("use_markers_with_id", use_markers_with_id_);
  get_parameter<int>("publish_rate", publish_rate_);
  get_parameter<std::string>("frame_id", frame_id_);
  get_parameter<bool>("use_system_timestamp", use_system_timestamp_);
  get_parameter<bool>("calibrate_timestamp_offset", calibrate_timestamp_offset_);
  get_parameter<int>("calibration_samples", calibration_samples_);

  // Initialize calibration state
  timestamp_offset_ns_ = 0;
  timestamp_offset_calibrated_ = false;

  RCLCPP_INFO(get_logger(), "Param host_name: %s", host_name_.c_str());
  RCLCPP_INFO(get_logger(), "Param port: %d", port_);
  RCLCPP_INFO(get_logger(), "Param last_frame_number: %d", last_frame_number_);
  RCLCPP_INFO(get_logger(), "Param frame_count: %d", frame_count_);
  RCLCPP_INFO(get_logger(), "Param dropped_frame_count: %d", dropped_frame_count_);
  RCLCPP_INFO(get_logger(), "Param qos_history_policy: %s", qos_history_policy_.c_str());
  RCLCPP_INFO(get_logger(), "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
  RCLCPP_INFO(get_logger(), "Param qos_depth: %d", qos_depth_);
  RCLCPP_INFO(get_logger(), "Param use_markers_with_id: %s", use_markers_with_id_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Param publish_rate: %d", publish_rate_);
  RCLCPP_INFO(get_logger(), "Param frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "Param use_system_timestamp: %s", use_system_timestamp_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Param calibrate_timestamp_offset: %s", calibrate_timestamp_offset_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Param calibration_samples: %d", calibration_samples_);
}
