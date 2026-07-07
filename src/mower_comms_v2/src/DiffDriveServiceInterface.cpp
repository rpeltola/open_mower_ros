//
// Created by clemens on 25.07.24.
//

#include "DiffDriveServiceInterface.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

void DiffDriveServiceInterface::WriteConfiguration() {
  // A configuration transaction must contain ALL required registers, or the firmware
  // rejects it ("did not contain all required registers"). So always write the full set.
  StartTransaction(true);
  SetRegisterWheelDistance(wheel_distance_);
  SetRegisterWheelTicksPerMeter(ticks_per_meter_);
  // 0 = duty (default), 1 = duty_loop (firmware PI on measured wheel speed -> duty).
  SetRegisterControlMode(control_mode_);
  // Speed-loop overrides (<0 for Kp/Ki, <=0 for max/slew => firmware built-in per-mode value).
  SetRegisterLoopKp(loop_kp_);
  SetRegisterLoopKi(loop_ki_);
  SetRegisterLoopMaxOutput(loop_max_);
  SetRegisterLoopSlew(loop_slew_);
  CommitTransaction();
}

bool DiffDriveServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  WriteConfiguration();
  return true;
}

void DiffDriveServiceInterface::ApplyConfiguration(uint8_t mode, float loop_kp, float loop_ki, float loop_max,
                                                   float loop_slew) {
  control_mode_ = mode;
  loop_kp_ = loop_kp;
  loop_ki_ = loop_ki;
  loop_max_ = loop_max;
  loop_slew_ = loop_slew;
  // Re-send the full configuration; the firmware reads these registers live.
  WriteConfiguration();
}

void DiffDriveServiceInterface::SendTwist(const geometry_msgs::TwistConstPtr& msg) {
  // TRANSLATION LAYER. The high-level stack (FTC planner, joystick) still emits the legacy
  // normalized/duty-equivalent twist, but the firmware now expects PHYSICAL units:
  // linear.x [m/s], angular.z [rad/s]. Scale here so a full-scale command maps to the real
  // max speed, reproducing the pre-physical firmware exactly (default 0.5 m/s and 0.5 rad/s
  // = the old kMaxWheelSpeedMps). This is the single seam between the normalized ROS stack
  // and the physical firmware; when the stack is migrated to emit real m/s, set the scales
  // to 1.0 and this becomes a no-op.
  double data[6]{};
  data[0] = msg->linear.x * max_linear_speed_;
  data[1] = msg->linear.y;
  data[2] = msg->linear.z;
  data[3] = msg->angular.x;
  data[4] = msg->angular.y;
  data[5] = msg->angular.z * max_angular_speed_;
  SendControlTwist(data, 6);
}

void DiffDriveServiceInterface::OnActualTwistChanged(const double* new_value, uint32_t length) {
  // 3 linear, 3 angular
  if (length == 6) {
    // Convert to ROS and publish
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.header.seq = seq++;
    twist.twist.linear.x = new_value[0];
    twist.twist.linear.y = new_value[1];
    twist.twist.linear.z = new_value[2];
    twist.twist.angular.x = new_value[3];
    twist.twist.angular.y = new_value[4];
    twist.twist.angular.z = new_value[5];

    actual_twist_publisher_.publish(twist);
  }
}

void DiffDriveServiceInterface::OnLeftESCCurrentChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.current = new_value;
}

void DiffDriveServiceInterface::OnRightESCTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.temperature_pcb = new_value;
}

void DiffDriveServiceInterface::OnRightESCCurrentChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.current = new_value;
}

void DiffDriveServiceInterface::OnWheelTicksChanged(const uint32_t* new_value, uint32_t length) {
  if (length != 2) return;
  left_esc_state_.tacho = new_value[0];
  right_esc_state_.tacho = new_value[1];
}

void DiffDriveServiceInterface::OnLeftESCTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.temperature_pcb = new_value;
}

void DiffDriveServiceInterface::OnLeftESCRpmChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.rpm = static_cast<int16_t>(new_value);
}

void DiffDriveServiceInterface::OnRightESCRpmChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.rpm = static_cast<int16_t>(new_value);
}

void DiffDriveServiceInterface::OnLeftESCDutyCycleChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.duty_cycle = new_value;
}

void DiffDriveServiceInterface::OnRightESCDutyCycleChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.duty_cycle = new_value;
}

void DiffDriveServiceInterface::OnLeftESCInputVoltageChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.input_voltage = new_value;
}

void DiffDriveServiceInterface::OnRightESCInputVoltageChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.input_voltage = new_value;
}

void DiffDriveServiceInterface::OnLeftESCMotorTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.temperature_motor = new_value;
}

void DiffDriveServiceInterface::OnRightESCMotorTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.temperature_motor = new_value;
}

void DiffDriveServiceInterface::OnLeftESCTachoAbsoluteChanged(const uint32_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.tacho_absolute = new_value;
}

void DiffDriveServiceInterface::OnRightESCTachoAbsoluteChanged(const uint32_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.tacho_absolute = new_value;
}

void DiffDriveServiceInterface::OnLeftESCDirectionChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.direction = new_value;
}

void DiffDriveServiceInterface::OnRightESCDirectionChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.direction = new_value;
}

void DiffDriveServiceInterface::OnLeftESCFWMajorChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.fw_major = new_value;
}

void DiffDriveServiceInterface::OnRightESCFWMajorChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.fw_major = new_value;
}

void DiffDriveServiceInterface::OnLeftESCFWMinorChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.fw_minor = new_value;
}

void DiffDriveServiceInterface::OnRightESCFWMinorChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.fw_minor = new_value;
}

void DiffDriveServiceInterface::OnServiceConnected(uint16_t service_id) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  right_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
}

void DiffDriveServiceInterface::OnLeftESCStatusChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = new_value;
}

void DiffDriveServiceInterface::OnRightESCStatusChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.status = new_value;
}

void DiffDriveServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  right_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
}

void DiffDriveServiceInterface::OnTransactionEnd() {
  // Publish values to ROS
  left_esc_status_publisher_.publish(left_esc_state_);
  right_esc_status_publisher_.publish(right_esc_state_);
}
