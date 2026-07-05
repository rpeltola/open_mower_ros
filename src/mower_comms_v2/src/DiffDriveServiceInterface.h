//
// Created by clemens on 25.07.24.
//

#ifndef DIFFDRIVESERVICEINTERFACE_H
#define DIFFDRIVESERVICEINTERFACE_H

#include <geometry_msgs/Twist.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <ros/ros.h>

#include <DiffDriveServiceInterfaceBase.hpp>

class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
 public:
  DiffDriveServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                            const ros::Publisher& actual_twist_publisher,
                            const ros::Publisher& left_esc_status_publisher,
                            const ros::Publisher& right_esc_status_publisher, double ticks_per_meter,
                            double wheel_distance, uint8_t control_mode, float loop_kp, float loop_ki, float loop_max,
                            float loop_slew, double max_linear_speed, double max_angular_speed)
      : DiffDriveServiceInterfaceBase(service_id, ctx),
        actual_twist_publisher_(actual_twist_publisher),
        left_esc_status_publisher_(left_esc_status_publisher),
        right_esc_status_publisher_(right_esc_status_publisher),
        wheel_distance_(wheel_distance),
        ticks_per_meter_(ticks_per_meter),
        control_mode_(control_mode),
        loop_kp_(loop_kp),
        loop_ki_(loop_ki),
        loop_max_(loop_max),
        loop_slew_(loop_slew),
        max_linear_speed_(max_linear_speed),
        max_angular_speed_(max_angular_speed) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

  /**
   * Convenience function to transmit the twist from a ROS message
   * @param msg The ROS message
   */
  void SendTwist(const geometry_msgs::TwistConstPtr& msg);

  /**
   * Re-apply the tunable configuration (control mode + firmware speed-loop gains) at
   * runtime by re-sending the full configuration transaction. The firmware reads these
   * registers live, so this lets you switch modes and tune gains in the field without a
   * redeploy. Driven from the control_mode / loop_* rosparams.
   * @param mode 0 = duty, 1 = duty_loop
   * @param loop_kp / loop_ki / loop_max / loop_slew  speed-loop tuning (<0 / <=0 => firmware built-in)
   */
  void ApplyConfiguration(uint8_t mode, float loop_kp, float loop_ki, float loop_max, float loop_slew);

 protected:
  /**
   * Callback whenever an updated twist arrives
   * @param new_value the updated value
   * @param length length of array
   */
  void OnActualTwistChanged(const double* new_value, uint32_t length) override;
  void OnLeftESCTemperatureChanged(const float& new_value) override;
  void OnLeftESCCurrentChanged(const float& new_value) override;
  void OnRightESCTemperatureChanged(const float& new_value) override;
  void OnRightESCCurrentChanged(const float& new_value) override;
  void OnWheelTicksChanged(const uint32_t* new_value, uint32_t length) override;
  void OnLeftESCStatusChanged(const uint8_t& new_value) override;
  void OnRightESCStatusChanged(const uint8_t& new_value) override;
  void OnLeftESCRpmChanged(const float& new_value) override;
  void OnRightESCRpmChanged(const float& new_value) override;
  void OnLeftESCDutyCycleChanged(const float& new_value) override;
  void OnRightESCDutyCycleChanged(const float& new_value) override;
  void OnLeftESCInputVoltageChanged(const float& new_value) override;
  void OnRightESCInputVoltageChanged(const float& new_value) override;
  void OnLeftESCMotorTemperatureChanged(const float& new_value) override;
  void OnRightESCMotorTemperatureChanged(const float& new_value) override;
  void OnLeftESCTachoAbsoluteChanged(const uint32_t& new_value) override;
  void OnRightESCTachoAbsoluteChanged(const uint32_t& new_value) override;
  void OnLeftESCDirectionChanged(const uint8_t& new_value) override;
  void OnRightESCDirectionChanged(const uint8_t& new_value) override;
  void OnLeftESCFWMajorChanged(const uint8_t& new_value) override;
  void OnRightESCFWMajorChanged(const uint8_t& new_value) override;
  void OnLeftESCFWMinorChanged(const uint8_t& new_value) override;
  void OnRightESCFWMinorChanged(const uint8_t& new_value) override;

 private:
  // Send the full configuration transaction (all required registers) to the firmware.
  void WriteConfiguration();

  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;

 private:
  // Store the seq number for the actual twist message
  uint32_t seq = 0;
  std::mutex state_mutex_{};

 public:
  const ros::Publisher& actual_twist_publisher_;
  const ros::Publisher& left_esc_status_publisher_;
  const ros::Publisher& right_esc_status_publisher_;
  double wheel_distance_;
  double ticks_per_meter_;
  // Drive control mode written to the firmware's Control Mode register:
  //   0 = duty (open-loop, default), 1 = duty_loop (firmware PI on wheel speed -> duty)
  uint8_t control_mode_;
  // Firmware speed-loop overrides (<0 for Kp/Ki, <=0 for max/slew => use firmware built-in).
  float loop_kp_;
  float loop_ki_;
  float loop_max_;
  float loop_slew_;
  // Translation layer: the high-level stack still emits a normalized [-1,1] twist, but the
  // firmware now expects PHYSICAL units. These scale normalized 1.0 -> real max speed
  // (m/s and rad/s) in SendTwist so behaviour is unchanged. TODO: drop once the stack
  // (planner + joystick) emits real m/s and these become 1.0.
  double max_linear_speed_;
  double max_angular_speed_;

  // Store the latest ESC state
  mower_msgs::ESCStatus left_esc_state_{};
  mower_msgs::ESCStatus right_esc_state_{};
};

#endif  // DIFFDRIVESERVICEINTERFACE_H
