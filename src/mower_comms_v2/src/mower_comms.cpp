//
// Created by Clemens Elflein on 15.03.22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <mower_msgs/MowerControlSrv.h>
#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/Imu.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>
#include <std_msgs/String.h>

#include "../../../services/service_ids.h"
#include "BmsServiceInterface.h"
#include "DiffDriveServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "GpsServiceInterface.h"
#include "HighLevelServiceInterface.h"
#include "ImuServiceInterface.h"
#include "InputServiceInterface.h"
#include "MowerServiceInterface.h"
#include "PowerServiceInterface.h"

ros::Publisher status_pub;
ros::Publisher nmea_pub;
ros::Publisher power_pub;
ros::Publisher bms_pub;
ros::Publisher gps_position_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;
ros::Publisher action_pub;

ros::Publisher sensor_imu_pub;

ros::ServiceClient highLevelClient;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;
std::unique_ptr<PowerServiceInterface> power_service = nullptr;
std::unique_ptr<BmsServiceInterface> bms_service = nullptr;
std::unique_ptr<GpsServiceInterface> gps_service = nullptr;
std::unique_ptr<InputServiceInterface> input_service = nullptr;
std::unique_ptr<HighLevelServiceInterface> high_level_service = nullptr;

xbot::serviceif::Context ctx{};

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest& req, mower_msgs::EmergencyStopSrvResponse& res) {
  emergency_service->SetHighLevelEmergency(req.emergency);
  return true;
}

void velReceived(const geometry_msgs::Twist::ConstPtr& msg) {
  diff_drive_service->SendTwist(msg);
}

// Map a control_mode string to the firmware Control Mode register value.
// Returns false if the string is not a known mode.
bool controlModeFromString(const std::string& s, uint8_t& mode) {
  if (s == "duty") {
    mode = 0;
  } else if (s == "duty_loop") {
    mode = 1;
  } else {
    return false;
  }
  return true;
}

// Read a numeric rosparam as float, accepting either an int or a double on the server
// (rosparam set from the CLI stores whole numbers as int).
bool getFloatParam(const std::string& name, float& out) {
  double d;
  if (ros::param::get(name, d)) {
    out = static_cast<float>(d);
    return true;
  }
  int i;
  if (ros::param::get(name, i)) {
    out = static_cast<float>(i);
    return true;
  }
  return false;
}

// Config last applied to the firmware; used to detect rosparam changes.
uint8_t applied_control_mode = 0;
float applied_loop_kp = -1.0f;
float applied_loop_ki = -1.0f;
float applied_loop_max = 0.0f;
float applied_loop_slew = 0.0f;

// Re-read the control_mode + loop tuning params and re-apply live when any changes, so
//   rosparam set /ll/services/diff_drive/{control_mode,loop_kp,loop_ki,loop_max,loop_slew}
// takes effect without a restart. The params ARE the interface - no custom topic.
void applyDiffDriveParamsTimerTask(const ros::TimerEvent&) {
  if (diff_drive_service == nullptr) {
    return;
  }
  std::string s;
  if (!ros::param::get("/ll/services/diff_drive/control_mode", s)) {
    return;
  }
  uint8_t mode = 0;
  if (!controlModeFromString(s, mode)) {
    return;  // ignore invalid values, keep the current config
  }
  float kp = applied_loop_kp, ki = applied_loop_ki, max = applied_loop_max, slew = applied_loop_slew;
  getFloatParam("/ll/services/diff_drive/loop_kp", kp);
  getFloatParam("/ll/services/diff_drive/loop_ki", ki);
  getFloatParam("/ll/services/diff_drive/loop_max", max);
  getFloatParam("/ll/services/diff_drive/loop_slew", slew);

  if (mode == applied_control_mode && kp == applied_loop_kp && ki == applied_loop_ki && max == applied_loop_max &&
      slew == applied_loop_slew) {
    return;
  }
  applied_control_mode = mode;
  applied_loop_kp = kp;
  applied_loop_ki = ki;
  applied_loop_max = max;
  applied_loop_slew = slew;
  diff_drive_service->ApplyConfiguration(mode, kp, ki, max, slew);
  ROS_INFO_STREAM("diff_drive config applied: mode=" << s << " loop_kp=" << kp << " loop_ki=" << ki
                                                     << " loop_max=" << max << " loop_slew=" << slew);
}

void rtcmReceived(const rtcm_msgs::Message& msg) {
  static std::vector<uint8_t> rtcm_buffer{};
  static ros::Time last_time_sent{0};
  ros::Time now = ros::Time::now();
  // Append the bytes to the buffer
  rtcm_buffer.insert(rtcm_buffer.end(), msg.message.begin(), msg.message.end());
  // In order to not spam after each received byte, limit packets to 5Hz and to max 1k of size
  if (rtcm_buffer.size() < 1000 && (now - last_time_sent).toSec() < 0.2) return;
  last_time_sent = now;
  gps_service->SendRTCM(rtcm_buffer.data(), rtcm_buffer.size());
  rtcm_buffer.clear();
}

void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent&) {
  emergency_service->Heartbeat();
}

void actionReceived(const std_msgs::String::ConstPtr& action) {
  input_service->OnAction(action->data);
}

void sendMowerEnabledTimerTask(const ros::TimerEvent& e) {
  mower_service->Tick();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest& req, mower_msgs::MowerControlSrvResponse& res) {
  mower_service->SetMowerEnabled(req.mow_enabled);
  return true;
}

static void spdlog_cb(const spdlog::details::log_msg& msg) {
  ros::console::Level level = ros::console::Level::Info;
  switch (msg.level) {
    case spdlog::level::level_enum::trace:
    case spdlog::level::level_enum::debug: level = ros::console::Level::Debug; break;
    case spdlog::level::level_enum::info: break;
    case spdlog::level::level_enum::warn: level = ros::console::Level::Warn; break;
    case spdlog::level::level_enum::err: level = ros::console::Level::Error; break;
    case spdlog::level::level_enum::critical: level = ros::console::Level::Fatal; break;
    case spdlog::level::level_enum::off: return;
  }
  ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%.*s", static_cast<int>(msg.payload.size()), msg.payload.data());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_v2");

  {
    auto sink = std::make_shared<spdlog::sinks::callback_sink_mt>(spdlog_cb);
    auto logger = std::make_shared<spdlog::logger>("", std::move(sink));
    spdlog::set_default_logger(logger);
  }

  ros::NodeHandle n;
  ros::NodeHandle paramNh("/ll");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");
  action_pub = n.advertise<std_msgs::String>("xbot/action", 1);

  std::string bind_ip = "0.0.0.0";
  paramNh.getParam("bind_ip", bind_ip);
  ROS_INFO_STREAM("Bind IP (Robot Internal): " << bind_ip);
  xbot::serviceif::SetShutdownCallback([] { ros::requestShutdown(); });
  ctx = xbot::serviceif::Start(true, bind_ip);

  // Emergency service
  emergency_pub = n.advertise<mower_msgs::Emergency>("ll/emergency", 1);
  emergency_service = std::make_unique<EmergencyServiceInterface>(xbot::service_ids::EMERGENCY, ctx, emergency_pub);
  emergency_service->Start();

  // Diff drive service
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ll/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/right_esc_status", 1);
  double wheel_ticks_per_m = 0.0;
  double wheel_distance_m = 0.0;
  if (!paramNh.getParam("services/diff_drive/ticks_per_m", wheel_ticks_per_m)) {
    ROS_ERROR("Need to provide param services/diff_drive/ticks_per_m");
    return 1;
  }
  if (!paramNh.getParam("services/diff_drive/wheel_distance_m", wheel_distance_m)) {
    ROS_ERROR("Need to provide param services/diff_drive/wheel_distance_m");
    return 1;
  }
  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  // Opt-in drive control mode written to the firmware. Defaults to "duty" (open-loop,
  // unchanged behavior). Accepts:
  //   duty      - open-loop duty (default)
  //   duty_loop - firmware PI on measured wheel speed -> duty
  std::string control_mode_str = "duty";
  paramNh.getParam("services/diff_drive/control_mode", control_mode_str);
  uint8_t control_mode = 0;
  if (!controlModeFromString(control_mode_str, control_mode)) {
    ROS_WARN_STREAM("Unknown diff_drive control_mode '" << control_mode_str << "', falling back to 'duty'");
    control_mode_str = "duty";
    control_mode = 0;
  }
  ROS_INFO_STREAM("Drive control mode: " << control_mode_str << " (" << static_cast<int>(control_mode) << ")");

  // Firmware speed-loop gain overrides. Sentinels (Kp/Ki < 0, max <= 0) mean "use the
  // firmware built-in per-mode gains". Tune live via rosparam set of these keys.
  float loop_kp = -1.0f, loop_ki = -1.0f, loop_max = 0.0f, loop_slew = 0.0f;
  getFloatParam("/ll/services/diff_drive/loop_kp", loop_kp);
  getFloatParam("/ll/services/diff_drive/loop_ki", loop_ki);
  getFloatParam("/ll/services/diff_drive/loop_max", loop_max);
  getFloatParam("/ll/services/diff_drive/loop_slew", loop_slew);
  ROS_INFO_STREAM("Speed-loop tuning: loop_kp=" << loop_kp << " loop_ki=" << loop_ki << " loop_max=" << loop_max
                                                << " loop_slew=" << loop_slew << " (<0 / <=0 = firmware default)");

  // Baseline for the live param re-read (OnConfigurationRequested applies these values).
  applied_control_mode = control_mode;
  applied_loop_kp = loop_kp;
  applied_loop_ki = loop_ki;
  applied_loop_max = loop_max;
  applied_loop_slew = loop_slew;

  // Normalized -> physical translation for the firmware (see SendTwist). A full-scale
  // command maps to these real speeds. Defaults (0.5 m/s, 0.5 rad/s) reproduce the old
  // normalized behaviour; raise later as the stack moves to real units.
  double max_linear_speed = 0.5, max_angular_speed = 0.5;
  paramNh.getParam("services/diff_drive/max_linear_speed", max_linear_speed);
  paramNh.getParam("services/diff_drive/max_angular_speed", max_angular_speed);
  ROS_INFO_STREAM("Twist translation: max_linear_speed=" << max_linear_speed << " m/s, max_angular_speed="
                                                         << max_angular_speed << " rad/s");

  int baud_rate = 0;
  paramNh.getParam("services/gps/baud_rate", baud_rate);

  std::string protocol;
  paramNh.getParam("services/gps/protocol", protocol);

  int gps_port_index = 0;
  paramNh.getParam("services/gps/port_index", gps_port_index);

  if (baud_rate == 0 || protocol.empty()) {
    ROS_ERROR("Need to specify GPS protocol and baud rate!");
    return 1;
  }

  ROS_INFO_STREAM("GPS protocol: " << protocol << ", baud rate: " << baud_rate
                                   << ", gps port index:" << gps_port_index);

  diff_drive_service = std::make_unique<DiffDriveServiceInterface>(
      xbot::service_ids::DIFF_DRIVE, ctx, actual_twist_pub, status_left_esc_pub, status_right_esc_pub,
      wheel_ticks_per_m, wheel_distance_m, control_mode, loop_kp, loop_ki, loop_max, loop_slew, max_linear_speed,
      max_angular_speed);
  diff_drive_service->Start();

  // Mower service
  status_pub = n.advertise<mower_msgs::Status>("ll/mower_status", 1);
  mower_service = std::make_unique<MowerServiceInterface>(xbot::service_ids::MOWER, ctx, status_pub);
  mower_service->Start();

  // IMU service
  std::string imu_axis_config;
  paramNh.getParam("services/imu/axis_config", imu_axis_config);
  ROS_INFO_STREAM("IMU axis config: " << imu_axis_config);
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("ll/imu/data_raw", 1);
  imu_service = std::make_unique<ImuServiceInterface>(xbot::service_ids::IMU, ctx, sensor_imu_pub, imu_axis_config);
  imu_service->Start();

  // Power service
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);

  // Mainly for monitoring and informational purposes
  float battery_full_voltage;
  float battery_empty_voltage;
  float battery_critical_voltage;
  float battery_critical_high_voltage;
  if (!paramNh.getParam("services/power/battery_full_voltage", battery_full_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_full_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_empty_voltage", battery_empty_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_empty_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_critical_voltage", battery_critical_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_critical_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_critical_high_voltage", battery_critical_high_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_critical_high_voltage");
    return 1;
  }

  // Optional charger configuration
  float charge_voltage = -1.0f;
  float charge_current = -1.0f;
  float charge_termination_current = -1.0f;
  float charge_precharge_current = -1.0f;
  int charge_recharge_voltage = -1;
  paramNh.getParam("services/power/charge_voltage", charge_voltage);
  paramNh.getParam("services/power/charge_current", charge_current);
  paramNh.getParam("services/power/charge_termination_current", charge_termination_current);
  paramNh.getParam("services/power/charge_pre_charge_current", charge_precharge_current);
  paramNh.getParam("services/power/charge_re_charge_voltage", charge_recharge_voltage);

  // Optional settings also required for charger DPM (dynamic power management)
  float system_current = -1.0f;  // Max. current allowed to be drawn from wall AC/DC
  paramNh.getParam("services/power/system_current", system_current);
  bool override_hw_charge_current_limit = false;
  paramNh.getParam("services/power/dangerously_override_hardware_charge_current_limit",
                   override_hw_charge_current_limit);
  power_service = std::make_unique<PowerServiceInterface>(
      xbot::service_ids::POWER, ctx, power_pub, battery_full_voltage, battery_empty_voltage, battery_critical_voltage,
      battery_critical_high_voltage, charge_voltage, charge_current, charge_termination_current,
      charge_precharge_current, charge_recharge_voltage, system_current, override_hw_charge_current_limit);
  power_service->Start();

  // BMS service
  bms_pub = n.advertise<mower_msgs::Bms>("ll/bms", 1);
  bms_service = std::make_unique<BmsServiceInterface>(xbot::service_ids::BMS, ctx, bms_pub);
  bms_service->Start();

  // GPS service
  double datum_lat, datum_long, datum_height;
  bool has_datum = true;
  has_datum &= paramNh.getParam("services/gps/datum_lat", datum_lat);
  has_datum &= paramNh.getParam("services/gps/datum_long", datum_long);
  has_datum &= paramNh.getParam("services/gps/datum_height", datum_height);
  if (!has_datum) {
    ROS_ERROR_STREAM("You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
    return 2;
  }
  ROS_INFO_STREAM("Datum: " << datum_lat << ", " << datum_long << ", " << datum_height);
  gps_position_pub = n.advertise<xbot_msgs::AbsolutePose>("ll/position/gps", 1);
  nmea_pub = n.advertise<nmea_msgs::Sentence>("ll/position/gps/nmea", 1);
  bool absolute_coords = true;
  paramNh.getParam("services/gps/absolute_coords", absolute_coords);
  gps_service = std::make_unique<GpsServiceInterface>(xbot::service_ids::GPS, ctx, gps_position_pub, nmea_pub,
                                                      datum_lat, datum_long, datum_height, baud_rate, protocol,
                                                      gps_port_index, absolute_coords);
  gps_service->Start();

  // Input service
  {
    std::string config_file = paramNh.param<std::string>("services/input/config_file", "");
    input_service = std::make_unique<InputServiceInterface>(xbot::service_ids::INPUT, ctx, config_file, action_pub);
    input_service->Start();
  }

  // HighLevel service
  high_level_service = std::make_unique<HighLevelServiceInterface>(xbot::service_ids::HIGH_LEVEL, ctx);
  high_level_service->Start();

  // All subscriptions, timers and service servers are registered after all service interfaces are
  // fully constructed, so callbacks can never fire on null pointers.
  ros::ServiceServer mow_service = n.advertiseService("ll/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer ros_emergency_service = n.advertiseService("ll/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("ll/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber rtcm_sub = n.subscribe("ll/position/gps/rtcm", 0, rtcmReceived);
  // ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer control_mode_timer = n.createTimer(ros::Duration(0.5), applyDiffDriveParamsTimerTask);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.5), sendEmergencyHeartbeatTimerTask);
  ros::Timer publish_timer_2 = n.createTimer(ros::Duration(5.0), sendMowerEnabledTimerTask);
  ros::Subscriber action_sub = n.subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("All mower_comms_v2 services started");

  ros::spin();
  xbot::serviceif::Stop();

  return 0;
}
