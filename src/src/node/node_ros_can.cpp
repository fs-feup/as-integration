#include "node/node_ros_can.hpp"

#include <canlib.h>

#include <bitset>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "safety/safety-mechanisms.hpp"
#include "utils/constants.hpp"
#include "utils/temp_converters.hpp"
#include "utils/utils.hpp"

RosCan::RosCan(std::shared_ptr<ICanLibWrapper> can_lib_wrapper_param)
    : Node("ros_can"), can_lib_wrapper_(std::move(can_lib_wrapper_param)) {
  // Publishers
  operational_status_ = this->create_publisher<custom_interfaces::msg::OperationalStatus>(
      "/vehicle/operational_status", 10);
  data_log_info_1_pub_ =
      this->create_publisher<custom_interfaces::msg::DataLogInfo1>("/vehicle/data_log_info_1", 10);
  data_log_info_2_pub_ =
      this->create_publisher<custom_interfaces::msg::DataLogInfo2>("/vehicle/data_log_info_2", 10);
  rl_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rl_rpm", 10);
  rr_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rr_rpm", 10);
  motor_rpm_pub_ =
      this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/motor_rpm", 10);
  motor_temp_pub_ =
      this->create_publisher<custom_interfaces::msg::Temperature>("/vehicle/motor_temp", 10);
  inverter_temp_pub_ =
      this->create_publisher<custom_interfaces::msg::Temperature>("/vehicle/inverter_temp", 10);
  imu_acc_pub_ =
      this->create_publisher<custom_interfaces::msg::ImuAcceleration>("/vehicle/acceleration", 10);
  imu_angular_velocity_pub_ =
      this->create_publisher<custom_interfaces::msg::YawPitchRoll>("/vehicle/angular_velocity", 10);
  bosch_steering_angle_publisher_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/bosch_steering_angle", 10);

  cells_temps_pub_ =
      this->create_publisher<custom_interfaces::msg::CellsTemps>("vehicle/cells/temperature", 10);
  _inverter_errors_pub_ =
      this->create_publisher<custom_interfaces::msg::BamocarErrors>("/vehicle/inverter_errors", 10);
  _bamocar_current_pub =
      this->create_publisher<std_msgs::msg::Int32>("/vehicle/bamocar_current", 10);

  steering_motor_state_pub_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/steering_motor_state", 10);
  steering_motor_temperature =
      this->create_publisher<std_msgs::msg::Int8>("/vehicle/steering_motor_temperature", 10);
  steering_motor_current =
      this->create_publisher<std_msgs::msg::Float64>("/vehicle/steering_motor_current", 10);
  steering_motor_error =
      this->create_publisher<std_msgs::msg::Int8>("/vehicle/steering_motor_error", 10);

  hydraulic_line_pressure_publisher_ =
      this->create_publisher<custom_interfaces::msg::HydraulicLinePressure>(
          "/vehicle/hydraulic_line_pressure", 10);

  fr_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/fr_rpm", 10);
  fl_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/fl_rpm", 10);

  inverter_voltage_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/vehicle/inverter_voltage", 10);

  bms_errors_pub_ =
      this->create_publisher<custom_interfaces::msg::BmsErrors>("/vehicle/battery_errors", 10);

  apps_higher_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle/apps/higher", 10);
  apps_lower_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle/apps/lower", 10);

  implausability_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vehicle/implausability", 10);
  driving_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/vehicle/driving_state", 10);

  this->can_line_0_stats_pub_ = this->create_publisher<custom_interfaces::msg::CanStatistics>(
      "/vehicle/can_line_0_stats", 10);
  this->can_line_1_stats_pub_ = this->create_publisher<custom_interfaces::msg::CanStatistics>(
      "/vehicle/can_line_1_stats", 10);

  // Subscritpions
  control_listener_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));

  fr_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>(
      "/vehicle/fr_rpm", 10);
  fl_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>(
      "/vehicle/fl_rpm", 10);

  manual_throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/vehicle/manual_throttle", 10);
  
  manual_brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/vehicle/manual_brake", 10);

  apps_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/vehicle/apps/error", 10);

  // Subscriptions
  control_listener_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));

  this->perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "/perception/cones", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          auto const &cone_array = msg->cone_array;
          this->cones_count_actual_ = static_cast<uint8_t>(cone_array.size());
      });

  this->velocities_subscription_ = this->create_subscription<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", 10,
      [this](const custom_interfaces::msg::Velocities::SharedPtr msg) {
          this->speed_actual_ = static_cast<uint8_t>(msg->velocity_x) * 3.6; // convert to km/h
          this->yaw_rate_ = static_cast<int16_t>(msg->angular_velocity * 180/ M_PI); // convert to degrees/s
      });

  this->map_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "/state_estimation/map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          auto const &cone_array = msg->cone_array;
          this->cones_count_all_ = static_cast<uint16_t>(cone_array.size());
      });

  this->lap_counter_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/state_estimation/lap_counter", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        this->lap_counter_ = static_cast<int>(msg->data);
      });

  this->path_subscription_ = this->create_subscription<custom_interfaces::msg::PathPointArray>(
      "/path_planning/path", 10, [this](const custom_interfaces::msg::PathPointArray::SharedPtr msg) {
        if (msg->pathpoint_array.size() > 0) {
          this->speed_target_ = static_cast<uint8_t>(msg->pathpoint_array[0].v) * 3.6; // convert to km/h
        } else {
          this->speed_target_ = 0; // No path points, set speed target to
        }
      });

  this->movella_imu_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/filter/free_acceleration", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          this->acceleration_longitudinal_ = msg->vector.x;
          this->acceleration_lateral_ = msg->vector.y;
      });
  
  // Services
  emergency_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/as_srv/emergency",
      std::bind(&RosCan::emergency_callback, this, std::placeholders::_1, std::placeholders::_2));
  mission_finished_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/as_srv/mission_finished", std::bind(&RosCan::mission_finished_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

  bosch_steering_angle_reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/as_srv/bosch_sa_reset", std::bind(&RosCan::bosch_sa_reset_callack, this,
                                          std::placeholders::_1, std::placeholders::_2));

  // Timers
  timer_ = this->create_wall_timer(std::chrono::microseconds(500),
                                   std::bind(&RosCan::can_sniffer, this));
  timer_alive_msg_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&RosCan::alive_msg_callback, this));

  dv_timer_ = this->create_wall_timer(std::chrono::milliseconds(15),
                                   std::bind(&RosCan::dv_messages_callback, this));

  _cell_temps_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&RosCan::publish_cell_temperatures, this));
  
  can_statistics_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&RosCan::can_statistics_callback, this));

  // initialize the CAN library
  canInitializeLibrary();
  // A channel to a CAN circuit is opened. The channel depend on the hardware

  hnd0_ = canOpenChannel(0, canOPEN_EXCLUSIVE);  // TODO: this will be used only for SAS
  hnd1_ = canOpenChannel(1, canOPEN_EXCLUSIVE);  // TODO: this is for everything else
  // Setup CAN parameters for the channel
  stat_ = canSetBusParams(hnd0_, canBITRATE_500K, 0, 0, 4, 0, 0);  // check these values later
  stat_ = canSetBusParams(hnd1_, canBITRATE_1M, 0, 0, 4, 0, 0);    // check these values later

  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN parameters");
    return;
  }
  // There are different types of controllers, this is the default
  stat_ = canSetBusOutputControl(hnd0_, canDRIVER_NORMAL);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN controller");
    return;
  }
  stat_ = canSetBusOutputControl(hnd1_, canDRIVER_NORMAL);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN controller");
  }
  stat_ = canBusOn(hnd0_);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to turn on CAN bus");
  }
  stat_ = canBusOn(hnd1_);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to turn on CAN bus");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Node initialized");
  sleep(1);
}

// -------------- ROS TO CAN --------------

void RosCan::control_callback(custom_interfaces::msg::ControlCommand::SharedPtr msg) {
  // Check if the steering value is within the limits
  if (msg->steering < STEERING_LOWER_LIMIT || msg->steering > STEERING_UPPER_LIMIT) {
    RCLCPP_WARN(this->get_logger(), "Steering value out of range: %lf", msg->steering);
    return;
  }

  // Check if the throttle value is within the limits
  if (msg->throttle < THROTTLE_LOWER_LIMIT || msg->throttle > THROTTLE_UPPER_LIMIT) {
    RCLCPP_WARN(this->get_logger(), "Throttle value out of range");
    return;
  }

  // Convert to steering in actuator
  double steering_angle_command = 0.0;
  if (transform_steering_angle_command(msg->steering, steering_angle_command) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform steering angle command");
    return;
  }

  double steering_degrees = msg->steering * 180.0 / M_PI;
  this->steering_angle_target_ = static_cast<int8_t>(steering_degrees * 2);

  this->motor_moment_actual_ = static_cast<int8_t>(msg->throttle * 100);
  this->motor_moment_target_ = static_cast<int8_t>(msg->throttle * 100);


  if (this->go_signal_) {
    RCLCPP_INFO(this->get_logger(), "State is Driving: Steering: %f (radians), Throttle: %f",
                msg->steering, msg->throttle);

    send_steering_control(steering_angle_command);
    send_throttle_control(msg->throttle);
  } else {
    RCLCPP_INFO(this->get_logger(), "No go signal!");
  }
}

void RosCan::send_steering_control(double steering_angle_command) {
  long id = STEERING_COMMAND_CUBEM_ID;
  char buffer_steering[4];
  create_steering_angle_command(steering_angle_command, buffer_steering);
  void *steering_requestData = static_cast<void *>(buffer_steering);
  unsigned int steering_dlc = 4;
  unsigned int flag = canMSG_EXT;  // Steering motor used CAN Extended

  // Write the steering message to the CAN bus
  // DO NOT EVER EDIT THIS CODE BLOCK
  // CODE BLOCK START
  check_steering_safe(steering_requestData);  // DO NOT REMOVE EVER
  stat_ = can_lib_wrapper_->canWrite(hnd1_, id, steering_requestData, steering_dlc, flag);
  // CODE BLOCK END
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus: %d", stat_);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Write steering to CAN bus success: %f (degrees)",
                 steering_angle_command);
  }
}

void RosCan::send_throttle_control(double throttle_value_ros) {
  // Prepare the throttle message
  int throttle_command = static_cast<int>(throttle_value_ros * BAMOCAR_MAX_SCALE);
  RCLCPP_DEBUG(this->get_logger(), "Command after conversion: %d", throttle_command);
  // CRITICAL CHECK - DO NOT REMOVE
  // Limit brake command if needed
  if (throttle_command < 0) {
    throttle_command = std::max(
        throttle_command, max_torque_dynamic_limits(this->inverter_voltage_, this->motor_speed_));
  }

  RCLCPP_DEBUG(this->get_logger(), "Command after limiting: %d", throttle_command);
  long throttle_id = BAMO_COMMAND_ID;
  unsigned char buffer_throttle[3];
  buffer_throttle[0] = TORQUE_COMMAND_BAMO_BYTE;
  buffer_throttle[2] = (throttle_command >> 8) & 0xff;
  buffer_throttle[1] = throttle_command & 0xff;
  void *throttle_requestData = static_cast<void *>(buffer_throttle);
  unsigned int throttle_dlc = 3;
  unsigned int flag = 0;

  // Write the throttle message to the CAN bus
  // DO NOT EVER EDIT THIS CODE BLOCK
  // CODE BLOCK START
  check_throttle_safe(throttle_requestData);  // DO NOT REMOVE EVER
  stat_ = can_lib_wrapper_->canWrite(hnd1_, throttle_id, throttle_requestData, throttle_dlc, flag);
  // CODE BLOCK END
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus: %d ", stat_);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Write throttle to CAN bus success: %f", throttle_value_ros);
  }
}

void RosCan::emergency_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  response->success = true;

  RCLCPP_DEBUG(this->get_logger(), "Received emergency signal.");

  // Prepare the emergency message
  long id = AS_CU_NODE_ID;              // Set the CAN ID
  unsigned char data = EMERGENCY_CODE;  // Set the data
  void *requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_WARN(this->get_logger(), "Emergency signal received, sending to CAN");

  stat_ = can_lib_wrapper_->canWrite(hnd0_, id, requestData, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  response->success = true;

  RCLCPP_DEBUG(this->get_logger(), "Received mission finished.");

  // Prepare the emergency message
  long id = AS_CU_NODE_ID;                     // Set the CAN ID
  unsigned char data = MISSION_FINISHED_CODE;  // Set the data
  void *requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_INFO(this->get_logger(), "Mission finished signal received, sending to CAN");

  stat_ = can_lib_wrapper_->canWrite(hnd1_, id, requestData, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::bosch_sa_reset_callack(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  RCLCPP_DEBUG(this->get_logger(), "Received steering angle reset request.");

  int result = this->bosch_steering_angle_set_origin();

  if (result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering angle sensor");
    response->success = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Reset steering angle value!");
    response->success = true;
  }
}

void RosCan::alive_msg_callback() {
  long id = AS_CU_NODE_ID;
  unsigned char data = ALIVE_MESSAGE;
  void *msg = &data;
  unsigned int dlc = 1;
  unsigned int flag = 0;
  // RCLCPP_DEBUG(this->get_logger(), "Sending alive message from AS_CU_NODE");

  stat_ = can_lib_wrapper_->canWrite(hnd1_, id, msg, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write alive message to CAN bus");
  }
}

void RosCan::cubem_set_origin() {
  long id = SET_ORIGIN_CUBEM_ID;
  char buffer[1];
  buffer[0] = 0x00;
  void *steering_requestData = static_cast<void *>(buffer);
  unsigned int steering_dlc = 1;
  unsigned int flag = canMSG_EXT;  // Steering motor used CAN Extended
  RCLCPP_INFO(this->get_logger(), "Setting origin of steering controller");

  // Write the steering message to the CAN bus
  stat_ = can_lib_wrapper_->canWrite(hnd1_, id, steering_requestData, steering_dlc, flag);
  RCLCPP_INFO(this->get_logger(), "ORIGIN SET :)");
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering controller");
  }
}

// Not currently used, check header file for more info
int RosCan::bosch_steering_angle_set_origin() {
  long id = SET_ORIGIN_BOSCH_STEERING_ANGLE_ID;
  char buffer[8] = {0};
  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET;  // Reset message code
  void *request_data = static_cast<void *>(buffer);
  unsigned int dlc = 8;
  unsigned int flag = 0;

  // Reset previous origin
  stat_ = canWrite(hnd0_, id, request_data, dlc, flag);
  if (stat_ != canOK) {
    return 1;
  }
  sleep(1);  // Necessary for configuration to sink in

  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_SET;
  request_data = static_cast<void *>(buffer);

  // Set new origin
  stat_ = canWrite(hnd0_, id, request_data, dlc, flag);
  if (stat_ != canOK) {
    return 1;
  }
  return 0;
}

/**
 * @brief Function to turn ON and OFF the CAN BUS
 */
/*void RosCan::busStatus_callback(std_msgs::msg::String busStatus) {
  if (busStatus.data == "ON") {
    hnd0_ = canOpenChannel(0, canOPEN_CAN_FD);
    stat_ = canBusOn(hnd0_);
  } else if (busStatus.data == "OFF") {
  stat_ = canBusOff(hnd0_);
    canClose(hnd0_);
  }
}*/

// -------------- CAN TO ROS --------------

void RosCan::can_statistics_callback() {
  custom_interfaces::msg::CanStatistics stats0_msg;
  custom_interfaces::msg::CanStatistics stats1_msg;

  stats0_msg.header.stamp = this->get_clock()->now();
  stats1_msg.header.stamp = stats0_msg.header.stamp;

  bool success0 = read_can_statistics(hnd0_, stats0_msg);
  bool success1 = read_can_statistics(hnd1_, stats1_msg);

  if (success0) {
    can_line_0_stats_pub_->publish(stats0_msg);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to collect CAN statistics for line 0");
  }

  if (success1) {
    can_line_1_stats_pub_->publish(stats1_msg);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to collect CAN statistics for line 1");
  }
}

bool RosCan::read_can_statistics(canHandle handle, custom_interfaces::msg::CanStatistics& msg) {
  canBusStatistics stats;
  canRequestBusStatistics(handle);
  canStatus status = canGetBusStatistics(handle, &stats, sizeof(stats));

  if (status != canOK) {
    return false;
  }

  // Fill fields from canBusStatistics
  msg.load = static_cast<float>(stats.busLoad) / 100.0f;  // Convert 0–10000 → 0.00–100.00 %
  msg.overrun_count = stats.overruns;

  unsigned int txErr = 0, rxErr = 0, unused = 0;
  if (canReadErrorCounters(handle, &txErr, &rxErr, &unused) == canOK) {
    msg.tx_error_count = txErr;
    msg.rx_error_count = rxErr;
  }

  unsigned long flags = 0;
  if (canReadStatus(handle, &flags) == canOK) {
    if (flags & canSTAT_BUS_OFF)
      msg.status = 2;
    else if (flags & canSTAT_ERROR_PASSIVE || flags & canSTAT_ERROR_WARNING)
      msg.status = 1;
    else
      msg.status = 0;
  }

  return true;
}

void RosCan::can_sniffer() {
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;

  // Read from channel 0
  stat_ = can_lib_wrapper_->canRead(hnd0_, &id, &msg, &dlc, &flag, &time);
  while (stat_ == canOK) {
    can_interpreter(id, msg, dlc, flag, time);
    stat_ = can_lib_wrapper_->canRead(hnd0_, &id, &msg, &dlc, &flag, &time);
  }

  // Read from channel 1
  stat_ = can_lib_wrapper_->canRead(hnd1_, &id, &msg, &dlc, &flag, &time);
  while (stat_ == canOK) {
    can_interpreter(id, msg, dlc, flag, time);
    stat_ = can_lib_wrapper_->canRead(hnd1_, &id, &msg, &dlc, &flag, &time);
  }
}

void RosCan::can_interpreter(long id, const unsigned char msg[8], unsigned int dlc, unsigned int,
                             unsigned long) {
  switch (id) {
    case MASTER_ID: {
      can_interpreter_master(msg);
      break;
    }
    case DATA_LOGGER_SIGNALS_1: {
      RCLCPP_DEBUG(this->get_logger(), "Received Data Logger Signals 1 message");
      data_log_info_1_publisher(msg);
      break;
    }
    case DATA_LOGGER_SIGNALS_2: {
      RCLCPP_DEBUG(this->get_logger(), "Received Data Logger Signals 2 message");
      data_log_info_2_publisher(msg);
      break;
    }
    case IMU_ACC: {
      imu_acc_publisher(msg);
      break;
    }

    case IMU_GYRO: {
      imu_angular_velocity_publisher(msg);
      break;
    }

    case TEENSY_DASH: {
      dash_interpreter(msg);
      break;
    }
    case STEERING_CUBEM_ID: {
      steering_angle_cubem_publisher(msg);
      break;
    }
    case STEERING_BOSCH_ID: {
      steering_angle_bosch_publisher(msg);
      break;
    }
    case BAMO_RESPONSE_ID: {
      can_interpreter_bamocar(msg);
      break;
    }
    case BMS_THERMISTOR_ID: {
      bms_cells_temps_callback(msg);
      break;
    }
    case BMS_ERRORS_ID: {
      bms_errors_publisher(msg, dlc);
      break;
    }
    case ALL_TEMPS_ID:
    case ALL_TEMPS_ID + 1:
    case ALL_TEMPS_ID + 2:
    case ALL_TEMPS_ID + 3:
    case ALL_TEMPS_ID + 4:
    case ALL_TEMPS_ID + 5: {
      if (dlc >= 2) {  // At least board_id + msg_index
        uint8_t board_id_from_payload = msg[0];
        uint8_t msg_index = msg[1];
        if (board_id_from_payload == id) {
          this->process_cell_temperatures(msg, board_id_from_payload, msg_index, dlc);
        }
      } 
      break;
    }
    default:
      break;
  }
}

void RosCan::bamocar_current_publisher(const unsigned char msg[8]) {
  std_msgs::msg::Int32 current_msg;
  int16_t current = static_cast<int16_t>((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]);
  current_msg.data = current;
  _bamocar_current_pub->publish(current_msg);
}

void RosCan::can_interpreter_bamocar(const unsigned char msg[8]) {
  switch (msg[0]) {
    case BAMOCAR_BATTERY_VOLTAGE_CODE: {
      inverter_voltage_publisher(msg);
      break;
    }
    case BAMOCAR_MOTOR_SPEED_CODE: {
      motor_speed_publisher(msg);
      break;
    }
    case BAMOCAR_MOTOR_TEMP_CODE: {
      motor_temp_publisher(msg);
      break;
    }
    case BAMOCAR_INVERTER_TEMP_CODE: {
      inverter_temp_publisher(msg);
      break;
    }
    case BAMO_CURRENT_ID: {
      bamocar_current_publisher(msg);
      break;
    }
    case BAMO_ERRORS_ID: {

      break;
    }
    default:
      break;
  }
}

void RosCan::can_interpreter_master(const unsigned char msg[8]) {
  RCLCPP_DEBUG(this->get_logger(), "Received Master message");
  RCLCPP_DEBUG(this->get_logger(), "Message: %x %x %x %x %x %x %x %x", msg[0], msg[1], msg[2],
              msg[3], msg[4], msg[5], msg[6], msg[7]);
  switch (msg[0]) {
    case MASTER_AS_STATE_CODE: {
      if (msg[1] == 3) {  // If AS State == Driving
        this->go_signal_ = 1;
      } else if (msg[1] == 2){
        cubem_set_origin(); // If AS State == Ready
      }
      
      else {
        this->go_signal_ = 0;
      }
      this->as_status_ = static_cast<uint8_t>(msg[1]);
      if (this->as_status_ == 4) {
        this->as_status_ = 5;
      } else if (this->as_status_ == 5) {
        this->as_status_ = 4;
      }
      op_status_publisher();
      break;
    }
    case MASTER_AS_MISSION_CODE: {
      this->as_mission_ = msg[1];

      switch(this->as_mission_){
        case 0: 
          this->ami_state_ = 0;     // Manual
          break;
        case 1:
          this->ami_state_ = 1;     // Acceleration 
          break;
        case 2:
          this->ami_state_ = 2;     // Skidpad 
          break;
        case 3:
          this->ami_state_ = 6;     // Autocross
          break;
        case 4:
          this->ami_state_ = 3;     // Trackdrive
          break;
        case 5:
          this->ami_state_ = 4;    // EBS test
          break;
        case 6:
          this->ami_state_ = 5;    // Inspection
          break;
        default:
          break;
      }

      op_status_publisher();
      break;
    }
    case MASTER_EBS_STATE_CODE: {
      this->asb_ebs_state_ = static_cast<uint8_t>(msg[1]);
      break;
    }
    case MASTER_EBS_REDUNDANCY_STATE_CODE: {
      this->asb_redundancy_state_ = static_cast<uint8_t>(msg[1]);
      break;
    }
    default:
      break;  // add error message
  }
}

void RosCan::dash_interpreter(const unsigned char msg[8]) {
  switch (msg[0]) {
    case HYDRAULIC_LINE: {
      hydraulic_line_callback(msg);
      break;
    }
    case FR_RPM_CODE: {
      fr_rpm_publisher(msg);
      break;
    }
    case FL_RPM_CODE: {
      fl_rpm_publisher(msg);
      break;
    }
    case APPS_HIGHER: {
      apps_higher_publisher(msg);
      break;
    }
    case APPS_LOWER: {
      apps_lower_publisher(msg);
      break;
    }
    case DRIVING_STATE: {
      driving_state_publisher(msg);
      break;
    }
    default:
      break;  // add error message
  }
}

void RosCan::data_log_info_1_publisher(const unsigned char msg[8]) {
  bool placeholder = (msg[0] >> 7) & 0x01;
  bool asms_on = (msg[0] >> 6) & 0x01;
  bool asats_pressed = (msg[0] >> 5) & 0x01;
  bool ats_pressed = (msg[0] >> 4) & 0x01;
  bool tsms_sdc_closed = (msg[0] >> 3) & 0x01;
  bool master_sdc_closed = (msg[0] >> 2) & 0x01;
  bool ts_on = (msg[0] >> 1) & 0x01;
  bool wd_ready = msg[0] & 0x01;
  
  bool emergency_signal = (msg[1] >> 7) & 0x01;
  bool bms_dead = (msg[1] >> 6) & 0x01;
  bool engage_ebs_check = (msg[1] >> 5) & 0x01;
  bool release_ebs_check = (msg[1] >> 4) & 0x01;
  bool steer_dead = (msg[1] >> 3) & 0x01;
  bool pc_dead = (msg[1] >> 2) & 0x01;
  bool inversor_dead = (msg[1] >> 1) & 0x01;
  bool res_dead = msg[1] & 0x01;
  
  uint8_t state_checkup = msg[2] & 0x0F;
  bool pneumatic_line_pressure_1 = (msg[2] >> 7) & 0x01;
  bool pneumatic_line_pressure_2 = (msg[2] >> 6) & 0x01;
  bool pneumatic_line_pressure_main = (msg[2] >> 5) & 0x01;  
  uint32_t dc_voltage = (static_cast<uint32_t>(msg[3]) << 24) |
                        (static_cast<uint32_t>(msg[4]) << 16) |
                        (static_cast<uint32_t>(msg[5]) << 8) |
                        static_cast<uint32_t>(msg[6]);
  
  uint8_t mission = msg[7] & 0x0F;
  uint8_t state = (msg[7] >> 4) & 0x0F;

  custom_interfaces::msg::DataLogInfo1 data_log_info_1;
  data_log_info_1.header.stamp = this->get_clock()->now();

  data_log_info_1.pneumatic_line_pressure =
      pneumatic_line_pressure_1 && pneumatic_line_pressure_2 && pneumatic_line_pressure_main;
  data_log_info_1.asms_on = asms_on;
  data_log_info_1.asats_pressed = asats_pressed;
  data_log_info_1.ats_pressed = ats_pressed;
  data_log_info_1.tsms_sdc_closed = tsms_sdc_closed;
  data_log_info_1.master_sdc_closed = master_sdc_closed;
  data_log_info_1.ts_on = ts_on;
  data_log_info_1.wd_ready = wd_ready;
  data_log_info_1.emergency_signal = emergency_signal;
  data_log_info_1.bms_dead = bms_dead;
  data_log_info_1.engage_ebs_check = engage_ebs_check;
  data_log_info_1.release_ebs_check = release_ebs_check;
  data_log_info_1.steer_dead = steer_dead;
  data_log_info_1.pc_dead = pc_dead;
  data_log_info_1.inversor_dead = inversor_dead;
  data_log_info_1.res_dead = res_dead;
  data_log_info_1.state_checkup = state_checkup;
  data_log_info_1.dc_voltage = dc_voltage;
  data_log_info_1.mission = mission;
  data_log_info_1.state = state;
  data_log_info_1.pneumatic_line_pressure_1 = pneumatic_line_pressure_1;
  data_log_info_1.pneumatic_line_pressure_2 = pneumatic_line_pressure_2;
  data_log_info_1.pneumatic_line_pressure_main = pneumatic_line_pressure_main;
  data_log_info_1.placeholder = placeholder;

  data_log_info_1_pub_->publish(data_log_info_1);
}


void RosCan::data_log_info_2_publisher(const unsigned char msg[8]) {
  uint32_t hydraulic_line_front_pressure = (static_cast<uint32_t>(msg[0]) << 24) |
                                           (static_cast<uint32_t>(msg[1]) << 16) |
                                           (static_cast<uint32_t>(msg[2]) << 8) |
                                           static_cast<uint32_t>(msg[3]);
  
  uint32_t hydraulic_line_pressure = (static_cast<uint32_t>(msg[4]) << 24) |
                                     (static_cast<uint32_t>(msg[5]) << 16) |
                                     (static_cast<uint32_t>(msg[6]) << 8) |
                                     static_cast<uint32_t>(msg[7]);

  custom_interfaces::msg::DataLogInfo2 data_log_info_2;
  data_log_info_2.header.stamp = this->get_clock()->now();

  data_log_info_2.hydraulic_line_front_pressure = hydraulic_line_front_pressure;
  data_log_info_2.hydraulic_line_pressure = hydraulic_line_pressure;

}

void RosCan::op_status_publisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.header.stamp = this->get_clock()->now();
  message.go_signal = this->go_signal_;
  message.as_mission = this->as_mission_;
  RCLCPP_DEBUG(this->get_logger(),
               "Received Operational Status Message: Go Signal: %d --- AS Mission: %d",
               go_signal_, as_mission_);
  operational_status_->publish(message);
}

void RosCan::imu_acc_publisher(const unsigned char msg[8]) {
  if ((msg[6] & 0b11110000) != 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid Signal");
    return;
  }

  float acc_x = ((msg[0] << 8 | msg[1]) - 0X8000) * QUANTIZATION_ACC;
  float acc_y = ((msg[2] << 8 | msg[3]) - 0X8000) * QUANTIZATION_ACC;
  float acc_z = ((msg[4] << 8 | msg[5]) - 0X8000) * QUANTIZATION_ACC;

  auto message = custom_interfaces::msg::ImuAcceleration();
  message.header.stamp = this->get_clock()->now();
  message.acc_x = acc_x;
  message.acc_y = acc_y;
  message.acc_z = acc_z;

  imu_acc_pub_->publish(message);
}

void RosCan::imu_angular_velocity_publisher(const unsigned char msg[8]) {
  // if ((msg[6] & 0b11110000) != 0){
  //   RCLCPP_WARN(this->get_logger(),
  //     "Invalid Signal");
  //   return;
  // }

  // if (!calculateCRC8_SAE_J1850(msg)) {
  //   RCLCPP_WARN(this->get_logger(),
  //               "Invalid CRC8 received from IMU Angular Velocity; dumping message...");
  //   return;
  // }

  float roll = ((msg[0] << 8 | msg[1]) - 0x8000) * QUANTIZATION_GYRO;
  float pitch = ((msg[2] << 8 | msg[3]) - 0x8000) * QUANTIZATION_GYRO;
  float yaw = ((msg[4] << 8 | msg[5]) - 0x8000) * QUANTIZATION_GYRO;


  auto message = custom_interfaces::msg::YawPitchRoll();
  message.header.stamp = this->get_clock()->now();
  message.roll = roll;
  message.pitch = pitch;
  message.yaw = yaw;

  imu_angular_velocity_pub_->publish(message);
}

void RosCan::bms_cells_temps_callback(const unsigned char msg[8]) {
  RCLCPP_DEBUG(this->get_logger(), "Registering BMS temps");
  this->_cells_temps_min_ = msg[1];
  this->_cells_temps_max_ = msg[2];
  this->_cells_temps_avg_ = msg[3];
}

// Used only to initially set actuator origin
void RosCan::steering_angle_cubem_publisher(const unsigned char msg[8]) {
  // When steering motor wakes up, set its origin
  if (!this->cubem_configuration_sent_) {
    this->cubem_configuration_sent_ = true;
    // bosch_steering_angle_set_origin();
    this->cubem_set_origin();
    RCLCPP_INFO(this->get_logger(), "New configuration sent!");
  }

  int16_t angle = (msg[0] << 8) | msg[1];    // Extract 16-bit motor angle
  int16_t speed = (msg[2] << 8) | msg[3];    // Extract 16-bit motor speed
  int16_t current = (msg[4] << 8) | msg[5];  // Extract 16-bit motor current
  int8_t temperature = msg[6];               // Extract 8-bit motor temperature
  int8_t error = msg[7];                     // Extract 8-bit motor error

  auto motor_message = custom_interfaces::msg::SteeringAngle();
  motor_message.header.stamp = this->get_clock()->now();
  motor_message.steering_angle = static_cast<double>(angle) * 0.1;   // Convert to degrees
  motor_message.steering_speed = static_cast<double>(speed) / 10.0;  // Convert to RPM

  auto current_msg = std_msgs::msg::Float64();
  auto temperature_msg = std_msgs::msg::Int8();
  auto error_msg = std_msgs::msg::Int8();
  current_msg.data = static_cast<double>(current) * 0.01;   // Convert to Amperes
  temperature_msg.data = static_cast<int8_t>(temperature);  // Convert to int8
  error_msg.data = static_cast<int8_t>(error);              // Convert to int8

  steering_motor_state_pub_->publish(motor_message);
  steering_motor_current->publish(current_msg);
  steering_motor_temperature->publish(temperature_msg);
  steering_motor_error->publish(error_msg);
}

void RosCan::steering_angle_bosch_publisher(const unsigned char msg[8]) {
  // Mask to check the first 3 bits
  char mask = 0b00100000;
  // Extract bits 7,6,5
  char first_three_bits = msg[6] & mask;

  // Error flags
  if (first_three_bits != mask) {
    RCLCPP_WARN(this->get_logger(), "Invalid message received from Bosch Steering Wheel Sensor: %x",
                first_three_bits);
    return;
  }

  if (!checkCRC8(msg)) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid CRC8 received from Bosch SA Sensor; dumping message...");
    return;
  }

  // Calculate angle
  bool negative = msg[2] & 0b00000001;
  short angle_value = msg[1] << 7 | (msg[2] >> 1);
  float angle = negative ? -(angle_value / 10.0) : angle_value / 10.0;

  // Calculate turning speed
  negative = msg[4] & 0b00000001;
  short speed_value = msg[3] << 7 | (msg[4] >> 1);
  float speed = negative ? -(speed_value / 10.0) : speed_value / 10.0;

  speed = speed * M_PI / 180;

  angle = angle * M_PI / 180;
  this->steering_angle_ = -angle;

  double steering_degrees = this->steering_angle_ * 180.0 / M_PI;
  steering_angle_actual_ = static_cast<int8_t>(steering_degrees * 2);

  RCLCPP_DEBUG(this->get_logger(), "Steering angle: %f", this->steering_angle_);

  double steering_angle_wheels;
  transform_steering_angle_reading(this->steering_angle_, steering_angle_wheels);

  // Send message
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = steering_angle_wheels;
  message.steering_speed = speed;
  // RCLCPP_DEBUG(this->get_logger(), "Received Bosch Steering Angle (radians): %f",
  // this->steering_angle_);
  bosch_steering_angle_publisher_->publish(message);
}

void RosCan::rr_rpm_publisher(const unsigned char msg[8]) {
  float rrRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rr_rpm = rrRPM;
  // RCLCPP_DEBUG(this->get_logger(), "Received RR RPM: %f", rrRPM);
  rr_rpm_pub_->publish(message);
}

void RosCan::rl_rpm_publisher(const unsigned char msg[8]) {
  float rlRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rl_rpm = rlRPM;
  // RCLCPP_DEBUG(this->get_logger(), "Received RL RPM: %f", rlRPM);
  rl_rpm_pub_->publish(message);
}

void RosCan::fr_rpm_publisher(const unsigned char msg[8]) {
  float frRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.fr_rpm = frRPM;
  // RCLCPP_DEBUG(this->get_logger(), "Received FR RPM: %f", frRPM);
  fr_rpm_pub_->publish(message);
}

void RosCan::fl_rpm_publisher(const unsigned char msg[8]) {
  float flRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.fl_rpm = flRPM;
  // RCLCPP_DEBUG(this->get_logger(), "Received FL RPM: %f", flRPM);
  fl_rpm_pub_->publish(message);
}

void RosCan::inverter_voltage_publisher(const unsigned char msg[8]) {
  this->inverter_voltage_ = (msg[2] << 8) | msg[1];
  auto message = std_msgs::msg::Int32();
  message.data = this->inverter_voltage_ / 31.58483;
  RCLCPP_DEBUG(this->get_logger(), "Received voltage from Bamocar: %d", this->inverter_voltage_);
  inverter_voltage_pub_->publish(message);
}

void RosCan::motor_speed_publisher(const unsigned char msg[8]) {
  short int temp_speed = (msg[2] << 8) | msg[1];
  this->motor_speed_ = static_cast<int>(temp_speed);
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rl_rpm = this->motor_speed_ * BAMOCAR_MAX_RPM / BAMOCAR_MAX_SCALE;
  message.rr_rpm = this->motor_speed_ * BAMOCAR_MAX_RPM / BAMOCAR_MAX_SCALE;
  RCLCPP_DEBUG(this->get_logger(), "Received motor speed from Bamocar: %d", this->motor_speed_);
  motor_rpm_pub_->publish(message);
}
void RosCan::motor_temp_publisher(const unsigned char msg[8]) {
  uint16_t motor_temp_adc_ = (msg[2] << 8) | msg[1];
  this->motor_temp_ = MotorTemperatureConverter().adc_to_temperature(motor_temp_adc_);
  auto motor_temp_msg = custom_interfaces::msg::Temperature();
  motor_temp_msg.header.stamp = this->get_clock()->now();
  motor_temp_msg.temperature = this->motor_temp_;
  motor_temp_pub_->publish(motor_temp_msg);
}
void RosCan::inverter_temp_publisher(const unsigned char msg[8]) {
  uint16_t inverter_temp_adc_ = (msg[2] << 8) | msg[1];
  this->inverter_temp_ = InverterTemperatureConverter().adc_to_temperature(inverter_temp_adc_);
  auto inverter_temp_msg = custom_interfaces::msg::Temperature();
  inverter_temp_msg.header.stamp = this->get_clock()->now();
  inverter_temp_msg.temperature = this->inverter_temp_;
  inverter_temp_pub_->publish(inverter_temp_msg);
}

void RosCan::manual_throttle_publisher(const unsigned char msg[8]) {
  int throttle_command = (static_cast<int>(msg[2]) << 8) |
                         static_cast<int>(msg[1]);

  // Sign extension for 16-bit signed value (if needed)
  if (throttle_command & 0x8000) {
    throttle_command |= 0xFFFF0000;
  }
  double throttle_value_ros = static_cast<double>(throttle_command) / BAMOCAR_MAX_SCALE;
  
  std_msgs::msg::Float64 throttle_msg;
  throttle_msg.data = static_cast<int>(throttle_value_ros);

  manual_throttle_pub_->publish(throttle_msg);
}


void RosCan::hydraulic_line_callback(const unsigned char msg[8]) {
  int hydraulic_line_pressure = (msg[2] << 8) | msg[1];

  auto message = custom_interfaces::msg::HydraulicLinePressure();
  message.header.stamp = this->get_clock()->now();
  message.pressure = hydraulic_line_pressure;

  auto brake_percentage = static_cast<float>(hydraulic_line_pressure - 150) / (341 - 150) * 100.0f; // convert to percentage

  this->brake_hydr_actual_ = static_cast<uint8_t>(brake_percentage);
  this->brake_hydr_target_ = static_cast<uint8_t>(brake_percentage);

  auto brake_message = std_msgs::msg::Float64();
  brake_message.data = brake_percentage;

  manual_brake_pub_->publish(brake_message);

  hydraulic_line_pressure_publisher_->publish(message);
}

void RosCan::bms_errors_publisher(const unsigned char msg[8], unsigned int dlc) {
  auto bms_errors_msg = custom_interfaces::msg::BmsErrors();
  bms_errors_msg.header.stamp = this->get_clock()->now();

  bms_errors_msg.bms_current = static_cast<int16_t>((msg[6] << 8) | msg[7]) / 10.0;

  // if (dlc >= 2) {
  //   uint16_t error_bitmap_1 = (msg[1] << 8) | msg[0];
  //   bms_errors_msg.error_bitmap_1 = error_bitmap_1;

  //   bms_errors_msg.discharge_limit_enforcement_fault = (error_bitmap_1 & (1 << 0)) != 0;
  //   bms_errors_msg.charger_safety_relay_fault = (error_bitmap_1 & (1 << 1)) != 0;
  //   bms_errors_msg.internal_hardware_fault = (error_bitmap_1 & (1 << 2)) != 0;
  //   bms_errors_msg.internal_heatsink_thermistor_fault = (error_bitmap_1 & (1 << 3)) != 0;
  //   bms_errors_msg.internal_software_fault = (error_bitmap_1 & (1 << 4)) != 0;
  //   bms_errors_msg.highest_cell_voltage_too_high_fault = (error_bitmap_1 & (1 << 5)) != 0;
  //   bms_errors_msg.lowest_cell_voltage_too_low_fault = (error_bitmap_1 & (1 << 6)) != 0;
  //   bms_errors_msg.pack_too_hot_fault = (error_bitmap_1 & (1 << 7)) != 0;
  // }

  // if (dlc >= 4) {
  //   uint16_t error_bitmap_2 = (msg[3] << 8) | msg[2];
  //   bms_errors_msg.error_bitmap_2 = error_bitmap_2;

  //   bms_errors_msg.internal_communication_fault = (error_bitmap_2 & (1 << 0)) != 0;
  //   bms_errors_msg.cell_balancing_stuck_off_fault = (error_bitmap_2 & (1 << 1)) != 0;
  //   bms_errors_msg.weak_cell_fault = (error_bitmap_2 & (1 << 2)) != 0;
  //   bms_errors_msg.low_cell_voltage_fault = (error_bitmap_2 & (1 << 3)) != 0;
  //   bms_errors_msg.open_wiring_fault = (error_bitmap_2 & (1 << 4)) != 0;
  //   bms_errors_msg.current_sensor_fault = (error_bitmap_2 & (1 << 5)) != 0;
  //   bms_errors_msg.highest_cell_voltage_over_5v_fault = (error_bitmap_2 & (1 << 6)) != 0;
  //   bms_errors_msg.cell_asic_fault = (error_bitmap_2 & (1 << 7)) != 0;
  //   bms_errors_msg.weak_pack_fault = (error_bitmap_2 & (1 << 8)) != 0;
  //   bms_errors_msg.fan_monitor_fault = (error_bitmap_2 & (1 << 9)) != 0;
  //   bms_errors_msg.thermistor_fault = (error_bitmap_2 & (1 << 10)) != 0;
  //   bms_errors_msg.external_communication_fault = (error_bitmap_2 & (1 << 11)) != 0;
  //   bms_errors_msg.redundant_power_supply_fault = (error_bitmap_2 & (1 << 12)) != 0;
  //   bms_errors_msg.high_voltage_isolation_fault = (error_bitmap_2 & (1 << 13)) != 0;
  //   bms_errors_msg.input_power_supply_fault = (error_bitmap_2 & (1 << 14)) != 0;
  //   bms_errors_msg.charge_limit_enforcement_fault = (error_bitmap_2 & (1 << 15)) != 0;
  // }
  bms_errors_pub_->publish(bms_errors_msg);
}

void RosCan::apps_higher_publisher(const unsigned char msg[8]) {
  apps_higher_value = (msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1];
  auto message = std_msgs::msg::Int32();
  message.data = apps_higher_value;

  apps_higher_pub_->publish(message);
}

void RosCan::apps_lower_publisher(const unsigned char msg[8]) {
  apps_lower_value = (msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1];
  auto message = std_msgs::msg::Int32();
  message.data = apps_lower_value;

  auto dif_message = std_msgs::msg::Float64();
  dif_message.data = (apps_higher_value - apps_lower_value - 140) / 480.0;
  apps_error_pub_->publish(dif_message);

  apps_lower_pub_->publish(message);
}

void RosCan::driving_state_publisher(const unsigned char msg[8]) {
  int8_t driving_state = msg[1];
  bool implausibility = (msg[2] != 0);
  auto driving_state_msg = std_msgs::msg::Int8();
  driving_state_msg.data = driving_state;
  driving_state_pub_->publish(driving_state_msg);

  auto implausability_msg = std_msgs::msg::Bool();
  implausability_msg.data = implausibility;
  implausability_pub_->publish(implausability_msg);
}

void RosCan::dv_messages_callback() {
    send_dv_driving_dynamics_1();
    send_dv_driving_dynamics_2();
    send_dv_system_status();
}

void RosCan::send_dv_driving_dynamics_1() {
    long id = DV_DRIVING_DYNAMICS_1_ID;
    unsigned char buffer[8] = {0};
    
    // Speed_actual (bit 0-7, unsigned)
    buffer[0] = speed_actual_;

    // Speed_target (bit 8-15, unsigned)
    buffer[1] = speed_target_;
    
    // Steering_angle_actual (bit 16-23, signed, scale 0.5)
    buffer[2] = static_cast<uint8_t>(steering_angle_actual_);
    
    // Steering_angle_target (bit 24-31, signed, scale 0.5)
    buffer[3] = static_cast<uint8_t>(steering_angle_target_);
    
    // Brake_hydr_actual (bit 32-39, unsigned)
    buffer[4] = brake_hydr_actual_;

    // Brake_hydr_target (bit 40-47, unsigned)
    buffer[5] = brake_hydr_target_;
    
    // Motor_moment_actual (bit 48-55, signed)
    buffer[6] = static_cast<uint8_t>(motor_moment_actual_);
    
    // Motor_moment_target (bit 56-63, signed)
    buffer[7] = static_cast<uint8_t>(motor_moment_target_);
    
    void *requestData = static_cast<void *>(buffer);
    unsigned int dlc = 8;
    unsigned int flag = 0;
    
    stat_ = can_lib_wrapper_->canWrite(hnd1_, id, requestData, dlc, flag);
    if (stat_ != canOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write DV driving dynamics 1 to CAN bus");
    }
}

void RosCan::send_dv_driving_dynamics_2() {
    long id = DV_DRIVING_DYNAMICS_2_ID;
    unsigned char buffer[8] = {0};
    
    // Acceleration longitudinal (bit 0-15, signed, scale 1/512)
    buffer[0] = (acceleration_longitudinal_ * 512) & 0xFF;
    buffer[1] = ((acceleration_longitudinal_ * 512) >> 8) & 0xFF;
    
    // Acceleration lateral (bit 16-31, signed, scale 1/512)
    buffer[2] = (acceleration_lateral_ * 512) & 0xFF;
    buffer[3] = ((acceleration_lateral_ * 512) >> 8) & 0xFF;
    
    // Yaw rate (bit 32-47, signed, scale 1/128)
    buffer[4] = (yaw_rate_ * 128) & 0xFF;
    buffer[5] = ((yaw_rate_ * 128) >> 8) & 0xFF;
    
    buffer[6] = 0;
    buffer[7] = 0;
    
    void *requestData = static_cast<void *>(buffer);
    unsigned int dlc = 6;
    unsigned int flag = 0;
    
    stat_ = can_lib_wrapper_->canWrite(hnd1_, id, requestData, dlc, flag);
    if (stat_ != canOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write DV driving dynamics 2 to CAN bus");
    }
}

void RosCan::send_dv_system_status() {
    long id = DV_SYSTEM_STATUS;
    unsigned char buffer[8] = {0};
    
    // AS_status (bit 0-2)
    buffer[0] = as_status_ & 0x07;
    
    // ASB_EBS_state (bit 3-4)
    buffer[0] |= (asb_ebs_state_ & 0x03) << 3;
    
    // AMI_state (bit 5-7)
    buffer[0] |= (ami_state_ & 0x07) << 5;
    
    // Steering_state (bit 8, bool)
    buffer[1] = steering_state_ ? 1 : 0;
    
    // ASB_redundancy_state (bit 9-10)
    buffer[1] |= (asb_redundancy_state_ & 0x03) << 1;
    
    // Lap_counter (bit 11-14, unsigned)
    buffer[1] |= (lap_counter_ & 0x0F) << 3;
    
    // Cones_count_actual (bit 15-22, unsigned)
    buffer[1] |= (cones_count_actual_ & 0x01) << 7; // bit 15
    buffer[2] = (cones_count_actual_ >> 1) & 0x7F;  // bits 16-22
    
    // Cones_count_all (bit 23-39, unsigned)
    buffer[2] |= (cones_count_all_ & 0x01) << 7;    // bit 23
    buffer[3] = (cones_count_all_ >> 1) & 0xFF;     // bits 24-31
    buffer[4] = (cones_count_all_ >> 9) & 0xFF;     // bits 32-39
    

    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    
    void *requestData = static_cast<void *>(buffer);
    unsigned int dlc = 5; 
    unsigned int flag = 0;
    
    stat_ = can_lib_wrapper_->canWrite(hnd1_, id, requestData, dlc, flag);
    if (stat_ != canOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write DV system status to CAN bus");
    }
}

void RosCan::process_cell_temperatures(const unsigned char msg[8], uint8_t stack_id, uint8_t msg_index, unsigned int dlc) {

  // Process temperature data starting from buf[2]
  uint8_t temp_count = dlc - 2;  
  uint8_t start_sensor_index = msg_index * 6;

  for (uint8_t i = 0; i < temp_count; i++) {
    uint8_t sensor_index = start_sensor_index + i;
    if (sensor_index < NTC_SENSOR_COUNT) {  
      _cells_temps_[stack_id * NTC_SENSOR_COUNT + sensor_index] =
          static_cast<int8_t>(msg[2 + i]); 
    }
  }
}

void RosCan::publish_cell_temperatures() {
  custom_interfaces::msg::CellsTemps cell_temps_msg;
  cell_temps_msg.header.stamp = this->get_clock()->now();
  
  for (uint8_t i = 0; i < NTC_SENSOR_COUNT * 6; i++) {
    cell_temps_msg.all_cells[i] = _cells_temps_[i];
  }

  cell_temps_msg.min_temp = this->_cells_temps_min_;
  cell_temps_msg.max_temp = this->_cells_temps_max_;
  cell_temps_msg.avg_temp = this->_cells_temps_avg_;
  
  cells_temps_pub_->publish(cell_temps_msg);
  RCLCPP_DEBUG(this->get_logger(), "Published cell temperatures: Min: %d, Max: %d, Avg: %d",
               this->_cells_temps_min_, this->_cells_temps_max_, this->_cells_temps_avg_);
}

void RosCan::publish_bamocar_errors(const unsigned char msg[8]) {
  custom_interfaces::msg::BamocarErrors bamocar_errors_msg;
  bamocar_errors_msg.header.stamp = this->get_clock()->now();
  int32_t message_value = (msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1];
  const uint16_t error_bitmap = static_cast<uint16_t>(message_value & 0xFFFF);
  const uint16_t warning_bitmap = static_cast<uint16_t>((message_value >> 16) & 0xFFFF);

  bamocar_errors_msg.bad_params = (warning_bitmap & 1);
  bamocar_errors_msg.hardware_error = (warning_bitmap >> 1 & 1);
  bamocar_errors_msg.rfe_not_present = (warning_bitmap >> 2 & 1);
  bamocar_errors_msg.bus_timeout= (warning_bitmap >> 3 & 1);
  bamocar_errors_msg.bad_missing_feedback = (warning_bitmap >> 4 & 1);
  bamocar_errors_msg.voltage_low = (warning_bitmap >> 5 & 1);
  bamocar_errors_msg.motor_temp = (warning_bitmap >> 6 & 1);
  bamocar_errors_msg.igbt_temp = (warning_bitmap >> 7 & 1);
  bamocar_errors_msg.voltage_high = (warning_bitmap >> 8 & 1);
  bamocar_errors_msg.i_peak = (warning_bitmap >> 9 & 1);
  bamocar_errors_msg.race_away = (warning_bitmap >> 10 & 1);
  bamocar_errors_msg.ecode_error = (warning_bitmap >> 11 & 1);
  bamocar_errors_msg.watchdog_reset= (warning_bitmap >> 12 & 1);
  bamocar_errors_msg.i_offset = (warning_bitmap >> 13 & 1);
  bamocar_errors_msg.internal_voltage = (warning_bitmap >> 14 & 1);
  bamocar_errors_msg.bleed_resistor_overload = (warning_bitmap >> 15 & 1);

  bamocar_errors_msg.bad_params = (error_bitmap & 1) * 2;
  bamocar_errors_msg.hardware_error = (error_bitmap >> 1 & 1) * 2;
  bamocar_errors_msg.rfe_not_present = (error_bitmap >> 2 & 1) * 2;
  bamocar_errors_msg.bus_timeout= (error_bitmap >> 3 & 1) * 2;
  bamocar_errors_msg.bad_missing_feedback = (error_bitmap >> 4 & 1) * 2;
  bamocar_errors_msg.voltage_low = (error_bitmap >> 5 & 1) * 2;
  bamocar_errors_msg.motor_temp = (error_bitmap >> 6 & 1) * 2;
  bamocar_errors_msg.igbt_temp = (error_bitmap >> 7 & 1) * 2;
  bamocar_errors_msg.voltage_high = (error_bitmap >> 8 & 1) * 2;
  bamocar_errors_msg.i_peak = (error_bitmap >> 9 & 1) * 2;
  bamocar_errors_msg.race_away = (error_bitmap >> 10 & 1) * 2;
  bamocar_errors_msg.ecode_error = (error_bitmap >> 11 & 1) * 2;
  bamocar_errors_msg.watchdog_reset= (error_bitmap >> 12 & 1) * 2;
  bamocar_errors_msg.i_offset = (error_bitmap >> 13 & 1) * 2;
  bamocar_errors_msg.internal_voltage = (error_bitmap >> 14 & 1) * 2;
  bamocar_errors_msg.bleed_resistor_overload = (error_bitmap >> 15 & 1) * 2;
  RCLCPP_DEBUG(this->get_logger(), "Publishing Bamocar Errors");

  _inverter_errors_pub_->publish(bamocar_errors_msg);
}


RosCan::~RosCan() {
  RCLCPP_INFO(this->get_logger(), "Shutting down CAN interface");

  if (hnd0_ >= 0) {
    canBusOff(hnd0_);
    canClose(hnd0_);
    hnd0_ = -1;
  }

  if (hnd1_ >= 0) {
    canBusOff(hnd1_);
    canClose(hnd1_);
    hnd1_ = -1;
  }
}
