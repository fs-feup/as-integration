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
  master_log_pub_ =
      this->create_publisher<custom_interfaces::msg::MasterLog>("/vehicle/master_log", 10);
  master_log_pub_2_ =
      this->create_publisher<custom_interfaces::msg::MasterLog2>("/vehicle/master_log_2", 10);
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

  steering_motor_state_pub_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>(

        "/vehicle/steering_motor_state", 10);
  steering_motor_temperature = this->create_publisher<std_msgs::msg::Int8>(
        "/vehicle/steering_motor_temperature", 10);
  steering_motor_current = this->create_publisher<std_msgs::msg::Float64>(
        "/vehicle/steering_motor_current", 10);
  steering_motor_error = this->create_publisher<std_msgs::msg::Int8>(

  hydraulic_line_pressure_publisher_ =
      this->create_publisher<custom_interfaces::msg::HydraulicLinePressure>(
          "/vehicle/hydraulic_line_pressure", 10);

  // Subscritpions
  control_listener_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));

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

  // initialize the CAN library
  canInitializeLibrary();
  // A channel to a CAN circuit is opened. The channel depend on the hardware

  hnd0_ = canOpenChannel(0, canOPEN_EXCLUSIVE);  // TODO: this will be used only for SAS
  hnd1_ = canOpenChannel(1, canOPEN_EXCLUSIVE);  // TODO: this is for everything else
  // Setup CAN parameters for the channel
  stat_ = canSetBusParams(hnd0_, canBITRATE_1M, 0, 0, 4, 0, 0);    // check these values later
  stat_ = canSetBusParams(hnd1_, canBITRATE_500K, 0, 0, 4, 0, 0);  // check these values later

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
  stat_ = can_lib_wrapper_->canWrite(hnd_, id, steering_requestData, steering_dlc, flag);
  // CODE BLOCK END
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
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
        throttle_command, max_torque_dynamic_limits(this->battery_voltage_, this->motor_speed_));
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
  stat_ = can_lib_wrapper_->canWrite(hnd0_, throttle_id, throttle_requestData, throttle_dlc, flag);
  // CODE BLOCK END
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus");
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
  RCLCPP_INFO(this->get_logger(), "Emergency signal received, sending to CAN");

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

  stat_ = can_lib_wrapper_->canWrite(hnd0_, id, requestData, dlc, flag);
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

  stat_ = can_lib_wrapper_->canWrite(hnd0_, id, msg, dlc, flag);
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
  stat_ = can_lib_wrapper_->canWrite(hnd0_, id, steering_requestData, steering_dlc, flag);
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
  stat_ = canWrite(hnd1_, id, request_data, dlc, flag);
  if (stat_ != canOK) {
    return 1;
  }
  sleep(1);  // Necessary for configuration to sink in

  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_SET;
  request_data = static_cast<void *>(buffer);

  // Set new origin
  stat_ = canWrite(hnd1_, id, request_data, dlc, flag);
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

void RosCan::can_interpreter(long id, const unsigned char msg[8], unsigned int, unsigned int,
                             unsigned long) {
  switch (id) {
    case MASTER_ID: {
      can_interpreter_master(msg);
      break;
    }

    // Accelearation from IMU
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
    default:
      break;
  }
}

void RosCan::can_interpreter_bamocar(const unsigned char msg[8]) {
  switch (msg[0]) {
    case BAMOCAR_BATTERY_VOLTAGE_CODE: {
      battery_voltage_callback(msg);
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
    default:
      break;
  }
}

void RosCan::can_interpreter_master(const unsigned char msg[8]) {
  switch (msg[0]) {
    case MASTER_AS_STATE_CODE: {
      if (msg[1] == 3) {  // If AS State == Driving
        this->go_signal_ = 1;
      } else {
        this->go_signal_ = 0;
      }
      op_status_publisher();
      break;
    }
    case MASTER_AS_MISSION_CODE: {
      this->as_mission_ = msg[1];
      op_status_publisher();
      break;
    }
    case MASTER_DBG_LOG_MSG: {
      this->master_logs_publisher(msg);
      break;
    }
    case MASTER_DBG_LOG_MSG_2: {
      this->master_logs_2_publisher(msg);
      break;
    }
    case TEENSY_RR_RPM_CODE: {
      rr_rpm_publisher(msg);
      break;
    }
    case TEENSY_RL_RPM_CODE: {
      rl_rpm_publisher(msg);
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
    default:
      break;  // add error message
  }
}

void RosCan::master_logs_publisher(const unsigned char msg[8]) {
  uint32_t hydraulic_pressure = (msg[1] << 24) | (msg[2] << 16) | (msg[3] << 8) | msg[4];
  bool emergency_signal = (msg[5] >> 7) & 0x01;
  bool pneumatic_line_pressure = (msg[5] >> 6) & 0x01;
  bool engage_ebs_check = (msg[5] >> 5) & 0x01;
  bool release_ebs_check = (msg[5] >> 4) & 0x01;
  bool steer_dead = (msg[5] >> 3) & 0x01;
  bool pc_dead = (msg[5] >> 2) & 0x01;
  bool inversor_dead = (msg[5] >> 1) & 0x01;
  bool res_dead = msg[5] & 0x01;
  bool asms_on = (msg[6] >> 7) & 0x01;
  bool ts_on = (msg[6] >> 6) & 0x01;
  bool sdc_open = (msg[6] >> 5) & 0x01;
  uint8_t checkup_state = (msg[6]) & 0x0F;
  uint8_t mission = msg[7] & 0x0F;
  uint8_t master_state = (msg[7] >> 4) & 0x0F;

  custom_interfaces::msg::MasterLog log_message;
  log_message.hydraulic_pressure = hydraulic_pressure;
  log_message.emergency_signal = emergency_signal;
  log_message.pneumatic_line_pressure = pneumatic_line_pressure;
  log_message.engage_ebs_check = engage_ebs_check;
  log_message.release_ebs_check = release_ebs_check;
  log_message.steer_dead = steer_dead;
  log_message.pc_dead = pc_dead;
  log_message.inversor_dead = inversor_dead;
  log_message.res_dead = res_dead;
  log_message.asms_on = asms_on;
  log_message.ts_on = ts_on;
  log_message.sdc_open = sdc_open;
  log_message.mission = mission;
  log_message.master_state = master_state;
  log_message.checkup_state = checkup_state;

  master_log_pub_->publish(log_message);
}

void RosCan::master_logs_2_publisher(const unsigned char msg[8]) {
  uint32_t dc_voltage = (msg[1] << 24) | (msg[2] << 16) | (msg[3] << 8) | msg[4];
  bool pneumatic1 = msg[5] & 0x01;
  bool pneumatic2 = msg[6] & 0x01;
  custom_interfaces::msg::MasterLog2 log_message_2;
  log_message_2.dc_voltage = dc_voltage;
  // log_message_2.pneumatic1 = pneumatic1;
  // log_message_2.pneumatic2 = pneumatic2;

  master_log_pub_2_->publish(log_message_2);
}

void RosCan::op_status_publisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.header.stamp = this->get_clock()->now();
  message.go_signal = this->go_signal_;
  message.as_mission = this->as_mission_;
  // RCLCPP_DEBUG(this->get_logger(),
  //              "Received Operational Status Message: Go Signal: %d --- AS Mission: %d",
  //              go_signal_, as_mission_);
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

// Used only to initially set actuator origin
void RosCan::steering_angle_cubem_publisher(const unsigned char msg[8]) {
  // When steering motor wakes up, set its origin
  if (!this->cubem_configuration_sent_) {
    this->cubem_configuration_sent_ = true;
    bosch_steering_angle_set_origin();
    this->cubem_set_origin();
  }

  int16_t angle = (msg[0] << 8) | msg[1];  // Extract 16-bit motor angle

  int speed = (msg[2] << 8) | msg[3];  // Extract 16-bit motor speed
  int current = (msg[4] << 8) | msg[5]; // Extract 16-bit motor current
  int temperature = msg[6]; // Extract 8-bit motor temperature
  int error = msg[7];  // Extract 8-bit motor error


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
  this->steering_angle_ = -angle;  // Used for initial adjustment
  RCLCPP_INFO(this->get_logger(), "Steering angle: %f", this->steering_angle_);

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

void RosCan::battery_voltage_callback(const unsigned char msg[8]) {
  this->battery_voltage_ = (msg[2] << 8) | msg[1];
  // RCLCPP_DEBUG(this->get_logger(), "Received voltage from Bamocar: %d", this->battery_voltage_);
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
void RosCan::hydraulic_line_callback(const unsigned char msg[8]) {
  int hydraulic_line_pressure = (msg[2] << 8) | msg[1];

  auto message = custom_interfaces::msg::HydraulicLinePressure();
  message.header.stamp = this->get_clock()->now();
  message.pressure = hydraulic_line_pressure;

  hydraulic_line_pressure_publisher_->publish(message);
}

RosCan::~RosCan() {
  RCLCPP_INFO(this->get_logger(), "Shutting down CAN interface");

  if (hnd_ >= 0) {
    canBusOff(hnd_);
    canClose(hnd_);
    hnd_ = -1;
  }
}