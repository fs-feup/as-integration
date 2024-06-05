
#include "node/node_ros_can.hpp"

#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "safety/safety-mechanisms.hpp"
#include "utils/constants.hpp"
#include "utils/utils.hpp"

RosCan::RosCan(std::shared_ptr<ICanLibWrapper> can_lib_wrapper_param)
    : Node("node_ros_can"), can_lib_wrapper_(std::move(can_lib_wrapper_param)) {
  operational_status_ = this->create_publisher<custom_interfaces::msg::OperationalStatus>(
      "/vehicle/operational_status", 10);

  rl_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rl_rpm", 10);
  rr_rpm_pub_ = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rr_rpm", 10);

  imu_yaw_acc_y_pub_ =
      this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_yaw_acc_y", 10);
  imu_roll_acc_x_pub_ =
      this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_roll_acc_x", 10);
  imu_pitch_acc_z_pub_ =
      this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_pitch_acc_z", 10);

  bosch_steering_angle_publisher_ = this->create_publisher<custom_interfaces::msg::SteeringAngle>(
      "/vehicle/bosch_steering_angle", 10);
  control_listener_ = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));
  emergency_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/as_srv/emergency",
      std::bind(&RosCan::emergency_callback, this, std::placeholders::_1, std::placeholders::_2));
  mission_finished_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/as_srv/mission_finished", std::bind(&RosCan::mission_finished_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));
  timer_ =
      this->create_wall_timer(std::chrono::microseconds(500), std::bind(&RosCan::can_sniffer, this));
  timer_alive_msg_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&RosCan::alive_msg_callback, this));

  // initialize the CAN library
  canInitializeLibrary();
  // A channel to a CAN circuit is opened. The channel depend on the hardware
  hnd_ = canOpenChannel(0, canOPEN_CAN_FD);
  if (hnd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN channel. Handle value: %d", hnd_);
  }
  // Setup CAN parameters for the channel
  stat_ = canSetBusParams(hnd_, canBITRATE_500K, 0, 0, 1, 0, 0);  // check these values later
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN parameters");
  }
  // There are different types of controllers, this is the default
  stat_ = canSetBusOutputControl(hnd_, canDRIVER_NORMAL);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN controller");
  }
  stat_ = canBusOn(hnd_);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to turn on CAN bus");
  }
  RCLCPP_INFO(this->get_logger(), "Node initialized");
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

  if (current_state_ == State::AS_DRIVING) {
    RCLCPP_INFO(this->get_logger(), "State is Driving: Steering: %f (radians), Throttle: %f",
                msg->steering, msg->throttle);

      send_steering_control(msg->steering);
      send_throttle_control(msg->throttle);
  }
}

void RosCan::send_steering_control(double steering_angle_ros) {
  // Convert to steering in actuator
  double steering_angle_command = 0.0;
  if (transform_steering_angle_command(steering_angle_ros, steering_angle_command) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform steering angle command");
    return;
  }

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
  // CRITICAL CHECK - DO NOT REMOVE
  // Limit brake command if needed
  if (throttle_command < 0) {
    throttle_command = std::max(
        throttle_command, max_torque_dynamic_limits(this->battery_voltage_, this->motor_speed_));
  }

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
  stat_ = can_lib_wrapper_->canWrite(hnd_, throttle_id, throttle_requestData, throttle_dlc, flag);
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

  // Prepare the emergency message
  long id = AS_CU_NODE_ID;              // Set the CAN ID
  unsigned char data = EMERGENCY_CODE;  // Set the data
  void *requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_INFO(this->get_logger(), "Emergency signal received, sending to CAN");

  stat_ = can_lib_wrapper_->canWrite(hnd_, id, requestData, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  response->success = true;

  // Prepare the emergency message
  long id = AS_CU_NODE_ID;                     // Set the CAN ID
  unsigned char data = MISSION_FINISHED_CODE;  // Set the data
  void *requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_INFO(this->get_logger(), "Mission finished signal received, sending to CAN");

  stat_ = can_lib_wrapper_->canWrite(hnd_, id, requestData, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::alive_msg_callback() {
  long id = AS_CU_NODE_ID;
  unsigned char data = ALIVE_MESSAGE;
  void *msg = &data;
  unsigned int dlc = 1;
  unsigned int flag = 0;
  RCLCPP_DEBUG(this->get_logger(), "Sending alive message to AS_CU_NODE");

  stat_ = can_lib_wrapper_->canWrite(hnd_, id, msg, dlc, flag);
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
  stat_ = can_lib_wrapper_->canWrite(hnd_, id, steering_requestData, steering_dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering controller");
  }
}

// Not currently used, check header file for more info
void RosCan::bosch_steering_angle_set_origin() {
  long id = SET_ORIGIN_BOSCH_STEERING_ANGLE_ID;
  char buffer[8] = {0};
  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET;  // Reset message code
  void *request_data = static_cast<void *>(buffer);
  unsigned int dlc = 8;
  unsigned int flag = 0;

  // Reset previous origin
  stat_ = canWrite(hnd_, id, request_data, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering angle sensor");
  }
  sleep(2);  // Necessary for configuration to sink in

  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_SET;
  request_data = static_cast<void *>(buffer);

  // Set new origin
  stat_ = canWrite(hnd_, id, request_data, dlc, flag);
  if (stat_ != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering angle sensor");
  }
  sleep(2);  // Necessary for configuration to sink in
}

/**
 * @brief Function to turn ON and OFF the CAN BUS
 */
/*void RosCan::busStatus_callback(std_msgs::msg::String busStatus) {
  if (busStatus.data == "ON") {
    hnd_ = canOpenChannel(0, canOPEN_CAN_FD);
    stat_ = canBusOn(hnd_);
  } else if (busStatus.data == "OFF") {
    stat_ = canBusOff(hnd_);
    canClose(hnd_);
  }
}*/

// -------------- CAN TO ROS --------------

/**
 * @brief Function cyclically reads all CAN msg from buffer
 */
void RosCan::can_sniffer() {
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;

  stat_ = can_lib_wrapper_->canRead(hnd_, &id, &msg, &dlc, &flag, &time);
  while (stat_ == canOK) {
      can_interpreter(id, msg, dlc, flag, time);
    stat_ = can_lib_wrapper_->canRead(hnd_, &id, &msg, &dlc, &flag, &time);
  }
}

void RosCan::can_interpreter(long id, const unsigned char msg[8], unsigned int, unsigned int,
                             unsigned long) {
  switch (id) {
    case MASTER_STATUS: {
        can_interpreter_master_status(msg);
      break;
    }
    case IMU_YAW_RATE_ACC_Y_ID: {
        imu_yaw_acc_y_publisher(msg);
      break;
    }
    case IMU_ROLL_RATE_ACC_X_ID: {
        imu_roll_acc_x_publisher(msg);
      break;
    }
    case IMU_PITCH_RATE_ACC_Z_ID: {
        imu_pitch_acc_z_publisher(msg);
      break;
    }
    case TEENSY_C1: {
      if (msg[0] == TEENSY_C1_RR_RPM_CODE) {
        rr_rpm_publisher(msg);
      } else if (msg[0] == TEENSY_C1_RL_RPM_CODE) {
        rl_rpm_publisher(msg);
      }
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
    default:
      break;
  }
}

void RosCan::can_interpreter_master_status(const unsigned char msg[8]) {
  switch (msg[0]) {
    case MASTER_AS_STATE_CODE: {
      if (msg[1] == 3) {  // If AS State == Driving
        // Fix initial actuator angle
        if (this->go_signal_ == 0) {
            this->send_steering_control(-this->steering_angle_);
        }
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
    default:
      break;
  }
}

void RosCan::op_status_publisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.header.stamp = this->get_clock()->now();
  message.go_signal = this->go_signal_;
  message.as_mission = this->as_mission_;
  RCLCPP_DEBUG(this->get_logger(),
               "Received Operational Status Message: Go Signal: %d --- AS Mission: %d", go_signal_,
               as_mission_);
  operational_status_->publish(message);
}

void RosCan::imu_yaw_acc_y_publisher(const unsigned char msg[8]) {
  float yawRate = (msg[0]) * QUANTIZATION_GYRO;
  float accY = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = yawRate;
  message.acc = accY;
  RCLCPP_DEBUG(this->get_logger(),
               "Received IMU Acc.Y and Yaw Rate Message: Yaw Rate: %f --- Acc Y: %f", yawRate,
               accY);
  imu_yaw_acc_y_pub_->publish(message);
}

void RosCan::imu_roll_acc_x_publisher(const unsigned char msg[8]) {
  float rollRate = (msg[0]) * QUANTIZATION_GYRO;
  float accX = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = rollRate;
  message.acc = accX;
  RCLCPP_DEBUG(this->get_logger(),
               "Received IMU Acc.X and Roll Rate Message: Roll Rate: %f --- Acc X: %f", rollRate,
               accX);
  imu_roll_acc_x_pub_->publish(message);
}

void RosCan::imu_pitch_acc_z_publisher(const unsigned char msg[8]) {
  float pitchRate = (msg[0]) * QUANTIZATION_GYRO;
  float accZ = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = pitchRate;
  message.acc = accZ;
  RCLCPP_DEBUG(this->get_logger(),
               "Received IMU Acc.Z and Pitch Rate Message: Pitch Rate: %f --- Acc Z: %f", pitchRate,
               accZ);
  imu_pitch_acc_z_pub_->publish(message);
}

// Useless right now
void RosCan::steering_angle_cubem_publisher(const unsigned char msg[8]) {
  // When steering motor wakes up, set its origin
  if (!this->cubem_configuration_sent_) {
    this->cubem_configuration_sent_ = true;
      this->cubem_set_origin();
  }

  // TODO: fix this code
  int angle = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
  int speed = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = static_cast<double>(angle) / 1000000;
  message.steering_speed = static_cast<double>(speed);
  RCLCPP_DEBUG(this->get_logger(), "Cubemars steering angle received: %f", message.steering_angle);
  // bosch_steering_angle_publisher_->publish(message); // wrong publisher
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

  RCLCPP_DEBUG(this->get_logger(), "Received Bosch Steering Angle (degrees): %f", angle);
  speed = speed * M_PI / 180;
  angle = angle * M_PI / 180;
  this->steering_angle_ = angle;  // Used for initial adjustment

  // Send message
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = angle;
  message.steering_speed = speed;
  RCLCPP_DEBUG(this->get_logger(), "Received Bosch Steering Angle (radians): %f", angle);
  bosch_steering_angle_publisher_->publish(message);
}

void RosCan::rr_rpm_publisher(const unsigned char msg[8]) {
  float rrRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rr_rpm = rrRPM;
  RCLCPP_DEBUG(this->get_logger(), "Received RR RPM: %f", rrRPM);
  rr_rpm_pub_->publish(message);
}

void RosCan::rl_rpm_publisher(const unsigned char msg[8]) {
  float rlRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rl_rpm = rlRPM;
  RCLCPP_DEBUG(this->get_logger(), "Received RL RPM: %f", rlRPM);
  rl_rpm_pub_->publish(message);
}

/**
 * @brief public function that sets the current_state_ to AS_DRIVING, helpful for testing
 */
void RosCan::set_as_driving_state() { current_state_ = State::AS_DRIVING; }

/**
 * @brief public function that sets the current_state_ to AS_OFF, helpful for testing
 */
void RosCan::set_as_off_state() { current_state_ = State::AS_OFF; }

void RosCan::battery_voltage_callback(const unsigned char msg[8]) {
  this->battery_voltage_ = (msg[2] << 8) | msg[1];
}

void RosCan::motor_speed_publisher(const unsigned char msg[8]) {
  this->motor_speed_ = (msg[2] << 8) | msg[1];
  // TODO: publish motor speed
}
