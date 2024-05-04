
#include <canlib.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "node/node_ros_can.hpp"
#include "utils/utils.hpp"
#include "safety/safety-mechanisms.hpp"
#include "utils/constants.hpp"


RosCan::RosCan(std::shared_ptr<ICanLibWrapper> canLibWrapperParam)
        : Node("node_ros_can"), canLibWrapper(std::move(canLibWrapperParam)) {
  operationalStatus =
      this->create_publisher<custom_interfaces::msg::OperationalStatus>("/vehicle/operational_status", 10);

  rlRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rl_rpm", 10);
  rrRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("/vehicle/rr_rpm", 10);

  imuYawAccYPub = this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_yaw_acc_y", 10);
  imuRollAccXPub = this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_roll_acc_x", 10);
  imuPitchAccZPub = this->create_publisher<custom_interfaces::msg::ImuData>("/vehicle/imu_pitch_acc_z", 10);

  bosch_steering_angle_publisher =
      this->create_publisher<custom_interfaces::msg::SteeringAngle>("/vehicle/bosch_steering_angle", 10);
  controlListener = this->create_subscription<custom_interfaces::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));
  emergency_service = this->create_service<std_srvs::srv::Trigger>(
      "/as_msgs/emergency", std::bind(&RosCan::emergency_callback, this, std::placeholders::_1, std::placeholders::_2));
  mission_finished_service = this->create_service<std_srvs::srv::Trigger>(
      "/as_msgs/mission_finished", std::bind(&RosCan::mission_finished_callback, this, std::placeholders::_1, std::placeholders::_2));
  timer =
          this->create_wall_timer(std::chrono::microseconds(500), std::bind(&RosCan::canSniffer, this));
  timerAliveMsg = this->create_wall_timer(std::chrono::milliseconds(100),
                                          std::bind(&RosCan::alive_msg_callback, this));

  // initialize the CAN library
  canInitializeLibrary();
  // A channel to a CAN circuit is opened. The channel depend on the hardware
  hnd = canOpenChannel(0, canOPEN_EXCLUSIVE);
  // Setup CAN parameters for the channel
  stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 4, 0, 0);  // check these values later
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN parameters");
  }
  // There are different types of controllers, this is the default
  stat = canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup CAN controller");
  }
  stat = canBusOn(hnd);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to turn on CAN bus");
  }
  RCLCPP_INFO(this->get_logger(), "Node initialized");
}



// -------------- ROS TO CAN --------------

void RosCan::control_callback(custom_interfaces::msg::ControlCommand::SharedPtr controlCmd) {
  double steering_angle_command = 0.0;
  if (transform_steering_angle_command(controlCmd->steering, steering_angle_command) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform steering angle command");
    return;
  }

  // Check if the steering value is within the limits
  if (steering_angle_command < STEERING_LOWER_LIMIT || steering_angle_command > STEERING_UPPER_LIMIT) {
    RCLCPP_WARN(this->get_logger(), "Steering value out of range");
    return;
  }

  // Check if the throttle value is within the limits
  if (controlCmd->throttle < THROTTLE_LOWER_LIMIT || controlCmd->throttle > THROTTLE_UPPER_LIMIT) {
    RCLCPP_WARN(this->get_logger(), "Throttle value out of range");
    return;
  }

  if (currentState == State::AS_Driving) {
    RCLCPP_DEBUG(this->get_logger(), "State is Driving: Steering: %f, Throttle: %f",
                 steering_angle_command, controlCmd->throttle);
    canInitializeLibrary();  // initialize the CAN library again, just in case (could be removed)

    long id = STEERING_COMMAND_CUBEM_ID;
    char buffer_steering[4];
    create_steering_angle_command(steering_angle_command, buffer_steering);
    void* steering_requestData = static_cast<void*>(buffer_steering);
    unsigned int steering_dlc = 4;
    unsigned int flag = canMSG_EXT; // Steering motor used CAN Extended

    // Write the steering message to the CAN bus
    // DO NOT EVER EDIT THIS CODE BLOCK
    // CODE BLOCK START
    check_steering_safe(steering_requestData); // DO NOT REMOVE EVER
    stat = canLibWrapper->canWrite(hnd, id, steering_requestData, steering_dlc, flag);
    // CODE BLOCK END
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Write steering to CAN bus success: %f", 
        steering_angle_command);
    }

    // Prepare the throttle message
    int throttle_command = static_cast<int>(controlCmd->throttle * BAMOCAR_MAX_SCALE);
    
    // CRITICAL CHECK - DO NOT REMOVE
    // Limit brake command if needed
    if (throttle_command < 0) {
      throttle_command = std::max(throttle_command, 
        max_torque_dynamic_limits(this->battery_voltage, this->motor_speed));
    }

    long throttle_id = BAMO_COMMAND_ID;
    unsigned char buffer_throttle[3];
    buffer_throttle[0] = TORQUE_COMMAND_BAMO_BYTE;
    buffer_throttle[2] = (throttle_command >> 8) & 0xff;
    buffer_throttle[1] = throttle_command & 0xff;
    void* throttle_requestData = static_cast<void*>(buffer_throttle);
    unsigned int throttle_dlc = 3;

    // Write the throttle message to the CAN bus
    // DO NOT EVER EDIT THIS CODE BLOCK
    // CODE BLOCK START
    check_throttle_safe(throttle_requestData); // DO NOT REMOVE EVER
    stat = canLibWrapper->canWrite(hnd, throttle_id, throttle_requestData, throttle_dlc, flag);
    // CODE BLOCK END
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus");
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Write throttle to CAN bus success: %f", controlCmd->throttle);
    }
  }
}

void RosCan::emergency_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  response->success = true;
  
  // Prepare the emergency message
  long id = AS_CU_NODE_ID;            // Set the CAN ID
  unsigned char data = EMERGENCY_CODE;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_INFO(this->get_logger(), "Emergency signal received, sending to CAN");

  stat = canLibWrapper->canWrite(hnd, can_id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // Handle request
  response->success = true;

  // Prepare the emergency message
  long id = AS_CU_NODE_ID;            // Set the CAN ID
  unsigned char data = MISSION_FINISHED_CODE;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;
  RCLCPP_INFO(this->get_logger(), "Mission finished signal received, sending to CAN");

  stat = canLibWrapper->canWrite(hnd, can_id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::alive_msg_callback() {
  long can_id = AS_CU_NODE_ID;
  unsigned char data = ALIVE_MESSAGE;
  void *msg = &data;
  unsigned int dlc = 1;
  unsigned int flag = 0;
  RCLCPP_DEBUG(this->get_logger(), "Sending alive message to AS_CU_NODE");

  stat = canLibWrapper->canWrite(hnd, can_id, msg, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write alive message to CAN bus");
  }
}

void RosCan::cubem_set_origin() {
  long id = SET_ORIGIN_CUBEM_ID;
  char buffer[1];
  buffer[0] = 0x00;
  void *steering_requestData = static_cast<void *>(buffer);
  unsigned int steering_dlc = 1;
  unsigned int flag = canMSG_EXT; // Steering motor used CAN Extended
  RCLCPP_INFO(this->get_logger(), "Setting origin of steering controller");

  // Write the steering message to the CAN bus
  stat = canLibWrapper->canWrite(hnd, id, steering_requestData, steering_dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering controller");
  }
}

// Not currently used, check header file for more info
void RosCan::bosch_steering_angle_set_origin() {
  long id = SET_ORIGIN_BOSCH_STEERING_ANGLE_ID;
  char buffer[8] = {0};
  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_RESET; // Reset message code
  void* request_data = static_cast<void*>(buffer);
  unsigned int dlc = 8;
  unsigned int flag = 0;

  // Reset previous origin
  stat = canWrite(hnd, id, request_data, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering angle sensor");
  }
  sleep(2); // Necessary for configuration to sink in

  buffer[0] = SET_ORIGIN_BOSCH_STEERING_ANGLE_SET;
  request_data = static_cast<void*>(buffer);
  
  // Set new origin
  stat = canWrite(hnd, id, request_data, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering angle sensor");
  }
  sleep(2); // Necessary for configuration to sink in
}

/**
 * @brief Function to turn ON and OFF the CAN BUS
 */
/*void RosCan::busStatus_callback(std_msgs::msg::String busStatus) {
  if (busStatus.data == "ON") {
    hnd = canOpenChannel(0, canOPEN_CAN_FD);
    stat = canBusOn(hnd);
  } else if (busStatus.data == "OFF") {
    stat = canBusOff(hnd);
    canClose(hnd);
  }
}*/





// -------------- CAN TO ROS --------------

/**
 * @brief Function cyclically reads all CAN msg from buffer
 */
void RosCan::canSniffer() {
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;

  do {
    stat = canLibWrapper->canRead(hnd, &id, &msg, &dlc, &flag, &time);
    canInterpreter(id, msg, dlc, flag, time);
    RCLCPP_DEBUG(this->get_logger(), "Received message with ID: %x", id); // Intended
  } while (stat == canOK);
}

void RosCan::canInterpreter(long id, const unsigned char msg[8], unsigned int dlc, unsigned int flag,
                            unsigned long time) {
  switch (id) {
    case MASTER_STATUS: {
      canInterpreterMasterStatus(msg);
      break;
    }
    case IMU_YAW_RATE_ACC_Y_ID: {
      imuYawAccYPublisher(msg);
      break;
    }
    case IMU_ROLL_RATE_ACC_X_ID: {
      imuRollAccXPublisher(msg);
      break;
    }
    case IMU_PITCH_RATE_ACC_Z_ID: {
      imuPitchAccZPublisher(msg);
      break;
    }
    case TEENSY_C1: {
      if (msg[0] == TEENSY_C1_RR_RPM_CODE) rrRPMPublisher(msg);
      break;
    }
    case STEERING_CUBEM_ID: {
      steeringAngleCubeMPublisher(msg);
      break;
    }
    case STEERING_BOSCH_ID: {
      steeringAngleBoschPublisher(msg);
      break;
    }
    case BAMO_RESPONSE_ID: {
      // batteryVoltageCallback(msg);
      break;
    }
    default:
      break;
  }
}

void RosCan::canInterpreterBamocar(const unsigned char msg[8]) {
  switch (msg[0]) {
    case BAMOCAR_BATTERY_VOLTAGE_CODE: {
      batteryVoltageCallback(msg);
      break;
    }
    case BAMOCAR_MOTOR_SPEED_CODE: {
      motorSpeedPublisher(msg);
      break;
    }
    default:
      break;
  }
}

void RosCan::canInterpreterMasterStatus(const unsigned char msg[8]) {
  switch (msg[0]) {
    case MASTER_AS_STATE_CODE: {
      if (msg[1] == 3)  // If AS State == Driving
        this->goSignal = 1;
      else
        this->goSignal = 0;
      opStatusPublisher();
      break;
    }
    case MASTER_AS_MISSION_CODE: {
      this->asMission = msg[1];
      opStatusPublisher();
      break;
    }
    case MASTER_RL_RPM_CODE: {
      rlRPMPublisher(msg);
      break;
    }
    default:
      break;
  }
}

void RosCan::opStatusPublisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.header.stamp = this->get_clock()->now();
  message.go_signal = this->goSignal;
  message.as_mission = this->asMission;
  RCLCPP_DEBUG(this->get_logger(), "Received Operational Status Message: Go Signal: %d --- AS Mission: %d",
               goSignal, asMission);
  operationalStatus->publish(message);
}

void RosCan::imuYawAccYPublisher(const unsigned char msg[8]) {
  float yawRate = (msg[0]) * QUANTIZATION_GYRO;
  float accY = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = yawRate;
  message.acc = accY;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.Y and Yaw Rate Message: Yaw Rate: %f --- Acc Y: %f", yawRate,
                accY);
  imuYawAccYPub->publish(message);
}

void RosCan::imuRollAccXPublisher(const unsigned char msg[8]) {
  float rollRate = (msg[0]) * QUANTIZATION_GYRO;
  float accX = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = rollRate;
  message.acc = accX;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.X and Roll Rate Message: Roll Rate: %f --- Acc X: %f", rollRate,
                accX);
  imuRollAccXPub->publish(message);
}

void RosCan::imuPitchAccZPublisher(const unsigned char msg[8]) {
  float pitchRate = (msg[0]) * QUANTIZATION_GYRO;
  float accZ = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = pitchRate;
  message.acc = accZ;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.Z and Pitch Rate Message: Pitch Rate: %f --- Acc Z: %f",
                pitchRate, accZ);
  imuPitchAccZPub->publish(message);
}

// Useless right now
void RosCan::steeringAngleCubeMPublisher(const unsigned char msg[8]) {

  // When steering motor wakes up, set its origin
  if (!this->cubem_configuration_sent) {
    this->cubem_configuration_sent = true;
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
  // bosch_steering_angle_publisher->publish(message); // wrong publisher
}

void RosCan::steeringAngleBoschPublisher(const unsigned char msg[8]) {
  // Mask to check the first 3 bits
  char mask = 0b00100000;

  // Extract bits 7,6,5
  char first_three_bits = msg[6] & mask;

  // Error flags
  if (first_three_bits != mask) {
    RCLCPP_ERROR(this->get_logger(), "Invalid message received from Bosch Steering Wheel Sensor");
    return;
  }

  if (!checkCRC8(msg)) {
    RCLCPP_WARN(this->get_logger(), "Invalid CRC8 received from Bosch SA Sensor; dumping message...");
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

  // Send message
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = angle;
  message.steering_speed = speed;
  RCLCPP_DEBUG(this->get_logger(), "Received Bosch Steering Angle: %f", angle);
  bosch_steering_angle_publisher->publish(message);
}

void RosCan::rrRPMPublisher(const unsigned char msg[8]) {
  float rrRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rr_rpm = rrRPM;
  RCLCPP_DEBUG(this->get_logger(), "Received RR RPM: %f", rrRPM);
  rrRPMPub->publish(message);
}

void RosCan::rlRPMPublisher(const unsigned char msg[8]) {
  float rlRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rl_rpm = rlRPM;
  RCLCPP_DEBUG(this->get_logger(), "Received RL RPM: %f", rlRPM);
  rlRPMPub->publish(message);
}

/**
 * @brief public function to test the control_callback
 * @param msg - the controls from ROS(control) msg
 */
void RosCan::test_control_callback(custom_interfaces::msg::ControlCommand::SharedPtr msg) {
  control_callback(msg);
}

/**
 * @brief public function that sets the currentState to AS_Driving, helpful for testing
 */
void RosCan::setASDrivingState() { currentState = State::AS_Driving; }

/**
 * @brief public function that sets the currentState to AS_Off, helpful for testing
 */
void RosCan::setASOffState() { currentState = State::AS_Off; }

/**
 * @brief public function to test the canSniffer
 */
void RosCan::testCanSniffer() {
  canSniffer();
}
void RosCan::batteryVoltageCallback(const unsigned char msg[8]) {
  this->battery_voltage = (msg[2] << 8) | msg[1];
}

void RosCan::motorSpeedPublisher(const unsigned char msg[8]) {
  this->motor_speed = (msg[2] << 8) | msg[2];
  // TODO: publish motor speed
}
