
#include <canlib.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fs_msgs/msg/control_command.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

#include "node/node_ros_can.hpp"
#include "cubemars/steering_utils.hpp"

RosCan::RosCan() : Node("node_ros_can") {
  operationalStatus =
      this->create_publisher<custom_interfaces::msg::OperationalStatus>("operationalStatus", 10);

  rlRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("rlRPM", 10);
  rrRPMPub = this->create_publisher<custom_interfaces::msg::WheelRPM>("rrRPM", 10);

  imuYawAccYPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuYawAccY", 10);
  imuRollAccXPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuRollAccXPub", 10);
  imuPitchAccZPub = this->create_publisher<custom_interfaces::msg::ImuData>("imuPitchAccZPub", 10);

  steeringAngleCubeM =
      this->create_publisher<custom_interfaces::msg::SteeringAngle>("steeringAngleCubeM", 10);
  controlListener = this->create_subscription<fs_msgs::msg::ControlCommand>(
      "/as_msgs/controls", 10, std::bind(&RosCan::control_callback, this, std::placeholders::_1));
  emergencyListener = this->create_subscription<std_msgs::msg::String>(
      "/as_msgs/emergency", 10,
      std::bind(&RosCan::emergency_callback, this, std::placeholders::_1));  // maybe change type
  missionFinishedListener = this->create_subscription<fs_msgs::msg::FinishedSignal>(
      "/as_msgs/mission_finished", 10,
      std::bind(&RosCan::mission_finished_callback, this,
                std::placeholders::_1));  // maybe change type
  // busStatus = this->create_subscription<std_msgs::msg::String>("busStatus", 10,
  // std::bind(&RosCan::busStatus_callback, this, std::placeholders::_1));
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
}

void RosCan::control_callback(fs_msgs::msg::ControlCommand::SharedPtr controlCmd) {
  // Check if the steering value is within the limits
  if (controlCmd->steering < STEERING_LOWER_LIMIT || controlCmd->steering > STEERING_UPPER_LIMIT) {
    RCLCPP_ERROR(this->get_logger(), "Steering value out of range");
    return;
  }
  // Check if the throttle value is within the limits
  if (controlCmd->throttle < THROTTLE_LOWER_LIMIT || controlCmd->throttle > THROTTLE_UPPER_LIMIT) {
    RCLCPP_ERROR(this->get_logger(), "Throttle value out of range");
    return;
  }
  if (currentState == State::AS_Driving || true) {
    RCLCPP_INFO(this->get_logger(), "State is Driving: Steering: %f, Throttle: %f",
                 controlCmd->steering, controlCmd->throttle);
    canInitializeLibrary();  // initialize the CAN library again, just in case (could be removed)

    // Prepare the steering message
    long id = STEERING_COMMAND_CUBEM_ID;
    char buffer[4];
    create_steering_angle_command(controlCmd->steering, buffer);
    void* steering_requestData = static_cast<void*>(buffer);
    unsigned int steering_dlc = 4;
    unsigned int flag = canMSG_EXT; // Steering motor used CAN Extended

    // Write the steering message to the CAN bus
    stat = canWrite(hnd, id, steering_requestData, steering_dlc, flag);
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write steering to CAN bus");
    }

    if (stat == canOK) {
      RCLCPP_INFO(this->get_logger(), "Write steering to CAN bus success");
    }

    RCLCPP_INFO(this->get_logger(), "Write res: %d", stat);

    // Prepare the throttle message
    long throttle_id = BAMO_RECEIVER; 
    void* throttle_requestData = static_cast<void*>(&controlCmd->throttle);
    unsigned int throttle_dlc = 3;

    // Write the throttle message to the CAN bus
    stat = canWrite(hnd, throttle_id, throttle_requestData, throttle_dlc, flag);
    if (stat != canOK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write throttle to CAN bus");
    }
  }
}

void RosCan::emergency_callback(std_msgs::msg::String::SharedPtr msg) {
  // Prepare the emergency message
  long id = AS_CU_NODE_ID;            // Set the CAN ID
  unsigned char data = EMERGENCY_CODE;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::mission_finished_callback(fs_msgs::msg::FinishedSignal::SharedPtr msg) {
  // Prepare the emergency message
  long id = AS_CU_NODE_ID;            // Set the CAN ID
  unsigned char data = MISSION_FINISHED_CODE;  // Set the data
  void* requestData = &data;
  unsigned int dlc = 1;  // Set the length of the data
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, requestData, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write emergency message to CAN bus");
  }
}

void RosCan::alive_msg_callback() {
  long id = AS_CU_NODE_ID;            // Set the CAN ID
  unsigned char data = ALIVE_MESSAGE;  // Set the data
  void* msg = &data;
  unsigned int dlc = 1;  // Msg length
  unsigned int flag = 0;

  // Write the emergency message to the CAN bus
  stat = canWrite(hnd, id, msg, dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write alive message to CAN bus");
  }
}

void RosCan::cubem_set_origin() {
  long id = SET_ORIGIN_CUBEM_ID;
  char buffer[1];
  buffer[0] = 0x00;
  void* steering_requestData = static_cast<void*>(buffer);
  unsigned int steering_dlc = 1;
  unsigned int flag = canMSG_EXT; // Steering motor used CAN Extended

  // Write the steering message to the CAN bus
  stat = canWrite(hnd, id, steering_requestData, steering_dlc, flag);
  if (stat != canOK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set origin of steering controller");
  }
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
    stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);
    canInterpreter(id, msg, dlc, flag, time);
    RCLCPP_DEBUG(this->get_logger(), "Received message with ID: %x", id);
  } while (stat == canOK);
}

void RosCan::canInterpreter(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag,
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
    default:
      break;
  }
}

void RosCan::canInterpreterMasterStatus(unsigned char msg[8]) {
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

/**
 * @brief Function to publish the Operational Status
 */
void RosCan::opStatusPublisher() {
  auto message = custom_interfaces::msg::OperationalStatus();
  message.header.stamp = this->get_clock()->now();
  message.go_signal = goSignal;
  message.as_mission = asMission;
  operationalStatus->publish(message);
}

/**
 * @brief Function to publish the Yaw rate and acceleration in y
 * @param msg - the CAN msg
 */
void RosCan::imuYawAccYPublisher(unsigned char msg[8]) {
  float yawRate = (msg[0]) * QUANTIZATION_GYRO;
  float accY = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = yawRate;
  message.acc = accY;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.Y and Yaw Rate Message: Yaw Rate: %f --- Acc Y: %f", yawRate, accY);
  imuYawAccYPub->publish(message);
}

/**
 * @brief Function to publish the Roll rate and acceleration in X
 * @param msg - the CAN msg
 */
void RosCan::imuRollAccXPublisher(unsigned char msg[8]) {
  float rollRate = (msg[0]) * QUANTIZATION_GYRO;
  float accX = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = rollRate;
  message.acc = accX;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.X and Roll Rate Message: Roll Rate: %f --- Acc X: %f", rollRate, accX);
  imuRollAccXPub->publish(message);
}

/**
 * @brief Function to publish the Pitch rate and acceleration in Z
 * @param msg - the CAN msg
 */
void RosCan::imuPitchAccZPublisher(unsigned char msg[8]) {
  float pitchRate = (msg[0]) * QUANTIZATION_GYRO;
  float accZ = (msg[4]) * QUANTIZATION_ACC;
  auto message = custom_interfaces::msg::ImuData();
  message.header.stamp = this->get_clock()->now();
  message.gyro = pitchRate;
  message.acc = accZ;
  RCLCPP_DEBUG(this->get_logger(), "Received IMU Acc.Z and Pitch Rate Message: Pitch Rate: %f --- Acc Z: %f", pitchRate, accZ);
  imuPitchAccZPub->publish(message);
}

/**
 * @brief Function to publish the steering angle form steering actuator (CubeMars)
 * @param msg - the CAN msg
 */
void RosCan::steeringAngleCubeMPublisher(unsigned char msg[8]) {

  // When steering motor wakes up, set its origin
  if (!this->cubem_configuration_sent) {
    this->cubem_configuration_sent = true;
    this->cubem_set_origin();
  }

  int angle = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
  int speed = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = static_cast<double>(angle) / 1000000;
  message.steering_speed = static_cast<double>(angle);
  RCLCPP_DEBUG(this->get_logger(), "Cubemars steering angle received: %lf", message.steering_angle);
  steeringAngleCubeM->publish(message);
}

/**
 * @brief Function to publish the steering angle form Bosch
 * @param msg - the CAN msg
 */
void RosCan::steeringAngleBoschPublisher(unsigned char msg[8]) {
  // Mask to check the first 3 bits
  char mask = 0b00000111;

  // Extract the first 3 bits
  char firstThreeBits = msg[3] & mask;

  if (firstThreeBits == mask) {
    int angle = (msg[1] << 8) | msg[0];
    int speed = msg[2];
    auto message = custom_interfaces::msg::SteeringAngle();
    message.steering_angle = angle;
    message.steering_speed = speed;
    steeringAngleBosch->publish(message);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid message received from Bosch Speed Sensor");
  }

  int angle = (msg[1] << 8) | msg[0];
  auto message = custom_interfaces::msg::SteeringAngle();
  message.header.stamp = this->get_clock()->now();
  message.steering_angle = angle;
  RCLCPP_DEBUG(this->get_logger(), "Received Bosch Steering Angle: %d", angle);
  steeringAngleCubeM->publish(message);
}

/**
 * @brief Function to publish rrRPM
 * @param msg - the CAN msg
 */
void RosCan::rrRPMPublisher(unsigned char msg[8]) {
  float rrRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rr_rpm = rrRPM;
  rrRPMPub->publish(message);
}

/**
 * @brief Function to publish rlRPM
 * @param msg - the CAN msg
 */
void RosCan::rlRPMPublisher(unsigned char msg[8]) {
  float rlRPM = ((msg[4] << 24) | (msg[3] << 16) | (msg[2] << 8) | msg[1]) / 100.0f;
  auto message = custom_interfaces::msg::WheelRPM();
  message.header.stamp = this->get_clock()->now();
  message.rl_rpm = rlRPM;
  rlRPMPub->publish(message);
}
