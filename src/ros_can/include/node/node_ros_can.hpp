#include <canlib.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <std_srvs/srv/trigger.hpp>

#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

class RosCan : public rclcpp::Node {
 private:
  enum class State { AS_Manual, AS_Off, AS_Ready, AS_Driving, AS_Finished, AS_Emergency };
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operationalStatus;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rlRPMPub;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rrRPMPub;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imuYawAccYPub;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imuRollAccXPub;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imuPitchAccZPub;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steeringAngleBosch;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr bosch_steering_angle_publisher;

  // Enum to hold the state of the AS
  State currentState = State::AS_Off;
  int battery_voltage = 0;
  int motor_speed = 0;
  int hydraulic_line_pressure = 0;

  // rclcpp::Subscription<std_msgs::msg::String::SharedPtr> busStatus;
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr controlListener;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_finished_service;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr timerAliveMsg;

  // Holds a handle to the CAN channel
  canHandle hnd;
  // Status returned by the Canlib calls
  canStatus stat;

  /*
  goSignal:
  0 - Stop
  1 - Go
  */
  bool goSignal = 0;

  /*Current Mission:
  0 - Manual
  1 - Acceleration
  2 - Skidpad
  3 - Autocross
  4 - Trackdrive
  5 - EBS_Test
  6 - Inspection*/
  int asMission;

  /**
   * Cubemars set origin sent
  */
  bool cubem_configuration_sent = false;

  /**
   * Steering angle from cubemars
  */
  double cubem_steering_angle = 0.0;

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   * @param busStatus - the status of the bus
   *
   *void busStatus_callback(std_msgs::msg::String busStatus);
   */
  
  /**
   * @brief Function cyclically reads all CAN msg from buffer
   */
  void canSniffer();

  /**
   * @brief Function to interpret the CAN msg
   * @param id - the CAN msg id
   * @param msg - the CAN msg
   * @param dlc - the CAN msg length
   * @param flag - the CAN msg flag - see kvaser documentation for more info
   * @param time - the CAN msg time stamp
   */
  void canInterpreter(long id, const unsigned char msg[8], unsigned int dlc, unsigned int flag,
                      unsigned long time);

  /**
   * @brief Function to publish the Operational Status
   */
  void opStatusPublisher();

  /**
   * @brief Function to publish the Yaw rate and Acceleration in Y
   * @param msg - the CAN msg
   */
  void imuYawAccYPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the Roll rate and Acceleration in X
   * @param msg - the CAN msg
   */
  void imuRollAccXPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the Pitch rate and Acceleration in Z
   * @param msg - the CAN msg
   */
  void imuPitchAccZPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle form steering actuator (CubeMars)
   * @param msg - the CAN msg
   */
  void steeringAngleCubeMPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle form Bosch
   * @param msg - the CAN msg
   */
  void steeringAngleBoschPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the rear right rpm
   * @param msg - the CAN msg
   */
  void rrRPMPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the rear left rpm
   * @param msg - the CAN msg
   */
  void rlRPMPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to receive hydraulic line pressure
   * Used to check if you can go to Driving
   * @param msg - the CAN msg
   */
  void hydraulicLineCallback(const unsigned char msg[8]);

  /**
   * @brief Function to publish the motor speed, coming from the encoder
   * @param msg - the CAN msg
  */
  void motorSpeedPublisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the battery voltage, from the BAMOCAR
   * @param msg - the CAN msg
  */
  void batteryVoltageCallback(const unsigned char msg[8]);

  /**
   * @brief Function to interpret the BAMOCAR CAN msg
   * @param msg - the CAN msg
  */
  void canInterpreterBamocar(const unsigned char msg[8]);

  /**
   * @brief Function to interpret the master status CAN msg
   * @param msg - the CAN msg
   */
  void canInterpreterMasterStatus(const unsigned char msg[8]);

  /**
   * @brief Function to handle the emergency message
   */
  void emergency_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Function to handle the mission finished message
   */
  void mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Function to handle the control command message
   */
  void control_callback(custom_interfaces::msg::ControlCommand::SharedPtr msg);

  /**
   * @brief Function to send the alive message from the AS CU to Master
  */
  void alive_msg_callback();

  /**
   * @brief Sets the angle origin of the Cubemars steering actuator
  */
  void cubem_set_origin();

  /**
   * @brief Sets the current position as the Bosch steering angle origin (permanent)
   * @note This function is not currently used as it is meant to be used only once
  */
  void bosch_steering_angle_set_origin();

 public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan();
};