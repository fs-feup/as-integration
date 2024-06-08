#include <canlib.h>
#include <gtest/gtest_prod.h>

#include <chrono>
#include <functional>
#include <memory>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "canlib_wrappers/ican_lib_wrapper.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

class RosCan : public rclcpp::Node {
private:
  enum class State { AS_MANUAL, AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY };
  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr operational_status_;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rl_rpm_pub_;
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr rr_rpm_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imu_yaw_acc_y_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imu_roll_acc_x_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ImuData>::SharedPtr imu_pitch_acc_z_pub_;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr steering_angle_bosch_;
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr
      bosch_steering_angle_publisher_;

  rclcpp::Publisher<sensor_msgs::Imu>::SharedPtr imu_acc_pub_;

  // Enum to hold the state of the AS
  State current_state_ = State::AS_OFF;
  int battery_voltage_ = 0;
  int motor_speed_ = 0;
  int hydraulic_line_pressure_ = 0;
  double steering_angle_ = 0.0;

  std::shared_ptr<ICanLibWrapper> can_lib_wrapper_;

  // rclcpp::Subscription<std_msgs::msg::String::SharedPtr> busStatus;
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr control_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_finished_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_alive_msg_;

  // Holds a handle to the CAN channel
  canHandle hnd_;
  // Status returned by the Canlib calls
  canStatus stat_;

  /*
  go_signal_:
  0 - Stop
  1 - Go
  */
  bool go_signal_ = 0;

  /*Current Mission:
  0 - Manual
  1 - Acceleration
  2 - Skidpad
  3 - Autocross
  4 - Trackdrive
  5 - EBS_Test
  6 - Inspection*/
  int as_mission_;

  /**
   * Cubemars set origin sent
   */
  bool cubem_configuration_sent_ = false;

  /**
   * Steering angle from cubemars
   */
  double cubem_steering_angle_ = 0.0;

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   * @param busStatus - the status of the bus
   *
   *void busStatus_callback(std_msgs::msg::String busStatus);
   */

  /**
   * @brief Function cyclically reads all CAN msg from buffer
   */
  void can_sniffer();

  /**
   * @brief Function to interpret the CAN msg
   * @param id - the CAN msg id
   * @param msg - the CAN msg
   * @param dlc - the CAN msg length
   * @param flag - the CAN msg flag - see kvaser documentation for more info
   * @param time - the CAN msg time stamp
   */
  void can_interpreter(long id, const unsigned char msg[8], unsigned int, unsigned int,
                       unsigned long);

  /**
   * @brief Function to publish the Operational Status
   */
  void op_status_publisher();

  /**
   * @brief Function to publish the Yaw rate and Acceleration in Y
   * @param msg - the CAN msg
   */
  void imu_yaw_acc_y_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the Roll rate and Acceleration in X
   * @param msg - the CAN msg
   */
  void imu_roll_acc_x_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the Pitch rate and Acceleration in Z
   * @param msg - the CAN msg
   */
  void imu_pitch_acc_z_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle form steering actuator (CubeMars)
   * @param msg - the CAN msg
   */
  void steering_angle_cubem_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the steering angle form Bosch
   * @param msg - the CAN msg
   */
  void steering_angle_bosch_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the rear right rpm
   * @param msg - the CAN msg
   */
  void rr_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the rear left rpm
   * @param msg - the CAN msg
   */
  void rl_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to receive hydraulic line pressure
   * Used to check if you can go to Driving
   * @param msg - the CAN msg
   */
  void hydraulic_line_callback(const unsigned char msg[8]);

  /**
   * @brief Function to publish the motor speed, coming from the encoder
   * @param msg - the CAN msg
   */
  void motor_speed_publisher(const unsigned char msg[8]);

  /**
   * @brief Function to publish the battery voltage, from the BAMOCAR
   * @param msg - the CAN msg
   */
  void battery_voltage_callback(const unsigned char msg[8]);

  /**
   * @brief Function to interpret the BAMOCAR CAN msg
   * @param msg - the CAN msg
   */
  void can_interpreter_bamocar(const unsigned char msg[8]);

  /**
   * @brief Function to interpret the master status CAN msg
   * @param msg - the CAN msg
   */
  void can_interpreter_master_status(const unsigned char msg[8]);

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
   * @brief Function to publish the steering command to CAN
   * @param double steering value from ROS (in radians)
   */
  void send_steering_control(double steering_angle_ros);

  /**
   * @brief Function to publish the throttle command to CAN
   * @param double value of the command from ROS
   */
  void send_throttle_control(double throttle_value_ros);

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
   * @brief public function that sets the current_state_ to AS_DRIVING, helpful for testing
   */
  void set_as_driving_state();

  /**
   * @brief public function that sets the current_state_ to AS_OFF, helpful for testing
   */
  void set_as_off_state();

  /**
   * @brief Contructor for the RosCan class
   */
  RosCan(std::shared_ptr<ICanLibWrapper> can_lib_wrapper_param);

  FRIEND_TEST(RosCanTest, ControlCallback);
  FRIEND_TEST(RosCanTest, PublishControlCallback);
  FRIEND_TEST(RosCanTest, TestImuYawAccYPublisher);
  FRIEND_TEST(RosCanTest, TestCanInterpreterMasterStatusMission);
  FRIEND_TEST(RosCanTest, TestCanInterpreter_TEENSY_C1_RR_RPM_CODE);
  FRIEND_TEST(RosCanTest, TestOutOfRangeUpperSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeLowerSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeSingleSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestCarStateMustBeDriving);
  FRIEND_TEST(RosCanTest, TestAliveMsgCallback);
};