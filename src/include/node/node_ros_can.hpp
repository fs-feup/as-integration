#include <canlib.h>
#include <gtest/gtest_prod.h>
#include <std_msgs/Int32.h>

#include <chrono>
#include <functional>
#include <memory>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "canlib_wrappers/ican_lib_wrapper.hpp"
#include "custom_interfaces/msg/bms_errors.hpp"
#include "custom_interfaces/msg/can_statistics.hpp"
#include "custom_interfaces/msg/cells_temps.hpp"
#include "custom_interfaces/msg/bamocar_errors.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/data_log_info1.hpp"
#include "custom_interfaces/msg/data_log_info2.hpp"
#include "custom_interfaces/msg/hydraulic_line_pressure.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/imu_acceleration.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/temperature.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "custom_interfaces/msg/yaw_pitch_roll.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"

/**
 * @class RosCan
 * @brief Provides a translation interface between ROS and CAN communication.
 *
 * The `RosCan` class translates between CAN messages and ROS topics/services. It handles CAN
 * communication using the `ICanLibWrapper` and publishes data to ROS topics. Additionally, it
 * subscribes to ROS topics and services, forwarding commands and status updates to the CAN bus.
 */
class RosCan : public rclcpp::Node {
private:
  /**
   * @enum State
   * @brief Represents the various operational states of the vehicle.
   */
  enum class State { AS_MANUAL, AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY };

  bool is_debug_ = false;  ///< Flag to enable debug mode

  rclcpp::Publisher<custom_interfaces::msg::OperationalStatus>::SharedPtr
      operational_status_;  ///< Publisher for operational status
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      rl_rpm_pub_;  ///< Publisher for rear left wheel RPM
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      rr_rpm_pub_;  ///< Publisher for rear right wheel RPM

  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      fl_rpm_pub_;  ///< Publisher for front left wheel RPM
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      fr_rpm_pub_;  ///< Publisher for front right wheel RPM
  rclcpp::Publisher<custom_interfaces::msg::WheelRPM>::SharedPtr
      motor_rpm_pub_;  ///< Publisher for motor RPM
  rclcpp::Publisher<custom_interfaces::msg::DataLogInfo1>::SharedPtr
      data_log_info_1_pub_;  ///< Publisher for data log info 1
  rclcpp::Publisher<custom_interfaces::msg::DataLogInfo2>::SharedPtr
      data_log_info_2_pub_;  ///< Publisher for data log info 2
  rclcpp::Publisher<custom_interfaces::msg::Temperature>::SharedPtr
      motor_temp_pub_;  ///< Publisher for motor temp
  rclcpp::Publisher<custom_interfaces::msg::Temperature>::SharedPtr
      inverter_temp_pub_;  ///< Publisher for motor temp
  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr
      bosch_steering_angle_publisher_;  ///< Publisher for Bosch steering angle
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
      inverter_voltage_pub_;  ///< Publisher for inverter voltage(bamocar)
  rclcpp::Publisher<custom_interfaces::msg::BamocarErrors>::SharedPtr _inverter_errors_pub_;  ///< Publisher for inverter errors
  rclcpp::Publisher<custom_interfaces::msg::BmsErrors>::SharedPtr
      bms_custom_pub_;  ///< Publisher for BMS errors
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
      apps_higher_pub_;  ///< Publisher for APPs higher value
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
      apps_lower_pub_;  ///< Publisher for APPs lower value
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      implausability_pub_;  ///< Publisher for implausibility status
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      driving_state_pub_;  ///< Publisher for driving state
  rclcpp::Publisher<custom_interfaces::msg::CanStatistics>::SharedPtr
      can_line_0_stats_pub_;  ///< Publisher for statistics of the first CAN line
  rclcpp::Publisher<custom_interfaces::msg::CanStatistics>::SharedPtr
      can_line_1_stats_pub_;  ///< Publisher for statistics of the second CAN line

  rclcpp::Publisher<custom_interfaces::msg::SteeringAngle>::SharedPtr
      steering_motor_state_pub_;  ///< Publisher for CubeM Motor Steering angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      steering_motor_current;  // Publisher for CubeMars motor current
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      steering_motor_temperature;  // Publisher for Cube Mars Steering motor temperature
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      steering_motor_error;  // Publisher for steering motor error

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _bamocar_current_pub;

  rclcpp::Publisher<custom_interfaces::msg::CellsTemps>::SharedPtr
      cells_temps_pub_;  ///< Subscriber cells temperatures

  rclcpp::Publisher<custom_interfaces::msg::HydraulicLinePressure>::SharedPtr
      hydraulic_line_pressure_publisher_;  ///< Publisher for hydraulic line pressure

  // IMU Data Publishers
  rclcpp::Publisher<custom_interfaces::msg::ImuAcceleration>::SharedPtr
      imu_acc_pub_;  ///< Publisher for IMU acceleration data
  rclcpp::Publisher<custom_interfaces::msg::YawPitchRoll>::SharedPtr
      imu_angular_velocity_pub_;  ///< Publisher for IMU angular velocity data

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      manual_throttle_pub_;  ///< Publisher for manual throttle value

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      manual_brake_pub_;  ///< Publisher for manual brake value

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      apps_error_pub_;
      
  std::shared_ptr<ICanLibWrapper> can_lib_wrapper_;  ///< Wrapper for CAN library  
  // rclcpp::Subscription<std_msgs::msg::String::SharedPtr> busStatus;
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr
  control_listener_;  ///< Subscription for control commands  
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr
  perception_subscription_;  ///< Subscription for perception data  
  rclcpp::Subscription<custom_interfaces::msg::Velocities>::SharedPtr
  velocities_subscription_;  ///< Subscription for velocity data  
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr
  map_subscription_;  ///< Subscription for map data  
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
  lap_counter_subscription_;  ///< Subscription for lap counter data  
  rclcpp::Subscription<custom_interfaces::msg::PathPointArray>::SharedPtr
  path_subscription_;  ///< Subscription for path data  
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
  movella_imu_subscription_;  ///< Subscription for path data  
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  emergency_service_;  ///< Service for emergency handling
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  mission_finished_service_;  ///< Service for mission status updates
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  bosch_steering_angle_reset_service_;        ///< Service for reset bosch steering angle
  rclcpp::TimerBase::SharedPtr timer_;            ///< Timer for periodic tasks
  rclcpp::TimerBase::SharedPtr timer_alive_msg_;  ///< Timer for sending alive messages
  rclcpp::TimerBase::SharedPtr dv_timer_;         ///< Timer for sending driving dynamics messages
  rclcpp::TimerBase::SharedPtr can_statistics_timer_;  ///< Timer for CAN statistics  
  rclcpp::TimerBase::SharedPtr _cell_temps_timer_;  ///< Timer for cell temperatures
  canHandle hnd0_;  ///< Handle to the CAN channel
  canHandle hnd1_;  ///< Handle to the CAN channel  
  canStatus stat_;  ///< Status of the last CANlib call  


  // Internal Logging Variables  

  State current_state_ = State::AS_OFF;  ///< Current operational state of the vehicle
  int inverter_voltage_ = 0;             ///< Battery voltage in volts, in Bamocar scale
  int motor_speed_ = 0;                  ///< Motor speed in RPM, in Bamocar scale
  int motor_temp_ = 0;                   ///< Motor temp
  int inverter_temp_ = 0;                ///< Inverter temp
  int hydraulic_line_pressure_ = 0;      ///< Hydraulic line pressure
  double steering_angle_ = 0.0;          ///< Steering angle in radians (steering column)
  bool go_signal_ = false;  ///< Flag to control vehicle (false - stop/ true - go)  
  /* Current Mission:
  0 - Manual
  1 - Acceleration
  2 - Skidpad
  3 - Autocross
  4 - Trackdrive
  5 - EBS_Test
  6 - Inspection
  */
  int as_mission_;  ///< Current mission index

  int32_t apps_higher_value; ///< Value of the higher APPS (throttle sensor)
  int32_t apps_lower_value; ///< Value of the lower APPS (throttle sensor)
  int8_t _cells_temps_[120];  ///< Array to hold temperatures of all cells
  int8_t _cells_temps_max_ = 0;  ///< Maximum temperature of all cells
  int8_t _cells_temps_avg_ = 0;  ///< Average temperature of
  int8_t _cells_temps_min_ = 0;

  bool cubem_configuration_sent_ = false;  ///< Flag to check if Cubemars configuration is sent

  double cubem_steering_angle_ = 0.0;  ///< Steering angle from Cubemars

  // DV driving dynamics 1
  uint8_t speed_actual_ = 0;
  uint8_t speed_target_ = 0;
  int8_t steering_angle_actual_ = 0;
  int8_t steering_angle_target_ = 0;
  uint8_t brake_hydr_actual_ = 0;
  uint8_t brake_hydr_target_ = 0;
  int8_t motor_moment_actual_ = 0;
  int8_t motor_moment_target_ = 0;

  // DV driving dynamics 2
  int16_t acceleration_longitudinal_ = 0;
  int16_t acceleration_lateral_ = 0;
  int16_t yaw_rate_ = 0;

  // DV system status
  uint8_t as_status_ = 1;
  uint8_t asb_ebs_state_ = 1;
  uint8_t ami_state_ = 1;
  bool steering_state_ = false;
  uint8_t asb_redundancy_state_ = 1;
  uint8_t lap_counter_ = 0;
  uint8_t cones_count_actual_ = 0;
  uint16_t cones_count_all_ = 0;

  /**
   * @brief Function to turn ON and OFF the CAN BUS
   * @param busStatus - the status of the bus

   void busStatus_callback(std_msgs::msg::String busStatus);
  */

  /**
   * @brief Function to cyclically read all CAN messages from the buffer.
   * This function continuously monitors the CAN bus and processes incoming CAN messages.
   */
  void can_sniffer();

  /**
   * @brief Parses and interprets a CAN message and processes it based on its ID and content.
   * @param id CAN message ID
   * @param msg CAN message data
   * @param dlc Data length code
   * @param flag CAN message flags - see kvaser documentation for more info
   * @param time Timestamp of the CAN message
   */
  void can_interpreter(long id, const unsigned char msg[8], unsigned int dlc, unsigned int,
                       unsigned long);

  void bms_cells_temps_callback(const unsigned char msg[8]);

  void bamocar_current_publisher(const unsigned char msg[8]);

  /**
   * @brief Reads the statistics of the CAN lines and publishes them to ROS.
   * 
   */
  void can_statistics_callback();

    /**
     * @brief Reads the CAN statistics for a specific channel and populates the message.
     * @param handle CAN handle of the channel
     * @param msg Message to populate with statistics
     * @return true if statistics were read successfully, false otherwise
     */
  bool read_can_statistics(canHandle handle, custom_interfaces::msg::CanStatistics& msg);


  /**
   * @brief Publishes the current operational status to ROS.
   */
  void op_status_publisher();

  /**
   * @brief Receives IMU accelerations from CAN and publishes them to ROS
   * @param msg CAN message data
   */
  void imu_acc_publisher(const unsigned char msg[8]);

  /**
   * @brief Receives IMU orientation from CAN and publishes them to ROS.
   * @param msg CAN message data
   */
  void imu_angular_velocity_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the steering angle from Cubemars steering actuator.
   * Used only to initially set actuator origin
   * @param msg CAN message data
   */
  void steering_angle_cubem_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the steering angle from Bosch.
   * @param msg CAN message data
   */
  void steering_angle_bosch_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the rear right RPM to ROS.
   * @param msg CAN message data
   */
  void rr_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the rear left RPM to ROS.
   * @param msg CAN message data
   */
  void rl_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the front right RPM to ROS.
   * @param msg CAN message data
   */
  void fr_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the front left RPM to ROS.
   * @param msg CAN message data
   */
  void fl_rpm_publisher(const unsigned char msg[8]);

  /**
   * @brief Receives hydraulic line pressure from CAN and processes it.
   * Used to check if you can go to Driving
   * @param msg CAN message data
   */
  void hydraulic_line_callback(const unsigned char msg[8]);

  /**
   * @brief Publishes the motor speed, received from the encoder, to ROS.
   *
   * @param msg CAN message data
   */
  void motor_speed_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the motor temp, received from bamocar, to ROS.
   *
   * @param msg CAN message data
   */
  void motor_temp_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the inverter temp, received from bamocar, to ROS.
   *
   * @param msg CAN message data
   */
  void inverter_temp_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the battery voltage, received from BAMOCAR, to ROS.
   *
   * @param msg CAN message data
   */
  void inverter_voltage_publisher(const unsigned char msg[8]);

  /**
   * @brief Interprets the BAMOCAR CAN message.
   *
   * @param msg CAN message data
   */
  void can_interpreter_bamocar(const unsigned char msg[8]);

  /**
   * @brief Interprets the master CAN messages.
   *
   * @param msg CAN message data
   */
  void can_interpreter_master(const unsigned char msg[8]);

  /**
   * @brief Interprets the dash CAN messages.
   *
   * @param msg CAN message data
   */
  void dash_interpreter(const unsigned char msg[8]);


  void data_log_info_1_publisher(const unsigned char msg[8]);
  void data_log_info_2_publisher(const unsigned char msg[8]);
  void manual_throttle_publisher(const unsigned char msg[8]);


  /**
   * @brief Handles the emergency service callback.
   *
   * @param request Service request
   * @param response Service response
   */
  void emergency_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Handles the mission finished service callback.
   *
   * @param request Service request
   * @param response Service response
   */
  void mission_finished_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Handles the request to reset the steering angle value from the Bosch sensor.
   *
   * @param request Service request
   * @param response Service response
   */
  void bosch_sa_reset_callack(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Handles the control command message callback.
   *
   * @param msg Control command message
   */
  void control_callback(custom_interfaces::msg::ControlCommand::SharedPtr msg);

  /**
   * @brief Sends the steering control command to CAN.
   *
   * @param steering_angle_command Steering angle in radians
   */
  void send_steering_control(double steering_angle_command);

  /**
   * @brief Sends the throttle control command to CAN.
   *
   * @param throttle_value_ros Throttle value from ROS
   */
  void send_throttle_control(double throttle_value_ros);

  /**
   * @brief Sends an alive message from the AS CU to the master periodically.
   */
  void alive_msg_callback();

  /**
   * @brief Sets the angle origin of the Cubemars steering actuator.
   */
  void cubem_set_origin();

  /**
   * @brief Sets the current position as the Bosch steering angle origin (permanent)
   * @note This function is not currently used as it is meant to be used only once
   *
   * @return 0 on success, otherwise on failure
   */
  int bosch_steering_angle_set_origin();

  /**
   * @brief Publishes the BMS errors to ROS.
   *
   * @param msg CAN message data
   * @param dlc Data length code
   */
  void bms_errors_publisher(const unsigned char msg[8], unsigned int dlc);

  /**
   * @brief Publishes the APPs higher value to ROS.
   *
   * @param msg CAN message data
   */
  void apps_higher_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the APPs lower value to ROS.
   *
   * @param msg CAN message data
   */
  void apps_lower_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes the driving state to ROS.
   *
   * @param msg CAN message data
   */
  void driving_state_publisher(const unsigned char msg[8]);

  /**
   * @brief Publishes FSG Datalogger messages to CAN
   */
  void send_dv_driving_dynamics_1();
  void send_dv_driving_dynamics_2();
  void send_dv_system_status();
  void dv_messages_callback();

  /**
   * @brief Publishes cell temperature data to ROS.
   * 
   * @param msg CAN message data
   * @param stack_id Cell ID for which the temperature is being published
   */
  void process_cell_temperatures(const unsigned char msg[8], uint8_t stack_id, uint8_t msg_index, unsigned int dlc);

  /**
   * @brief Periodically publishes cell temperatures to ROS.
   * This function collects the cell temperatures from the internal array and publishes them.
   */
  void publish_cell_temperatures();

  /**
   * @brief Publishes the Bamocar errors to ROS.
   * @param msg CAN message data
   */
  void publish_bamocar_errors(const unsigned char msg[8]);

public:
  /**
   * @brief Contructor for the RosCan class
   */
  RosCan(std::shared_ptr<ICanLibWrapper> can_lib_wrapper_param);

  ~RosCan() override;

  friend class RosCanTest;
  FRIEND_TEST(RosCanTest, ControlCallback);
  FRIEND_TEST(RosCanTest, ControlCallbackLowerThrottle);
  FRIEND_TEST(RosCanTest, ControlCallbackHigherThrottle);
  FRIEND_TEST(RosCanTest, ControlCallbackLowerSteering);
  FRIEND_TEST(RosCanTest, ControlCallbackHigherSteering);
  FRIEND_TEST(RosCanTest, PublishControlCallback);
  FRIEND_TEST(RosCanTest, TestImuAccPublisher);
  FRIEND_TEST(RosCanTest, TestImuGyroPublisher);
  FRIEND_TEST(RosCanTest, TestCanInterpreterMasterStatusMission);
  FRIEND_TEST(RosCanTest, TestCanInterpreter_TEENSY_DASH_RR_RPM_CODE);
  FRIEND_TEST(RosCanTest, TestOutOfRangeUpperSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeLowerSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestOutOfRangeSingleSteeringThrottle);
  FRIEND_TEST(RosCanTest, TestCarStateMustBeDriving);
  FRIEND_TEST(RosCanTest, TestAliveMsgCallback);
};