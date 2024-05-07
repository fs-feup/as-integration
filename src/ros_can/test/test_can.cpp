#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "canlib_test_wrappers/mock_can_lib_wrapper.hpp"
#include "node/node_ros_can.hpp"
#include "test_utils/test_utils.hpp"
#include "utils/constants.hpp"

/**
 * @test This test case checks if the control callback function correctly writes the steering and
 * throttle commands to the CAN bus.
 */
TEST_F(RosCanTest, ControlCallback) {
  prepare_out_of_range_values(0.3, 0.3, 1);
    ros_can_->control_callback(control_command_);
}

/**
 * @test This test publishes a command with the ROSCAN node initialized, then we test if the
 * behaviour of the nodes is correct, activate the control callback and correctly write to can the
 * received values.
 */
TEST_F(RosCanTest, PublishControlCommand) {
  control_command_->throttle = 0.5;
  control_command_->steering = 0.1;
  control_command_publisher_ =
      test_node_->create_publisher<custom_interfaces::msg::ControlCommand>(topics_["controls"], 10);

  long steering_id = STEERING_COMMAND_CUBEM_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, steering_id, PointeeAsAngleEqualTo(control_command_->steering),
                       testing::_, testing::_))
      .Times(1)
      .WillOnce(testing::Return(canOK));

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  long throttle_id = BAMO_COMMAND_ID;
  EXPECT_CALL(
      *mock_can_lib_wrapper_,
      canWrite(testing::_, throttle_id, PointeeAsThrottleValueEqualTo(control_command_->throttle),
               testing::_, testing::_))
      .Times(1)
      .WillOnce(testing::Return(canOK));

  control_command_publisher_->publish(*control_command_);

  rclcpp::spin_some(ros_can_);
}

TEST_F(RosCanTest, EmergencyCallback) { test_service_call("/as_srv/emergency", EMERGENCY_CODE); }

TEST_F(RosCanTest, MissionFinishedCallback) {
  test_service_call("/as_srv/mission_finished", MISSION_FINISHED_CODE);
}

/**
 * @test This test checks if the IMU acceleration y readings from can are correctly published to the
 * correct ROS TOPIC, the test node is used to subscribe to the respective topic and read the values
 * and verify them.
 */
TEST_F(RosCanTest, TestImuYawAccYPublisher) {
  unsigned char msg[8] = {0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  long id = IMU_YAW_RATE_ACC_Y_ID;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  auto imuYawAccYSub = test_node_->create_subscription<custom_interfaces::msg::ImuData>(
      "imuYawAccY", 10, [&](const custom_interfaces::msg::ImuData::SharedPtr msg) {
        double tolerance = 0.00000001;

        EXPECT_TRUE(is_approx_equal(msg->gyro, 1.0 * QUANTIZATION_GYRO, tolerance));
        EXPECT_TRUE(is_approx_equal(msg->acc, 2.0 * QUANTIZATION_ACC, tolerance));
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}
/**
 * @test This test checks if the operational status callback function is called after receiving an
 * operational status message from ROS and if the the callback function correctly sends the
 * operational status signal to can.
 */
TEST_F(RosCanTest, TestCanInterpreterMasterStatusMission) {
  unsigned char msg[8] = {MASTER_AS_MISSION_CODE, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  long id = MASTER_STATUS;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  auto masterStatusSub = test_node_->create_subscription<custom_interfaces::msg::OperationalStatus>(
      topics_["status"], 10, [&](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
        EXPECT_EQ(msg->as_mission, 2);
        EXPECT_EQ(msg->go_signal, 0);
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}
/**
 * @test This test case checks if after receiving RR_RPM from the CAN bus, the values are correctly
 * published to the correct ROS TOPIC, the test node is used to subscribe to the respective topic
 * and read the values published and verify them
 */
TEST_F(RosCanTest, TestCanInterpreter_TEENSY_C1_RR_RPM_CODE) {
  unsigned char msg[8] = {TEENSY_C1_RR_RPM_CODE, 0x1F, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00};
  long id = TEENSY_C1;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  auto rr_rpm_pub_sub = test_node_->create_subscription<custom_interfaces::msg::WheelRPM>(
      topics_["right_rear"], 10, [&](const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
        double tolerance = 0.0001;
        EXPECT_TRUE(is_approx_equal(msg->rr_rpm, 164.15, tolerance));
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}

TEST_F(RosCanTest, TestOutOfRangeUpperSteeringThrottle) {
  prepare_out_of_range_values(STEERING_UPPER_LIMIT + 1, STEERING_UPPER_LIMIT + 1, 0);
    ros_can_->control_callback(control_command_);
}

TEST_F(RosCanTest, TestOutOfRangeLowerSteeringThrottle) {
  prepare_out_of_range_values(STEERING_LOWER_LIMIT - 1, STEERING_LOWER_LIMIT - 1, 0);
    ros_can_->control_callback(control_command_);
}

TEST_F(RosCanTest, TestOutOfRangeSingleSteeringThrottle) {
  prepare_out_of_range_values(STEERING_UPPER_LIMIT - 1, STEERING_UPPER_LIMIT + 1, 0);
    ros_can_->control_callback(control_command_);
}
/**
 * @test This test case confirms that the car state must be driving to send the steering and
 * throttle commands to the CAN bus.
 */
TEST_F(RosCanTest, TestCarStateMustBeDriving) {
    ros_can_->set_as_off_state();
  long steering_id = STEERING_COMMAND_CUBEM_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
      .Times(0);

  long throttle_id = BAMO_COMMAND_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
      .Times(0);

    ros_can_->control_callback(control_command_);
}
/**
 * @test This test case confirms that the alive message is sent to the CAN bus every 100ms.
 */
TEST_F(RosCanTest, TestAliveMsgCallback) {
  auto new_node = std::make_shared<RosCan>(mock_can_lib_wrapper_);
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(3)
      .WillRepeatedly(testing::Return(canOK));
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return(canERR_NOMSG));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(new_node);

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(395)) {
    executor.spin_some();
  }
}

/**
 * @test This test case checks if the the method correctly changes the wheels steering angle to the
 * actuator steering angle.
 */
TEST(UtilsTest, TransformSteeringAngleCommand) {
  double actuator_steering_angle = 0.0;

  double wheels_steering_angle = 0.1;
  double expected_actuator_steering_angle = 0.50677179486251434;

  int result = transform_steering_angle_command(wheels_steering_angle, actuator_steering_angle);
  EXPECT_EQ(result, 0);
  EXPECT_DOUBLE_EQ(actuator_steering_angle, expected_actuator_steering_angle);
}
