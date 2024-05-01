#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fs_msgs/msg/control_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include "../include/node/node_ros_can.hpp"
#include "canlib_test_wrappers/mock_can_lib_wrapper.hpp"
#include "test_utils/test_utils.hpp"


 TEST_F(RosCanTest, ControlCallback) {
     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, PointeeAsAngleEqualTo(controlCommand->steering) , testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     long throttle_id = BAMO_RECEIVER;
     void *throttle_requestData = static_cast<void *>(&controlCommand->throttle);
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, throttle_requestData, testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     rosCan->test_control_callback(controlCommand);
 }

 TEST_F(RosCanTest, PublishControlCommand) {
     controlCommand->throttle = 0.5;
     controlCommand->steering = 0.5;
     control_command_publisher =
             test_node_->create_publisher<fs_msgs::msg::ControlCommand>(topics["controls"], 10);

     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, PointeeAsAngleEqualTo(controlCommand->steering),
                          testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     EXPECT_CALL(*mockCanLibWrapper,
                 canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
             .WillRepeatedly(testing::Return(canERR_NOMSG));

     long throttle_id = BAMO_RECEIVER;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, PointeeAsDouble(controlCommand->throttle),
                          testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     control_command_publisher->publish(*controlCommand);

     rclcpp::spin_some(rosCan);
 }

 TEST_F(RosCanTest, EmergencyCallback) {
     auto emergencyMsg = std_msgs::msg::String();
     emergencyMsg.data = "emergency";
     my_string_publisher =
             test_node_->create_publisher<std_msgs::msg::String>(topics["emergency"], 10);

     unsigned char expectedData = EMERGENCY_CODE;

     long can_id = AS_CU_NODE_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
             .WillRepeatedly(testing::Return(canERR_NOMSG));
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, can_id, PointeeAsChar(expectedData), testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     my_string_publisher->publish(emergencyMsg);

     rclcpp::spin_some(rosCan);
 }

 TEST_F(RosCanTest, MissionFinishedCallback) {
     auto missionFinishedMsg = fs_msgs::msg::FinishedSignal();
     my_mission_finished_publisher =
             test_node_->create_publisher<fs_msgs::msg::FinishedSignal>(topics["mission_finished"], 10);
     long can_id = AS_CU_NODE_ID;
     unsigned char expectedData = MISSION_FINISHED_CODE;
     EXPECT_CALL(*mockCanLibWrapper,
                 canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
             .WillRepeatedly(testing::Return(canERR_NOMSG));
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, can_id, PointeeAsChar(expectedData), testing::_, testing::_))
             .Times(1)
             .WillOnce(testing::Return(canOK));

     my_mission_finished_publisher->publish(missionFinishedMsg);

     rclcpp::spin_some(rosCan);
 }

 TEST_F(RosCanTest, TestImuYawAccYPublisher) {
     unsigned char msg[8] = {0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
     long id = IMU_YAW_RATE_ACC_Y_ID;
     unsigned int dlc = 8;
     unsigned int flag = 0;
     unsigned long time = 0;

     EXPECT_CALL(*mockCanLibWrapper,
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

     rosCan->testCanSniffer();

     rclcpp::spin_some(test_node_);
 }

 TEST_F(RosCanTest, TestCanInterpreterMasterStatusMission) {
     unsigned char msg[8] = {MASTER_AS_MISSION_CODE, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
     long id = MASTER_STATUS;
     unsigned int dlc = 8;
     unsigned int flag = 0;
     unsigned long time = 0;

     EXPECT_CALL(*mockCanLibWrapper,
                 canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
             .Times(2)
             .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                                      testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                                      testing::SetArgPointee<5>(time), testing::Return(canOK)))
             .WillRepeatedly(testing::Return(canERR_NOMSG));

     auto masterStatusSub = test_node_->create_subscription<custom_interfaces::msg::OperationalStatus>(
             topics["status"], 10, [&](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
                 EXPECT_EQ(msg->as_mission, 2);
                 EXPECT_EQ(msg->go_signal, 0);
             });

     rosCan->testCanSniffer();

     rclcpp::spin_some(test_node_);
 }

 TEST_F(RosCanTest, TestCanInterpreter_TEENSY_C1_RR_RPM_CODE) {
     unsigned char msg[8] = {TEENSY_C1_RR_RPM_CODE, 0x1F, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00};
     long id = TEENSY_C1;
     unsigned int dlc = 8;
     unsigned int flag = 0;
     unsigned long time = 0;

     EXPECT_CALL(*mockCanLibWrapper,
                 canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
             .Times(2)
             .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                                      testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                                      testing::SetArgPointee<5>(time), testing::Return(canOK)))
             .WillRepeatedly(testing::Return(canERR_NOMSG));

     auto rrRPMPubSub = test_node_->create_subscription<custom_interfaces::msg::WheelRPM>(
             topics["right_rear"], 10, [&](const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
                 double tolerance = 0.0001;
                 EXPECT_TRUE(is_approx_equal(msg->rr_rpm, 164.15, tolerance));
             });

     rosCan->testCanSniffer();

     rclcpp::spin_some(test_node_);
 }

 TEST_F(RosCanTest, TestOutOfRangeUpperSteeringThrottle) {
     controlCommand->throttle = STEERING_UPPER_LIMIT + 1;
     controlCommand->steering = STEERING_UPPER_LIMIT + 1;
     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
             .Times(0);

     long throttle_id = BAMO_RECEIVER;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
             .Times(0);

     rosCan->test_control_callback(controlCommand);
 }

 TEST_F(RosCanTest, TestOutOfRangeLowerSteeringThrottle) {
     controlCommand->throttle = STEERING_LOWER_LIMIT - 1;
     controlCommand->steering = STEERING_LOWER_LIMIT - 1;
     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
             .Times(0);

     long throttle_id = BAMO_RECEIVER;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
             .Times(0);

     rosCan->test_control_callback(controlCommand);
 }

 TEST_F(RosCanTest, TestOutOfRangeSingleSteeringThrottle) {
     controlCommand->throttle = STEERING_UPPER_LIMIT - 1;
     controlCommand->steering = STEERING_UPPER_LIMIT + 1;
     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
             .Times(0);

     long throttle_id = BAMO_RECEIVER;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
             .Times(0);

     rosCan->test_control_callback(controlCommand);
 }

 TEST_F(RosCanTest, TestCarStateMustBeDriving) {
     rosCan->setASOffState();
     long steering_id = STEERING_COMMAND_CUBEM_ID;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
             .Times(0);

     long throttle_id = BAMO_RECEIVER;
     EXPECT_CALL(*mockCanLibWrapper,
                 canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
             .Times(0);

     rosCan->test_control_callback(controlCommand);
 }

TEST_F(RosCanTest, TestAliveMsgCallback) {

    auto new_node = std::make_shared<RosCan>(mockCanLibWrapper);
    EXPECT_CALL(*mockCanLibWrapper, canWrite(testing::_, testing::_, testing::_, testing::_, testing::_))
            .Times(3)
            .WillRepeatedly(testing::Return(canOK));
    EXPECT_CALL(*mockCanLibWrapper,
                canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
            .WillRepeatedly(testing::Return(canERR_NOMSG));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(new_node);

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(395)) {
        executor.spin_some();
    }
}