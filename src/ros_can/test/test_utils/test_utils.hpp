#pragma once
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "utils/utils.hpp"
#include "custom_interfaces/msg/control_command.hpp"

/**
 * @class RosCanTest
 * @brief A test fixture for testing the RosCan class. Initializes the roscan and test node, a wrapper for the mock canlib and an example of a controlCommand.
 */
class RosCanTest : public ::testing::Test {
protected:
/**
     * @brief Set up the test fixture.
     * This function is called before each test is run.
     */
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        test_node_ = std::make_shared<rclcpp::Node>("test_node");

        mockCanLibWrapper = std::make_shared<MockCanLibWrapper>();
        rosCan = std::make_shared<RosCan>(mockCanLibWrapper);

        controlCommand = std::make_shared<custom_interfaces::msg::ControlCommand>();
        controlCommand->throttle = 0;
        controlCommand->steering = 0.1;

        rosCan->setASDrivingState();
    }
    /**
     * @brief Tear down the test fixture.
     * This function is called after each test is run.
     */
    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::map<std::string, std::string> topics = {{"emergency",        "/as_msgs/emergency"},
                                                 {"mission_finished", "/as_msgs/mission_finished"},
                                                 {"controls",         "/as_msgs/controls"},
                                                 {"status",           "operationalStatus"},
                                                 {"right_rear",       "rrRPM"}
    };

    std::shared_ptr<rclcpp::Node> test_node_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_string_publisher;
    // rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr my_mission_finished_publisher;
    rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr control_command_publisher;
    std::shared_ptr<MockCanLibWrapper> mockCanLibWrapper;
    std::shared_ptr<RosCan> rosCan;
    std::shared_ptr<custom_interfaces::msg::ControlCommand> controlCommand;
};
/**
 * @brief Matcher to compare the value pointed by a void pointer to a given unsigned char value.
 * @param value The expected unsigned char value.
 */
MATCHER_P(PointeeAsChar, value, "") {
    return *(static_cast<unsigned char *>(arg)) == value;
}
/**
 * @brief Matcher to compare the value pointed by a void pointer to a given double value.
 * @param value The expected double value.
 */
MATCHER_P(PointeeAsDouble, value, "") {
    return *(static_cast<double *>(arg)) == value;
}
/**
     * @brief Check if two doubles are approximately equal.
     * @param a The first double.
     * @param b The second double.
     * @param tolerance The tolerance for the comparison.
     * @return True if the doubles are approximately equal, false otherwise.
     */
bool is_approx_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}
/**
     * @brief Get the angle from void* passed to the function.
     * @param steering_requestData The request data void*.
     * @return The angle.
     */
float get_angle_from_request_data(void *steering_requestData) {
    char *buffer = static_cast<char *>(steering_requestData);
    int converted_angle = 0;
    for (int i = 0; i < 4; i++) {
        converted_angle |= (static_cast<unsigned char>(buffer[i]) << (8 * (3 - i)));
    }
    float degree_angle = static_cast<float>(converted_angle) / 10000;
    float angle = degree_angle * M_PI / 180;
    return angle;
}

/**
 * @brief Matcher to compare the angle obtained from a void pointer to a given expected angle.
 * @param expected_angle The expected angle in radians.
 */
MATCHER_P(PointeeAsAngleEqualTo, expected_angle, "") {
    float angle = get_angle_from_request_data(const_cast<void *>(arg));
    double comp_angle = 0;
    transform_steering_angle_command(expected_angle, comp_angle);
    //print comp and expected angle
    std::cout << "comp_angle: " << comp_angle << std::endl;
    std::cout << "expected_angle: " << angle << std::endl;
    return is_approx_equal(comp_angle, angle, 0.001);
}
/**
 * @brief Action to set the value pointed by the third argument (arg2) to a given unsigned char array.
 * @param value The unsigned char array to be copied.
 */
ACTION_P(SetArg2ToUnsignedChar, value) {
    auto *dest = static_cast<unsigned char *>(arg2);
    std::copy(value, value + 8, dest);
}
