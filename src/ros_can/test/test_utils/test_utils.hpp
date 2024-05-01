#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <fs_msgs/msg/control_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>


class RosCanTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        test_node_ = std::make_shared<rclcpp::Node>("test_node");

        mockCanLibWrapper = std::make_shared<MockCanLibWrapper>();
        rosCan = std::make_shared<RosCan>(mockCanLibWrapper);

        controlCommand = std::make_shared<fs_msgs::msg::ControlCommand>();
        controlCommand->throttle = 1.5;
        controlCommand->steering = 1.5;

        rosCan->setASDrivingState();
    }

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_string_publisher;
    rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr my_mission_finished_publisher;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_command_publisher;
    std::shared_ptr<MockCanLibWrapper> mockCanLibWrapper;
    std::shared_ptr<RosCan> rosCan;
    std::shared_ptr<fs_msgs::msg::ControlCommand> controlCommand;
};

MATCHER_P(PointeeAsChar, value, "") {
    return *(static_cast<unsigned char *>(arg)) == value;
}

MATCHER_P(PointeeAsDouble, value, "") {
    return *(static_cast<double *>(arg)) == value;
}

bool is_approx_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

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


MATCHER_P(PointeeAsAngleEqualTo, expected_angle, "") {
    float angle = get_angle_from_request_data(const_cast<void *>(arg));
    return is_approx_equal(angle, expected_angle, 0.0001);
}

ACTION_P(SetArg2ToUnsignedChar, value) {
    auto *dest = static_cast<unsigned char *>(arg2);
    std::copy(value, value + 8, dest);
}
