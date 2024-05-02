#ifndef STEERING_UTILS
#define STEERING_UTILS

#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Creates buffer from steering angle for Cubemars
 * actuator position command
 * 
 * @param angle steering angle in radians
 * @param buffer steering angle in buffer in degrees * 10000
*/
void create_steering_angle_command(float angle, char* buffer) {
    float degree_angle = angle * 180 / M_PI;
    int converted_angle = static_cast<int>((degree_angle * 10000)); // Indicated by documentation
    for (unsigned int i = 0; i < 4; i++) {
        buffer[i] = converted_angle >> (8 * (3 - i));
    }
}

// #define LOG(message, level) \
//     do { \
//         rclcpp::Logger logger = rclcpp::get_logger("ros_can"); \
//         std::string complete_message = __FUNCTION__ + ":" + message; \
//         logger.log(level, complete_message); \
//     } while(0)

#endif // STEERING_UTILS_HPP