#pragma once
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

void create_steering_angle_command(float angle, char* buffer);
int transform_steering_angle_command(const double wheels_steering_angle, double &actuator_steering_angle);