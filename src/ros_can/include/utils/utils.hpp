#pragma once
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

void create_steering_angle_command(float angle, char* buffer);
int transform_steering_angle_command(const double wheels_steering_angle,
                                     double& actuator_steering_angle);

int transform_steering_angle_reading(const double sensor_steering_angle,
                                     double& wheels_steering_angle);
float interpolate(const std::map<float, float>& look_up_table, float resistance);