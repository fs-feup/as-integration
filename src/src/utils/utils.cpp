//
// Created by promao on 5/5/24.
//
#include "utils/utils.hpp"

#include <cmath>
#include <map>

#define AVERAGE_STEERING_RATIO 5.835  // Ratio between the steering angle at the wheels and the actuator

/**
 * @brief Creates buffer from steering angle for Cubemars
 * actuator position command
 *
 * @param angle steering angle in radians
 * @param buffer steering angle in buffer in degrees * 10000
 */
void create_steering_angle_command(float angle, char* buffer) {
  float degree_angle = angle * 180 / M_PI;
  int converted_angle = static_cast<int>((degree_angle * 10000));  // Indicated by documentation
  for (unsigned int i = 0; i < 4; i++) {
    buffer[i] = converted_angle >> (8 * (3 - i));
  }
}

/**
 * @brief Converts the steering angle command from wheels to
 * actuator position command
 *
 * @param double wheels_steering_angle steering angle in radians at the wheels
 * @param double actuator_steering_angle steering angle in radians for the actuator
 * @return int returns 0 if successful, 1 if error

*/
int transform_steering_angle_command(const double wheels_steering_angle,
                                     double& actuator_steering_angle) {
  actuator_steering_angle = wheels_steering_angle * AVERAGE_STEERING_RATIO;
  return 0;
}

/**
 * @brief Converts the steering angle from sensor to wheel angle
 *
 * @param double sensor_steering_angle steering angle in degrees from the sensor
 * @param double wheels_steering_angle steering angle in radians at the wheels
 * @return int returns 0 if successful, 1 if error
 */
int transform_steering_angle_reading(const double sensor_steering_angle,
                                     double& wheels_steering_angle) {
  double m = 0.203820182;
  double b = 5.1114e-16;

  wheels_steering_angle = (m * sensor_steering_angle + b);
  return 0;
}

float interpolate(const std::map<float, float>& look_up_table, float resistance) {
  using iterator = std::map<float, float>::const_iterator;
  iterator ub = look_up_table.upper_bound(resistance);
  if (ub == look_up_table.end()) {
    return (--ub)->second;
  }
  if (ub == look_up_table.begin()) {
    return ub->second;
  }
  iterator lb = ub;
  lb--;
  const float delta = (resistance - lb->first) / (ub->first - lb->first);
  return delta * ub->second + (1 - delta) * lb->second;  // std::lerp(lb->second, ub->second,
                                                         // delta);
}
