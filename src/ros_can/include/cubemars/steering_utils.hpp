#ifndef STEERING_UTILS
#define STEERING_UTILS

#include <cmath>

/**
 * @brief Creates buffer from steering angle
 * 
 * @param angle steering angle in radians
 * @param buffer steering angle in buffer in degrees * 10000
*/
void create_steering_angle_command(float angle, char *buffer) {
    float degree_angle = angle * 180 / M_PI;
    int converted_angle = static_cast<int>((degree_angle * 10000)); // Indicated by documentation
    for (int i = 3, j = 0; i >= 0; i--, j++) {
        buffer[i] = converted_angle >> (8 * j);
    }
}


#endif // STEERING_UTILS