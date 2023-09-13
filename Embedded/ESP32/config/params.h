#pragma once
// #include "pins.h"
// #include "datatype.h"

const float wheel_diameter = 67.2e-3f;                // 轮子的直径，单位 mm
const float wheel_radius   = wheel_diameter / 2.0f;   // 轮子的半径，单位 mm

#define MOTOR_L 0
#define MOTOR_R 1

#define ENCODER_L 0
#define ENCODER_R 1

#define PPR_L 1976                                               // Pulse per revolution on left encoder
#define PPR_R 1890                                               // Pulse per revolution on right encoder

#define MAX_SPEED_L 5.0f / 6.0f                                  // Max speed of the left motor (revolution per second)
#define MAX_SPEED_R 200.0f / 241.0f                              // Max speed of the right motor (revolution per second)

#define MAX_LINEAR_SPEED_L (MAX_SPEED_L * PI * wheel_diameter)   // Max linear speed of the robot (meter per second)
#define MAX_LINEAR_SPEED_R (MAX_SPEED_R * PI * wheel_diameter)   // Max linear speed of the robot (meter per second)

const float max_linear_speed_l = MAX_LINEAR_SPEED_L;             // Max linear speed of the robot (meter per second)
const float max_linear_speed_r = MAX_LINEAR_SPEED_R;             // Max linear speed of the robot (meter per second)

/*** The size of the oled screen ***/
const uint8_t oled_width  = 128;
const uint8_t oled_height = 64;
