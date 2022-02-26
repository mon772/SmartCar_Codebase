#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <iostream>

extern const int BASE_ANGLE;
extern const int LEFT_MAX_ANGLE;
extern const int RIGHT_MAX_ANGLE;
extern const float TURNING_RATIO;

extern const int MOTOR_FIXED_SPEED;

extern const int SOBEL_THRESHOLD;  // Threshold of sobel operator
extern const float PATH_PLAN_CUTOFF;  // ignore some bottom part of edge (y-coord)

// Parameters for ComputerVision::pathPlanning()
extern const float PP01_CAR_HALF_WIDTH_SMALL;  // 9.5
extern const float PP01_CAR_HALF_WIDTH_LARGE;  // 17
extern const float PP01_SWITCH_HEIGHT;  // bottom - 6*track_width/4

extern const float DY01_SPEED_RATIO;
extern const uint16_t DY01_MAX_SPEED;
extern const uint16_t DY01_MIN_SPEED;
extern const float DY01_TURN_ANGLE_A;
extern const float DY01_TURN_ANGLE_B;
extern const float DY01_TURN_ANGLE_C;
extern const uint16_t DY01_TURN_SPEED_A;
extern const uint16_t DY01_TURN_SPEED_B;
extern const uint16_t DY01_TURN_SPEED_C;

extern const uint16_t DY04_BASE;
extern const uint16_t DY04_SCALE_1;
extern const uint16_t DY04_SCALE_2;
extern const uint16_t DY04_MAX;

extern const float MOTOR_PID_P;
extern const float MOTOR_PID_I;
extern const float MOTOR_PID_D;

#endif