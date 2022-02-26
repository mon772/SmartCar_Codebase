#include "car_config.h"
#include "mode.h"

// ---------- Old Car 2 ----------
#ifdef car_xray_2

const int BASE_ANGLE = 785;  // 785
const int LEFT_MAX_ANGLE = 1400;  // 1350+20
const int RIGHT_MAX_ANGLE = 170;  // 220
const float TURNING_RATIO = 17.3;

const int MOTOR_FIXED_SPEED = 1005;

const int SOBEL_THRESHOLD = 130;
// const float PATH_PLAN_CUTOFF = 9.0;  // For DynamicSpeed2
// const float PATH_PLAN_CUTOFF = 9.0;  // For DynamicSpeed3
const float PATH_PLAN_CUTOFF = 12.0;  // For DynamicSpeed4

// const float PP01_CAR_HALF_WIDTH_SMALL = 10.5;  // For DynamicSpeed2
// const float PP01_CAR_HALF_WIDTH_LARGE = 10.5;  // For DynamicSpeed2
const float PP01_CAR_HALF_WIDTH_SMALL = 10.5;  // For DynamicSpeed3
const float PP01_CAR_HALF_WIDTH_LARGE = 14.5;  // For DynamicSpeed3
const float PP01_SWITCH_HEIGHT = -22.10;

const uint16_t DY04_BASE = 2000;
const uint16_t DY04_SCALE_1 = 12;
const uint16_t DY04_SCALE_2 = 5;
const uint16_t DY04_MAX = 3200;

// const float MOTOR_PID_P = 0.017;
// const float MOTOR_PID_I = 0.0035;
// const float MOTOR_PID_D = 0.006;

// const float MOTOR_PID_P = 0.021;
// const float MOTOR_PID_I = 0.006;
// const float MOTOR_PID_D = 0.024;

const float MOTOR_PID_P = 0.020;
const float MOTOR_PID_I = 0.010;  // 0.006
const float MOTOR_PID_D = 0.025;

#endif

// ---------- New Car ----------
#ifdef car_new

// const int BASE_ANGLE = 600;  // 785
// const int LEFT_MAX_ANGLE = 200;
// const int RIGHT_MAX_ANGLE = 1000;  // 210
const int BASE_ANGLE = 750; //750
const int LEFT_MAX_ANGLE = 300; //300
const int RIGHT_MAX_ANGLE = 1100;
const float TURNING_RATIO = 9.0;

const int MOTOR_FIXED_SPEED = 1005;

const int SOBEL_THRESHOLD = 130;
// const float PATH_PLAN_CUTOFF = 9.0;  // For DynamicSpeed2
const float PATH_PLAN_CUTOFF = 9.0;  // For DynamicSpeed3

// const float PP01_CAR_HALF_WIDTH_SMALL = 10.5;  // For DynamicSpeed2
// const float PP01_CAR_HALF_WIDTH_LARGE = 10.5;  // For DynamicSpeed2
const float PP01_CAR_HALF_WIDTH_SMALL = 10.5;  // For DynamicSpeed3
const float PP01_CAR_HALF_WIDTH_LARGE = 14.5;  // For DynamicSpeed3
const float PP01_SWITCH_HEIGHT = -22.09895;

const float DY01_SPEED_RATIO = 0.238237;
const uint16_t DY01_MAX_SPEED = 1015;
const uint16_t DY01_MIN_SPEED = 1002;
const float DY01_TURN_ANGLE_A = 30.0;
const float DY01_TURN_ANGLE_B = 45.0;
const float DY01_TURN_ANGLE_C = 60.0;
const uint16_t DY01_TURN_SPEED_A = 1010;
const uint16_t DY01_TURN_SPEED_B = 1007;
const uint16_t DY01_TURN_SPEED_C = 1005;

const uint16_t DY04_BASE = 2000;
const uint16_t DY04_SCALE_1 = 12;
const uint16_t DY04_SCALE_2 = 5;
const uint16_t DY04_MAX = 3200;

const float MOTOR_PID_P = 0.021;
const float MOTOR_PID_I = 0.014;
const float MOTOR_PID_D = 0.022;

#endif

// ---------- Simulation Car 160 ----------
#ifdef car_sim_v160

const int BASE_ANGLE = 900;
const int LEFT_MAX_ANGLE = 1475;
const int RIGHT_MAX_ANGLE = 325;
const float TURNING_RATIO = 17.0;

const int MOTOR_FIXED_SPEED = 1008;

const int SOBEL_THRESHOLD = 135;
const float PATH_PLAN_CUTOFF = 0.0;

const float PP01_CAR_HALF_WIDTH_SMALL = 18.5;
const float PP01_CAR_HALF_WIDTH_LARGE = 25.0;
const float PP01_SWITCH_HEIGHT = -22.09895;

const float DY01_SPEED_RATIO = 0.238237;
const uint16_t DY01_MAX_SPEED = 1030;
const uint16_t DY01_MIN_SPEED = 1009;
const float DY01_TURN_ANGLE_A = 30.0;
const float DY01_TURN_ANGLE_B = 45.0;
const float DY01_TURN_ANGLE_C = 60.0;
const uint16_t DY01_TURN_SPEED_A = 1018;
const uint16_t DY01_TURN_SPEED_B = 1013;
const uint16_t DY01_TURN_SPEED_C = 1010;

const float MOTOR_PID_P = 1.0;
const float MOTOR_PID_I = 1.0;
const float MOTOR_PID_D = 1.0;

#endif