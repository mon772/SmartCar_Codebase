#include "control_unit.h"

Movement::Movement() {

}

void Movement::update(float angle, float distance) {
    this->angle = angle;
    this->distance = distance;
}

// void Movement::speed() {
//     motor.SetDegree(1000);
// }

// void Movement::steer() {
//     angle = BASE_ANGLE + angle*57.296 //1 rad = 57.2957795131 degree
//     if (angel > LEFT_MAX_ANGLE)
//         angel = LEFT_MAX_ANGLE;
//     else if (angel < RIGHT_MAX_ANGLE)
//         angel = RIGHT_MAX_ANGLE;
//     servo.setDegree(angle);
// }
