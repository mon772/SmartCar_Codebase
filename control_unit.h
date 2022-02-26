#ifndef CONTROL_UNIT_H_
#define CONTROL_UNIT_H_

#include <iostream>

class Movement {
    private:
        const float MOTOR_PID_P = 1;
        const float MOTOR_PID_I = 1;
        const float MOTOR_PID_D = 1;

        const int BASE_ANGLE = 900;
        const int LEFT_MAX_ANGLE = 1200; 
        const int RIGHT_MAX_ANGLE = 600; 
         
        float angle, distance;
    public:
        Movement();

        /**
         * get new angle and distance value from computer_vision.cpp
         * @param angle steering angle (in radian)
         * @param distance distance of the path (in pixel after perspective transformation)
        */
        void update(float angle, float distance);

        /**
         * uses the distance to calculate required speed and then changes the motor speed
        */
        void speed();

        /**
         * converts angle into servo angle and then changes the servo angle
        */
        void steer();
};

#endif /* CONTROL_UNIT_H_ */
