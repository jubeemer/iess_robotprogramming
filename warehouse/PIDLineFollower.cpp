#include "PIDLineFollower.h"

// Constructor for a PIDLineFollower
// robot_in : m3pi object to be driver
// speed_in : default uncorrected speed of motors
// KP_in    : proportional term scalar
// KI_in    : integral term scalar
// KD_in    : derivative term scalar
PIDLineFollower::PIDLineFollower(m3pi &robot_in, float speed_in, float KP_in, float KI_in, float KD_in)
                            : robot(robot_in), KP(KP_in), KI(KI_in), KD(KD_in),
                            line_error(0.0), error_prev(0.0), integral(0.0), correction(0.0), 
                            BASE_SPEED(speed_in), LEFT_SPEED(0.0), RIGHT_SPEED(0.0) {}

// Drive robot with a PID loop with a time increment of dt.
void PIDLineFollower::drive(float dt) {
    update_error();
    calculate_correction(dt);
    check_errors();
    throttle();
}

// Save the previous error and read the new error from the sensors.
void PIDLineFollower::update_error() {
    error_prev = line_error;
    line_error = robot.line_position();
}

// Calculate the correction to be applied to each motor.
void PIDLineFollower::calculate_correction(float dt) {
    correction = KP*line_error + KI*(integral + line_error*dt) + KD*((line_error - error_prev) / dt);
    LEFT_SPEED = BASE_SPEED + correction;
    RIGHT_SPEED = BASE_SPEED - correction;
}

// Check for a bad motor command and adjust speeds if necessary.
void PIDLineFollower::check_errors() {
    if(LEFT_SPEED > 1) {
        LEFT_SPEED = 1;
    }
    else if(LEFT_SPEED < -1) {
        LEFT_SPEED = -1;
    }
    if(RIGHT_SPEED > 1) {
        RIGHT_SPEED = 1;
    }
    else if(RIGHT_SPEED < -1) {
        RIGHT_SPEED = -1;
    }
}

// Set the motors to the corrected speeds.
void PIDLineFollower::throttle() {
    robot.left_motor(LEFT_SPEED);
    robot.right_motor(RIGHT_SPEED);
}
