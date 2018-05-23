#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <cmath>

m3pi robot;
btbee bt;
Timer timer;

const float dt = .008;

class PIDLineFollower {
public:
    PIDLineFollower(m3pi &robot_in, float speed_in, float KP_in, float KI_in, float KD_in)
                : robot(robot_in), KP(KP_in), KI(KI_in), KD(KD_in),
                line_error(0.0), error_prev(0.0), integral(0.0), derivative(0.0),
                correction(0.0), BASE_SPEED(speed_in), LEFT_SPEED(0.0), RIGHT_SPEED(0.0) {}

    void update_error() {
        error_prev = line_error;
        line_error = robot.line_position();
    }

    void calculate_correction(float dt) {
        correction = KP*line_error + KI*(integral + line_error*dt) + KD*((line_error - error_prev) / dt);
        LEFT_SPEED = BASE_SPEED + correction;
        RIGHT_SPEED = BASE_SPEED - correction;
    }

    void check_errors() {
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

    void drive() {
        robot.left_motor(LEFT_SPEED);
        robot.right_motor(RIGHT_SPEED);
    }

private:
    m3pi robot;
    float KP;
    float KI;
    float KD;
    float line_error;
    float error_prev;
    float integral;
    float derivative;
    float correction;

    float BASE_SPEED;
    float LEFT_SPEED;
    float RIGHT_SPEED;
};

int main() {
    robot.cls();
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3f", bat);
    
    robot.sensor_auto_calibrate();

    PIDLineFollower controller(robot, 0.9, 1.0, 0, 0.025);
    timer.start();

    while(1) {        
        if(timer.read() < dt) {
            continue;
        }
        
        controller.update_error();
        controller.calculate_correction(dt);
        controller.check_errors();
        controller.drive();
        
        timer.reset();
    } 
}