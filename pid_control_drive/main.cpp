#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <cmath>

// INITIALIZATION
m3pi robot;
btbee bt;
Timer timer;

const float dt = .008;
bool first_loop = 1;

const float BASE_SPEED = 0.9;
float LEFT_SPEED, RIGHT_SPEED;

float line_error = 0;
float error_prev = 0;

float integral = 0;
float derivative = 0;
float correction = 0;

// CLOSED-LOOP CONTROL PARAMETERS
float KP = 1.0;
float KI = 0;
float KD = 0.025;

int main() {
    timer.start();
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3f", bat);
    
    robot.sensor_auto_calibrate();
    
    while(1) {
        if(timer.read() < dt) {
            continue;
        }
        
        // CALCULATE ERROR TERMS
        line_error = robot.line_position();
        integral += (line_error * dt);
        derivative = (line_error - error_prev) / dt;
        
        // ERROR ADJUSTMENT
        correction = KP*line_error + KI*integral + KD*derivative;
        LEFT_SPEED = BASE_SPEED + correction;
        RIGHT_SPEED = BASE_SPEED - correction;
        
        // CHECK BAD COMMANDS
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

        // SET MOTORS
        robot.left_motor(LEFT_SPEED);
        robot.right_motor(RIGHT_SPEED);
        
        // UPDATE ERROR
        error_prev = line_error;
        
        // NECESSARY WAIT
        wait_ms(1);
        
        // RESET TIMER
        timer.reset();
    }
}