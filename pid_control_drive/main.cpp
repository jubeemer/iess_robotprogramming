#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <cmath>

// INITIALIZATION
m3pi robot;
btbee bt;
Timer timer;

float t0, t1;
float dt = .001;

float BASE_SPEED = 0.8;
float OUTER_MOTOR_SPEED = BASE_SPEED;
float INNER_MOTOR_SPEED;

float line_error = 0;
float error_prev = 0;

float integral = 0;
float derivative = 0;
float correction = 0;

// CLOSED-LOOP CONTROL PARAMETERS
float KP = 1.2;
float KI = 0;
float KD = 0.015;

int main() {
    timer.start();
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3fbat");
    
    robot.sensor_auto_calibrate();
    
    while(1) {
        // START TIME
        t0 = timer.read_ms();
        
        // CALCULATE ERROR TERMS
        line_error = robot.line_position();
        integral += (line_error * dt);
        derivative = (line_error - error_prev) / dt;
        
        // ERROR ADJUSTMENT
        correction = abs(KP*line_error + KI*integral + KD*derivative);
        INNER_MOTOR_SPEED = OUTER_MOTOR_SPEED - correction;
        
        // CHECK FOR BAD COMMANDS
        if(INNER_MOTOR_SPEED > 1) {
            INNER_MOTOR_SPEED = 1;
        }
        if(INNER_MOTOR_SPEED < -1) {
            INNER_MOTOR_SPEED = -1;
        }
        
        // SET MOTORS
        if(line_error < 0) {
            robot.left_motor(INNER_MOTOR_SPEED);
            robot.right_motor(OUTER_MOTOR_SPEED);
        }
        else {
            robot.left_motor(OUTER_MOTOR_SPEED);
            robot.right_motor(INNER_MOTOR_SPEED);
        }
        
        // UPDATE ERROR
        error_prev = line_error;
        
        // NECESSARY WAIT
        wait_ms(1);
        
        // END TIME
        t1 = timer.read_ms();
        dt = (t1 - t0) / 1000.0;  
    }
            
        
}