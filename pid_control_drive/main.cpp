#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <cmath>

m3pi robot;
btbee bt;
Timer timer;

double error1 = 0;
double error_prior = 0;
double integral = 0;
double derivative = 0;
double iteration_time = .0005;
float output_speed;
float correction;
double KP = .65;
double KI = 0.02;
double KD = 0.0015;

// speed = .4 
// kp = .19
// ki = 0 
// kd = .0004

int main() {
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3fbat");
    
    robot.sensor_auto_calibrate();
    
    float line_pos;
    float OUTER_MOTOR_SPEED, INNER_MOTOR_SPEED;
    
    while(1) {
        line_pos = robot.line_position();
                
        error1 = line_pos;
        integral += (error1 * iteration_time);
        derivative = (error1 - error_prior) / iteration_time;
        
        correction = abs(KP*error1 + KI*integral + KD*derivative);
        output_speed = -correction + 0.8; 
        error_prior = error1;
        
        OUTER_MOTOR_SPEED = output_speed + correction;
        INNER_MOTOR_SPEED = output_speed - correction;
        
        if(OUTER_MOTOR_SPEED > 1) {
            OUTER_MOTOR_SPEED = 1;
        }
        if(INNER_MOTOR_SPEED > 1) {
            INNER_MOTOR_SPEED = 1;
        }
        
        if(line_pos < 0) {
            robot.left_motor(INNER_MOTOR_SPEED);
            robot.right_motor(OUTER_MOTOR_SPEED);
        }
        else {
            robot.left_motor(OUTER_MOTOR_SPEED);
            robot.right_motor(INNER_MOTOR_SPEED);
        } 
        wait_ms(.5);  
    }
            
        
}