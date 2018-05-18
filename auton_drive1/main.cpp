#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"

m3pi robot;
btbee bt;
Timer timer;

const float TURN_SCALE = 0.6; // .4
const float SLOW_SCALE = 0.2; // .1
const float MAX_SPEED = 0.25; // .15

int main() {
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3fbat");
    
    robot.sensor_auto_calibrate();
    
    float line_pos;
    
    while(1) {
        line_pos = robot.line_position();
        // line is to left - must turn left
        if(line_pos < 0) {
            robot.left_motor(MAX_SPEED + SLOW_SCALE * line_pos);
            robot.right_motor(MAX_SPEED - TURN_SCALE * line_pos);
        }
        // line is to right - must turn right
        else {
            robot.left_motor(MAX_SPEED + TURN_SCALE * line_pos);
            robot.right_motor(MAX_SPEED - SLOW_SCALE * line_pos);
        }
    }
}
