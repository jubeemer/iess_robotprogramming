#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"

m3pi robot;
btbee bt;
Timer timer;

const float UPPER_THRESHOLD = 0.85;
const float MIDDLE_THRESHOLD = 0.4;

float outer_turn_scale(double line) {
    if(line > UPPER_THRESHOLD) {
        return 0.5;
    }
    else if(line > MIDDLE_THRESHOLD) {
        return 0.45;
    }
    else {
        return 0.4;
    }
}

float inner_turn_scale(double line) {
    if(line > UPPER_THRESHOLD) {
        return 0.9;
    }
    else if(line > MIDDLE_THRESHOLD) {
        return 0.8;
    }
    else {
        return 0.7;
    }
}

float max_speed(double line) {
    if(line > UPPER_THRESHOLD) {
        return 0.5;
    }
    else if(line > MIDDLE_THRESHOLD) {
        return 0.55;
    }
    else {
        return 0.6;
    }
}

int main() {
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3fbat");
    
    robot.sensor_auto_calibrate();
    
    float line_pos;
    
    while(1) {
        line_pos = robot.line_position();
        if(line_pos < 0) {
            robot.left_motor(max_speed(line_pos) + inner_turn_scale(line_pos) * line_pos);
            robot.right_motor(max_speed(line_pos) - outer_turn_scale(line_pos) * line_pos);
        }
        else {
            robot.left_motor(max_speed(line_pos) + outer_turn_scale(line_pos) * line_pos);
            robot.right_motor(max_speed(line_pos) - inner_turn_scale(line_pos) * line_pos);
        }   
    }
        
}

