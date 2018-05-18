#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <string>
#include <vector>

m3pi robot;
btbee bt;
Timer timer;

const float UPPER_THRESHOLD = 0.85;
const float MIDDLE_THRESHOLD = 0.4;
const int BLACK_TOLERANCE = 100; // 100
const int WHITE_TOLERANCE = 400;

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

void print_integer(int lap_counter, m3pi & robot) {
    robot.cls();

    vector<char> vec;
    while (lap_counter != 0) {
        char c = (lap_counter % 10) + '0';
        lap_counter /= 10;
        vec.push_back(c);
    }
    
    for (int j = vec.size() - 1; j >= 0; j--) {
        robot.printf(&vec[j]);
    }
}

bool is_black_surface(int sensor_data[5]) {
    for (int j = 0; j < 5; j++) {
        if (sensor_data[j] < 1000 - BLACK_TOLERANCE) 
            return false;
    }
    
    return true;
}

bool is_white_surface(int sensor_data[5]) {
    for (int j = 0; j < 5; j++) {
        if (sensor_data[j] > WHITE_TOLERANCE) 
            return false;
    }
    
    return true;
}


int main() {
    robot.cls();
        
    float bat = robot.battery();
    robot.locate(0, 0);
    robot.printf("%.3fbat");
    
    robot.sensor_auto_calibrate();
    
    float line_pos;

    // variables for lap time 
    int lap_counter = 0;
    bool on_black = true;
    int sensor_data[5];
    
    while(1) {
        // start timer here

        robot.calibrated_sensor(sensor_data);
        
        
        
        if (is_black_surface(sensor_data)) {
            if (!on_black) {
                on_black = true;
                lap_counter++;
                print_integer(lap_counter, robot);
            }
            // play music
        }
        else {
            on_black = false;
        }


        // move the robot
        if (is_white_surface(sensor_data)) {
            robot.backward(0.3);   
        }
        else {
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
        
}

