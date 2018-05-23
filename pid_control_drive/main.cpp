#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"
#include <cmath>
#include <vector>

m3pi robot;
btbee bt;
Timer timer;
Timer lap_timer;
DigitalIn mypin(p21, PullUp);

//global variables
const float dt = .008;
const int BLACK_TOLERANCE = 100;
const int WHITE_TOLERANCE = 400;
const int MAX_NUM_LAPS = 5;

struct Lap{
    // default constructor
    Lap() : lap_counter(0), on_black(true), total_lap_time(0) {};
    
    // variables
    int lap_counter;
    bool on_black;
    int sensor_data[5];
    vector<int> vec_time;
    int total_lap_time;
    
    // functions
    bool is_black_surface(){
        for (int j = 0; j < 5; j++) {
            if (sensor_data[j] < 1000 - BLACK_TOLERANCE) return false;
        }
        return true;
    }
    bool is_white_surface(){
        for (int j = 0; j < 5; j++) {
            if (sensor_data[j] > WHITE_TOLERANCE) return false;
        }
        return true;
    }
    void print_laps(m3pi& robot){
        robot.cls();
        robot.locate(0,0); robot.printf("printing");
        robot.locate(0,1); robot.printf("lap time");
        wait(2);
        
        robot.cls();
        robot.locate(0,0); robot.printf("#laps: ");
        robot.locate(0,1); robot.printf("%d", (int)vec_time.size());
        wait(2);
        
        for(int i = 0; i < vec_time.size(); i++){
            robot.cls();
            robot.locate(0,0); robot.printf("lap");
            robot.locate(3,0); robot.printf("%d", i+1);
            robot.locate(0,1); robot.printf("%d", vec_time[i]);
            robot.locate(6,1); robot.printf("ms");
            wait(3);
        }
        
        robot.cls();
        robot.locate(0,0); robot.printf("total");
        robot.locate(0,1); robot.printf("time is");
        wait(1);
        
        robot.cls();
        robot.locate(0,0);
        robot.printf("%d", total_lap_time);
        robot.locate(6,0); robot.printf("ms");
        wait(3);
        
    }
    void print_exit(m3pi& robot){
        robot.cls();
        robot.locate(0,0); robot.printf("Done !!");
        wait(3);
        robot.cls();
        robot.locate(0,0); robot.printf("Bye...");
        wait(3);
    }
    void update_lap(int time){
        on_black = true;
        lap_counter++;
        vec_time.push_back(time);
        total_lap_time+=time;
    }
    
};

class PIDLineFollower {
public:
    PIDLineFollower(m3pi &robot_in, float speed_in, float KP_in, float KI_in, float KD_in)
    : robot(robot_in), KP(KP_in), KI(KI_in), KD(KD_in),
    line_error(0.0), error_prev(0.0), integral(0.0), derivative(0.0),
    correction(0.0), BASE_SPEED(speed_in), LEFT_SPEED(0.0), RIGHT_SPEED(0.0) {}

    void drive(float dt) {
        update_error();
        calculate_correction(dt);
        check_errors();
        throttle();
    }
    
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
    
    void throttle() {
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
    
    robot.sensor_auto_calibrate();
    
    //controller constructor
    PIDLineFollower controller(robot, .95, 1.1, 0, 0.035);
    // Lap constructor
    Lap L;
    
    // timer starts
    lap_timer.start();
    timer.start();
    
    while(1) {
        // QUIT IF NUM LAPS IS REACHED
        if(L.lap_counter == MAX_NUM_LAPS || !mypin){
            robot.stop();
            L.print_laps(robot);
            break;
        }
        
        // LAP COUNTER
        robot.calibrated_sensor(L.sensor_data);
        if (L.is_black_surface()) {
            if(!L.on_black){
                L.update_lap(lap_timer.read_ms());
                lap_timer.reset();
            }
        }
        else L.on_black = false;
        
        if(timer.read() < dt) {
            continue;
        }
        
        // CONTROLLER
        controller.drive(dt);
        
        timer.reset();
    }
    
    L.print_exit(robot);
}

