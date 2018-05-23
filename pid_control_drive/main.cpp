#include "mbed.h"
#include "m3pi_ng.h"
#include "PIDLineFollower.h"
#include "RaceTracker.h"

m3pi robot;
Timer timer;
DigitalIn mypin(p21, PullUp);

const float dt = .008;
const int BLACK_TOLERANCE = 100;
const int WHITE_TOLERANCE = 400;

int main() {
    robot.cls();
    robot.sensor_auto_calibrate();
    
    PIDLineFollower controller(robot, .95, 1.1, 0, 0.035);
    RaceTracker race(robot, 5, BLACK_TOLERANCE, WHITE_TOLERANCE);
    
    timer.start();
    
    while(1) {
        // Quit and print race summary if button is pressed or the race 
        // is completed.
        if(race.is_finished() || !mypin){
            robot.stop();
            race.print_summary();
            break;
        }
        
        // Lap counter
        race.get_raw_sensors();
        if(race.is_black_surface()) {
            if(!race.get_line_flag()) {
                race.record_lap();
                race.set_line_flag(true);
            }
        }
        else {
            race.set_line_flag(false);
        }
        
        // Wait until dt has passed to run the control algorithm
        if(timer.read() < dt) {
            continue;
        }
        
        controller.drive(dt);
        timer.reset();
    }
    
    race.print_exit();
}
