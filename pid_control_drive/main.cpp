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
    RaceTracker tracker(robot, 5, BLACK_TOLERANCE, WHITE_TOLERANCE);
    
    timer.start();
    
    while(1) {
        // Quit and print race summary if button is pressed or the race 
        // is completed.
        if(tracker.is_finished() || !mypin){
            robot.stop();
            tracker.print_summary();
            break;
        }
        
        // Lap counter
        tracker.get_raw_sensors();
        if(tracker.is_black_surface()) {
            if(!tracker.get_line_flag()) {
                tracker.record_lap();
                tracker.set_line_flag(true);
            }
        }
        else {
            tracker.set_line_flag(false);
        }
        
        // Wait until dt has passed to run the control algorithm
        if(timer.read() < dt) {
            continue;
        }
        
        controller.drive(dt);
        timer.reset();
    }
    
    tracker.print_exit();
}
