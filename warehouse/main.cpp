#include "mbed.h"
#include "m3pi_ng.h"
#include "PIDLineFollower.h"
#include "RaceTracker.h"
// #include "Music.h"
#include "Warehouse.h"

m3pi robot;
Timer timer;
btbee bt;
DigitalIn mypin(p21, PullUp);

const float dt = .008;
const int BLACK_TOLERANCE = 100;
const int WHITE_TOLERANCE = 400;

int main() {
    int count = 0;
    robot.cls();
    robot.sensor_auto_calibrate();
    // Music m;

    Warehouse w(robot, bt);
    PIDLineFollower controller(robot, .6, 0.70, 0, 0.012);
    RaceTracker race(robot, 2, BLACK_TOLERANCE, WHITE_TOLERANCE);

    timer.start();
    
    while (1) {
        // Get any waypoints from bluetooth serial
        w.get_station_path();

        // Process intersections in warehouse
        race.get_raw_sensors();
        if (race.is_corner() && !w.is_end()) {
            ++count;
            robot.cls();
            robot.locate(0, 0);
            robot.printf("%d", count);
            w.run();
        }

        // Press button to kill robot
        if (!mypin) {
            robot.stop();
            // m.play_fight_song_2(robot);
            break;
        }
        
        // Don't move is queue is empty
       if (w.is_end()) {
            robot.stop();
            continue;
        }

        // Wait until dt has passed to run the control algorithm
        if (timer.read() < dt) {
            continue;
        }

        // Line following algorithm
        controller.drive(dt);

        timer.reset();

    }
}
