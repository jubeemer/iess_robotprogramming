#include "RaceTracker.h"

// Constructor for a RaceTracker.
// robot_in           : the robot which will be racing
// race_num_laps_in   : number of laps in the race
// black_tolerance_in : the tolerance band for what sensor values are 
//                      considered "black"
// white_tolerance_in : the tolerance band for what sensor values are
//                      considered "white"
RaceTracker::RaceTracker(m3pi &robot_in, int race_num_laps_in,
    int black_tolerance_in, int white_tolerance_in)
    : robot(robot_in), race_num_laps(race_num_laps_in), line_flag(true),
    num_laps(0), total_time(0), BLACK_TOLERANCE(black_tolerance_in),
    WHITE_TOLERANCE(white_tolerance_in) {}

// Reads calibrated sensor values from the infrared sensors.
void RaceTracker::get_raw_sensors() {
    robot.calibrated_sensor(sensor_data);
}

// Returns true if the robot's front edge is over a completely black surface
// (i.e., the start/finish line perpendicular to the track)
bool RaceTracker::is_black_surface() {
    for (int j = 0; j < 5; j++) {
        if (sensor_data[j] < 1000 - BLACK_TOLERANCE) {
            return false;
        }
    }
    return true;
}

// Returns true if the robot's front edge is over a completely white surface
// (i.e., off the track entirely).
bool RaceTracker::is_white_surface() {
    for (int j = 0; j < 5; j++) {
        if (sensor_data[j] > WHITE_TOLERANCE) {
            return false;
        }
    }
    return true;
}

// Print to the LCD display a summary that includes the number of laps traversed,
// the time taken for each lap, and the total race time.
void RaceTracker::print_summary() {
    robot.cls();
    robot.locate(0, 0); robot.printf("printing");
    robot.locate(0, 1); robot.printf("lap time");
    wait(0.5);

    robot.cls();
    robot.locate(0, 0); robot.printf("#laps: ");
    robot.locate(0, 1); robot.printf("%d", (int)lap_times.size());
    wait(1);

    for (int i = 0; i < lap_times.size(); i++) {
        robot.cls();
        robot.locate(0, 0); robot.printf("lap");
        robot.locate(3, 0); robot.printf("%d", i + 1);
        robot.locate(0, 1); robot.printf("%d", lap_times[i]);
        robot.locate(6, 1); robot.printf("ms");
        wait(1);
    }

    robot.cls();
    robot.locate(0, 0); robot.printf("total");
    robot.locate(0, 1); robot.printf("time is");
    wait(0.5);

    robot.cls();
    robot.locate(0, 0);
    robot.printf("%d", total_time);
    robot.locate(6, 0); robot.printf("ms");
    wait(2);
}

void RaceTracker::print_summary(int ms) {
    robot.cls();
    robot.locate(0, 0);
    robot.printf("%d", ms);
}

// Print a goodbye message to the user
void RaceTracker::print_exit() {
    robot.cls();
    robot.locate(0, 0); robot.printf("Done !!");
    wait(3);
    robot.cls();
    robot.locate(0, 0); robot.printf("Bye...");
    wait(3);
}

// Record the time of the previous lap and increment the lap counter.
void RaceTracker::record_lap() {
    int time = lap_timer.read_ms();
    lap_times.push_back(time);
    total_time += time;
    lap_timer.reset();
    ++num_laps;
}

// Set the flag indicating whether the robot is currently over the start/finish line
void RaceTracker::set_line_flag(bool flag) {
    line_flag = flag;
}

// Returns line_flag
bool RaceTracker::get_line_flag() {
    return line_flag;
}

// Check whether the robot has traversed the required number of laps.
bool RaceTracker::is_finished() {
    return num_laps >= race_num_laps;
}

// Start the timer
void RaceTracker::start_timer() {
    lap_timer.start();
}


//check for intersections
bool RaceTracker::is_corner() {
    return (sensor_data[0] > 800 && sensor_data[1] > 800 && sensor_data[2] > 800)
        || (sensor_data[2] > 800 && sensor_data[3] > 800 && sensor_data[4] > 800);
}
