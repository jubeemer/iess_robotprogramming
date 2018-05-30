#include "m3pi_ng.h"
#include <vector>
#include <cmath>

class RaceTracker {
public:
	// Constructor for a RaceTracker.
	// robot_in           : the robot which will be racing
	// race_num_laps_in   : number of laps in the race
	// black_tolerance_in : the tolerance band for what sensor values are 
	//                      considered "black"
	// white_tolerance_in : the tolerance band for what sensor values are
	//                      considered "white"
	RaceTracker(m3pi &robot_in, int race_num_laps_in,
		int black_tolerance_in, int white_tolerance_in);

	// Reads calibrated sensor values from the infrared sensors.
	void get_raw_sensors();

	// Returns true if the robot's front edge is over a completely black surface
	// (i.e., the start/finish line perpendicular to the track)
	bool is_black_surface();

	// Returns true if the robot's front edge is over a completely white surface
	// (i.e., off the track entirely).
	bool is_white_surface();

	// Print to the LCD display a summary that includes the number of laps traversed,
	// the time taken for each lap, and the total race time.
	void print_summary();

	// Print to the LCD display a summary of the total race time for the warehouse
	void print_summary(int ms);

	// Print a goodbye message to the user
	void print_exit();

	// Record the time of the previous lap and increment the lap counter.
	void record_lap();

	// Set the flag indicating whether the robot is currently over the start/finish line
	void set_line_flag(bool flag);

	// Returns line_flag
	bool get_line_flag();

	// Check whether the robot has traversed the required number of laps.
	bool is_finished();

	// Start the timer
	void start_timer();

	// check for intersections
	bool is_corner();

private:
	m3pi robot;
	Timer lap_timer;
	int race_num_laps;
	bool line_flag;
	int sensor_data[5];
	vector<int> lap_times;
	int num_laps;
	int total_time;

	const int BLACK_TOLERANCE, WHITE_TOLERANCE;
};

