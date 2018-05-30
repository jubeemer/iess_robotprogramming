#include "mbed.h"
#include "m3pi_ng.h"
#include "PIDLineFollower.h"
#include "RaceTracker.h"
#include "Music.h"
#include "Warehouse.h"

m3pi robot;
Timer timer;
btbee bt;
DigitalIn mypin(p21, PullUp);

const float dt = .008;
const int BLACK_TOLERANCE = 100;
const int WHITE_TOLERANCE = 400;

int main() {
	robot.cls();
	robot.sensor_auto_calibrate();
	Music m;

	Warehouse w(robot, bt);
	PIDLineFollower controller(robot, .4, 0.9, 0, 0);
	RaceTracker race(robot, 2, BLACK_TOLERANCE, WHITE_TOLERANCE);

	timer.start();
	race.start_timer();

	// Read in starting location from bluetooth
	while (!w.is_started()) {
		w.set_from();
	}

	// Read in first destination from bluetooth
	while (!w.is_to_received()) {
		w.set_to();
	}

	while (1) {
		// Get path from bluetooth
		w.get_station_path();

		// Warehouse
		race.get_raw_sensors();
		if (race.is_corner() && !w.is_end()) {
			w.run();
		}

		// Press button to kill robot
		if (!mypin) {
			robot.stop();
			race.print_summary();
			m.play_fight_song_2(robot);
			break;
		}

		if (w.is_end()) {
			robot.stop();
			continue;
		}

		// Wait until dt has passed to run the control algorithm
		if (timer.read() < dt) {
			continue;
		}

		controller.drive(dt);

		timer.reset();

	}
}