#include "mbed.h"
#include "m3pi_ng.h"
#include "PIDLineFollower.h"
#include "RaceTracker.h"
#include "Music.h"
#include "warehouse.h"

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
    Music M;
    // read in starting location from bluetooth
    
    Warehouse W (robot, bt);
    PIDLineFollower controller(robot, .4, 0.9, 0, 0);
    RaceTracker race(robot, 2, BLACK_TOLERANCE, WHITE_TOLERANCE);
    //M.play_mario(robot);
    timer.start();
    race.start_timer();
    
    while(!W.is_started()){
        W.set_from();
    }
    
    while(!W.is_to_received()){
        W.set_to();
    }
    
    //read in starting station
    
    while(1) {
        // Get path from bluetooth
        W.get_station_path();
        
        // Quit and print race summary if button is pressed or the race 
        // is completed.
        
        // Warehouse
        race.get_raw_sensors();
        if(race.is_corner() && !W.is_end()){
            W.run();
        }
        
        if(race.is_finished() || !mypin){
            robot.stop();
            race.print_summary();
            M.play_fight_song_2(robot);
            break;
        }
        
        if(W.is_end()){
            robot.stop();
        }
        else {
            controller.drive(dt);
        }

        
        // Lap counter
        /*
        race.get_raw_sensors();
        if(race.is_black_surface()) {
            if(!race.get_line_flag()) {
                race.record_lap();
                race.set_line_flag(true);
            }
        }
        else {
            race.set_line_flag(false);
        }*/
        
        // Wait until dt has passed to run the control algorithm
        if(timer.read() < dt) {
            continue;
        }
        
        timer.reset();
        
    }
    
    //race.print_exit();
}
