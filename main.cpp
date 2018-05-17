#include "mbed.h"
#include "m3pi_ng.h"
#include "btbee.h"

m3pi robot;
btbee bt;
Timer timer;

int main() {
    char buffer[100];
    int buffer_len = 100;
    int chars_read = 0;
    
    float bat = robot.battery();
    robot.cls();
    robot.locate(0, 0);
    robot.printf("battery:");
    robot.locate(0, 1);
    robot.printf("%fbat");
    
    robot.sensor_auto_calibrate();
    
    int sensors[5];
    
    while(1) {
        robot.calibrated_sensor(sensors);
        int num = bt.read_all(buffer, buffer_len, &chars_read);
            
        robot.cls();
        robot.locate(0, 0);
        for(int i = 0; i < chars_read; ++i) {
            if(buffer[i] == '\xDA' && buffer[i+1] == '\xAD') {            
                switch(buffer[i+2]) {
                 // stop
                 case '\x00': robot.stop();
                              robot.printf("stop");
                              break;
                 // fwd
                 case '\x01': robot.forward(0.5);
                              robot.printf("fwd");
                              break;
                 // backward
                 case '\x02': robot.backward(0.5);
                              robot.printf("backward");
                              break;
                 // left
                 case '\x04': robot.printf("left");
                              robot.left(0.1);
                              break;
                 // left + fwd
                 case '\x05': robot.printf("l + f");
                              robot.left_motor(0.25);
                              robot.right_motor(.3);
                              break;
                 // left + back
                 case '\x06': robot.printf("l + b");
                              robot.left_motor(-0.25);
                              robot.right_motor(-.3);
                              break;
                 // right             
                 case '\x08': robot.printf("right");
                              robot.right(0.1);
                              break;
                 // right + fwd
                 case '\x09': robot.printf("r + f");
                              robot.left_motor(.3);
                              robot.right_motor(0.25);
                              break;
                 // right + back
                 case '\x0a': robot.printf("r + b");
                              robot.left_motor(-.3);
                              robot.right_motor(-0.25);
                              break;
                 // default
                default: robot.printf("stop");
                         robot.stop();
                         break;
                }
                i += 2;
            }
        }
        wait(1);
    }
}
