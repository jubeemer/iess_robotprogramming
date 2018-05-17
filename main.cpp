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
    
    while(1) {
        bt.read_all(buffer, buffer_len, &chars_read);
        for(int i = 0; i < chars_read; ++i) {
            // if(buffer[i] == '\xDA' && buffer[i+1] == '\xAD') {            
                switch(buffer[i]) {
                 // stop
                 case '\x00': robot.stop();
                              break;
                 // fwd
                 case '\x01': robot.forward(0.3);
                              break;
                 // backward
                 case '\x02': robot.backward(0.3);
                              break;
                 // left
                 case '\x04': robot.left(0.15);
                              break;
                 // left + fwd
                 case '\x05': robot.left_motor(0.2);
                              robot.right_motor(0.3);
                              break;
                 // left + back
                 case '\x06': robot.left_motor(-0.2);
                              robot.right_motor(-0.3);
                              break;
                 // right             
                 case '\x08': robot.right(0.15);
                              break;
                 // right + fwd
                 case '\x09': robot.left_motor(0.3);
                              robot.right_motor(0.2);
                              break;
                 // right + back
                 case '\x0a': robot.left_motor(-0.3);
                              robot.right_motor(-0.2);
                              break;
                 // default
                 default: robot.stop();
                          break;
                }
            //    i += 2;
            //}
        }
    }
}
