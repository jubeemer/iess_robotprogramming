#include "m3pi_ng.h"
#include "btbee.h"
#include <vector>
#include <string>
#include <deque>
#include <algorithm>

using namespace std;

class Warehouse {
public:
    // Constructor
    Warehouse(m3pi &robot_in, btbee &bt_in);
    
    // Initialize path matrix
    void init_path_matrix();
    
    // Helper function - reverses a path
    string reverse_path(const string &original);
    
    // Add path from -> to to the queue
    void add_path(int to);
    
    // Read from Serial, add necessary paths to the queue
    void get_station_path();
    
    // Handle an intersection
    void run();
    
    // Perform a right turn for TURN_WAIT ms
    void turn_right();
    
    // Perform a left turn for TURN_WAIT ms
    void turn_left();
    
    // Drive forward for FORWARD_WAIT ms
    void go_forward();
    
    // Turn 180 degrees.
    void turn_180();
    
    // Check if queue is empty.
    bool is_end();
    
private:
    m3pi robot;
    btbee bt;
    int from;
    string path_matrix[6][6];
    const float TURN_WAIT;
    const float TURN_SPEED;
    const float FORWARD_WAIT;
    const float FORWARD_SPEED;
    deque<char> q_path;
    
    //for bluetooth
    char buffer[100];
    int buffer_len;
    int chars_read;
};
