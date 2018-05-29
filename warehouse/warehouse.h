#include "m3pi_ng.h"
#include "btbee.h"
#include <vector>
#include <string>
#include <deque>
#include <algorithm>

using namespace std;

const char S_ONE = '1';
const char S_TWO = '2';
const char S_THREE = '3';
const char S_FOUR = '4';
const char S_FIVE = '5';
const char S_SIX = '6';

struct Warehouse{
    
    // constructor
    Warehouse(m3pi &robot_in, btbee &bt_in);
    
    // initialize path_matrix
    void init_path_matrix();
    
    // reverse path : helper function for path_matrix;
    string reverse_path(const string &original);
    
    // add path to the q_path
    void add_path(int to);
    
    // get station path via bluetooth
    void get_station_path();
    
    void run();
    
    void turn_right();
    
    void turn_left();
    
    void go_forward();
    
    void turn_180();
    
    // check if it's end of the game
    bool is_end();
    
    // returns 
    bool is_started();
    
    // set from
    void set_from();
    bool is_to_received(); 
    void set_to();
    
    
private:
    m3pi robot;
    btbee bt;
    int from;
    bool make_turn;
    string path_matrix[6][6];
    const int BLACK_TOLERANCE;
    const float TURN_WAIT;
    const float TURN_SPEED;
    const float FORWARD_WAIT;
    deque<char> q_path;
    
    //for bluetooth
    char buffer[100];
    int buffer_len;
    int chars_read;
    bool started;
    bool to_received;
};
