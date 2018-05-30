//
//  warehouse.cpp
//  warehouse
//
//  Created by Sabina Kim on 5/28/18.
//  Copyright © 2018 Sabina Kim. All rights reserved.
//

#include "Warehouse.h"

// constructor
Warehouse::Warehouse(m3pi &robot_in, btbee &bt_in): 
robot(robot_in), bt(bt_in), 
BLACK_TOLERANCE(900), TURN_WAIT(80), TURN_SPEED(1), FORWARD_WAIT(300),
buffer_len(100), chars_read(0), started(false), to_received(false) {
    init_path_matrix();
    }

void Warehouse::init_path_matrix(){
    path_matrix[0][1] = "RLLR";
    path_matrix[0][2] = "RLFF";
    path_matrix[0][3] = "RLFRL";
    path_matrix[0][4] = "RRLFLRRRL";
    path_matrix[0][5] = "RRLR";
    
    path_matrix[1][2] = "LLF";
    path_matrix[1][3] = "LLRL";
    path_matrix[1][4] = "LFRLRRL";
    path_matrix[1][5] = "LRFLR";
    
    path_matrix[2][3] = "LL";
    path_matrix[2][4] = "FLRLRRL";
    path_matrix[2][5] = "FFFLR";
    
    path_matrix[3][4] = "FFRL";
    path_matrix[3][5] = "FRLRL";
    
    path_matrix[4][5] = "RLLLRL";
    
    //reverse 
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < i; j++){
            path_matrix[i][j] = reverse_path(path_matrix[j][i]);
        }
    }
    
}
void Warehouse::run(){
    
    switch(q_path.front()){
        case 'L':
            turn_left();
        break;
        case 'R':
            turn_right();
        break;
        case 'F':
            go_forward();
        break;
        case 'D':
            turn_180();
        break; 
    }
    q_path.pop_front();
}

string Warehouse::reverse_path(const string &original){
    
    string r_path = original;
    reverse(r_path.begin(), r_path.end());
    

    for(int i = 0; i < r_path.size(); ++i){
        if(r_path[i] == 'L'){
            r_path[i] = 'R';
        }
        else if(r_path[i] == 'R'){
            r_path[i] = 'L';
        }
    }
    return r_path;
}

void Warehouse::add_path(int to){
    string str = path_matrix[from][to];
    for(int i = 0; i < str.size(); i++){
        q_path.push_back(str[i]);
    }
    q_path.push_back('D');
    from = to;
}

void Warehouse::turn_right(){
    robot.right_motor(0);
    robot.left_motor(TURN_SPEED);
    wait_ms(TURN_WAIT);
}


void Warehouse::turn_left(){
    robot.right_motor(TURN_SPEED);
    robot.left_motor(0);
    wait_ms(TURN_WAIT);
}


void Warehouse::go_forward(){
    robot.forward(.3);
    wait_ms(250);
}

void Warehouse::turn_180(){
    robot.left(1);
    wait_ms(120);
    robot.stop();
    wait(1);
}

bool Warehouse::is_end(){
    return q_path.empty();
}

void Warehouse::get_station_path(){
    bt.read_all(buffer, buffer_len, &chars_read);
    switch(buffer[0]){
        case '\x01': add_path(0);
        break;
        case '\x02': add_path(1);
        break;
        case '\x03': add_path(2); 
        break;
        case '\x04': add_path(3);
        break;
        case '\x05': add_path(4);
        break;
        case '\x06': add_path(5); 
        break;
    }
    buffer[0] = '0';
}

bool Warehouse::is_started(){
    return started;
}

void Warehouse::set_from(){
    bt.read_all(buffer, buffer_len, &chars_read);
    switch(buffer[0]){
        case '\x01': from = 0;  started=true; 
        break;
        case '\x02': from = 1; started=true; 
        break;
        case '\x03': from = 2; started=true; 
        break;
        case '\x04': from = 3; started=true; 
        break;
        case '\x05': from = 4;  started=true; 
        break;
        case '\x06': from = 5; started=true; 
        break;
    }
    buffer[0] = '0';
}

bool Warehouse::is_to_received(){
    return to_received;
}

void Warehouse::set_to(){
    bt.read_all(buffer, buffer_len, &chars_read);
    switch(buffer[0]){
        case '\x01': add_path(0); to_received=true; 
        break;
        case '\x02': add_path(1); to_received=true;
        break;
        case '\x03': add_path(2); to_received=true;
        break;
        case '\x04': add_path(3); to_received=true;
        break;
        case '\x05': add_path(4);  to_received=true;
        break;
        case '\x06': add_path(5);  to_received=true;
        break;
    }
    buffer[0] = '0';
}