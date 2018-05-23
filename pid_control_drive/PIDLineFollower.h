#include "m3pi_ng.h"

class PIDLineFollower {
public:
    // Constructor for a PIDLineFollower
    // robot_in : m3pi object to be driver
    // speed_in : default uncorrected speed of motors
    // KP_in    : proportional term scalar
    // KI_in    : integral term scalar
    // KD_in    : derivative term scalar
    PIDLineFollower(m3pi &robot_in, float speed_in, float KP_in, float KI_in, float KD_in);

    // Drive robot with a PID loop with a time increment of dt.
    void drive(float dt);
    
    // Save the previous error and read the new error from the sensors.
    void update_error();
    
    // Calculate the correction to be applied to each motor.
    void calculate_correction(float dt);
    
    // Check for a bad motor command and adjust speeds if necessary.
    void check_errors();
    
    // Set the motors to the corrected speeds.
    void throttle();
    
private:
    m3pi robot;
    float KP, KI, KD;
    float line_error, error_prev;
    float integral;
    float correction;
    float BASE_SPEED, LEFT_SPEED, RIGHT_SPEED;
};