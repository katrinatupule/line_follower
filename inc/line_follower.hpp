class LineFollower {
    private:
    float last_throttle; // {0.0; 1.0} forward, if needed {-1.-; 0.0} back
    float last_steer; // {-1.0; 1.0}: -1: left; 1: right
    bool new_input;
    int sensor_input_count;
    uint8_t steer_pin_nr;
    uint8_t *throttle_pin_nrs;
    int *sensor_pin_nrs;
    float *last_sensor_input;

    /*
    Write to last_sensor_input
    Update new_input flag
    */
    void read_sensor_data();
    
    /*
    Calculate next steer action based on current sensor readings
    */
    void calculate_steer();
    
    /*
    Calculate next torque action based on current sensor readings
    */
    void calculate_throttle();
    
    /*
    send action to speed motors
    */
    void motor_control_speed();

    /*
    send action to steer motors
    */
    void motor_control_steer();

    /*
    send action to motors if no separate steer motor is used
    */
    void control_motors();
  
    public:
    LineFollower();
    void calibrate_sensor();
    /*
    Call all functions needed for following line
    */
    void follow_line();
};
