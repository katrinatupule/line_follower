class LineFollower {
    private:

    unsigned long now;
    float last_throttle; // {0.0; 1.0} forward, if needed {-1.-; 0.0} back
    float last_steer; // {-1.0; 1.0}: -1: left; 1: right
    bool new_input;
    bool digital;

    int off_course;

    float white_th;
    float black_th;
    
    int sensor_input_count;
    int motor_phase;
    
    int slow_speed_right;
    int slow_speed_left;
    
    // uint8_t *throttle_pin_nrs;
    int *sensor_pin_nrs;
    float *last_sensor_input;

    // Motor control functions
    void forward();
    void backward();
    void left();
    void right();
    void back_left();
    void back_right();

    /*
    Write to last_sensor_input
    Update new_input flag
    */
    void read_sensor_data();
    
    /*
    Calculate next steer action based on current sensor readings
    */
    void calculate_steer2(int id_left, int id_right);
    void calculate_steer3(int id_left, int id_center, int id_right);
    void calculate_steer5();
    
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
    unsigned long timer;
    LineFollower();
    void calibrate_sensor();
    /*
    Call all functions needed for following line
    */
    void follow_line();
    void stopMotors();
    void test_motors();
};
